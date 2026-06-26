/** *************************************************************************************
  * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
  * Copyright (c) 2020-2021 Peng Cheng Laboratory
  *
  * XiangShan is licensed under Mulan PSL v2.
  * You can use this software according to the terms and conditions of the Mulan PSL v2.
  * You may obtain a copy of Mulan PSL v2 at:
  * http://license.coscl.org.cn/MulanPSL2
  *
  * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
  * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
  * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
  *
  * See the Mulan PSL v2 for more details.
  * *************************************************************************************
  */

package xscache.coupledL2.prefetch

import chisel3._
import chisel3.util._
import utility._
import org.chipsalliance.cde.config.Parameters
import utility.mbist.MbistPipeline
import xscache.coupledL2._
import xscache.coupledL2.utils._

/* virtual address */
trait HasPrefetcherHelper extends HasCircularQueuePtrHelper with HasCoupledL2Parameters {
  // filter
  val TRAIN_FILTER_SIZE = 4
  val REQ_FILTER_SIZE = 16
  val TLB_REPLAY_CNT = 10

  // parameters
  val BLK_ADDR_RAW_WIDTH = 10
  val REGION_SIZE = 1024
  val PAGE_OFFSET = pageOffsetBits
  val VADDR_HASH_WIDTH = 5

  // vaddr:
  // |       tag               |     index     |    offset    |
  // |       block addr                        | block offset |
  // |       region addr       |        region offset         |
  val BLOCK_OFFSET = offsetBits
  val REGION_OFFSET = log2Up(REGION_SIZE)
  val REGION_BLKS = REGION_SIZE / blockBytes
  val INDEX_BITS = log2Up(REGION_BLKS)
  val TAG_BITS = fullVAddrBits - REGION_OFFSET
  val PTAG_BITS = fullAddressBits - REGION_OFFSET
  val BLOCK_ADDR_BITS = fullVAddrBits - BLOCK_OFFSET

  // hash related
  val HASH_TAG_WIDTH = VADDR_HASH_WIDTH + BLK_ADDR_RAW_WIDTH

  def get_tag(vaddr: UInt) = {
    require(vaddr.getWidth == fullVAddrBits)
    vaddr(vaddr.getWidth - 1, REGION_OFFSET)
  }

  def get_ptag(vaddr: UInt) = {
    require(vaddr.getWidth == fullAddressBits)
    vaddr(vaddr.getWidth - 1, REGION_OFFSET)
  }

  def get_index(addr: UInt) = {
    require(addr.getWidth >= REGION_OFFSET)
    addr(REGION_OFFSET - 1, BLOCK_OFFSET)
  }

  def get_index_oh(vaddr: UInt): UInt = {
    UIntToOH(get_index(vaddr))
  }

  def get_block_vaddr(vaddr: UInt): UInt = {
    vaddr(vaddr.getWidth - 1, BLOCK_OFFSET)
  }

  def _vaddr_hash(x: UInt): UInt = {
    val width = VADDR_HASH_WIDTH
    val low = x(width - 1, 0)
    val mid = x(2 * width - 1, width)
    val high = x(3 * width - 1, 2 * width)
    low ^ mid ^ high
  }

  def block_hash_tag(vaddr: UInt): UInt = {
    val blk_addr = get_block_vaddr(vaddr)
    val low = blk_addr(BLK_ADDR_RAW_WIDTH - 1, 0)
    val high = blk_addr(BLK_ADDR_RAW_WIDTH - 1 + 3 * VADDR_HASH_WIDTH, BLK_ADDR_RAW_WIDTH)
    val high_hash = _vaddr_hash(high)
    Cat(high_hash, low)
  }

  def region_hash_tag(vaddr: UInt): UInt = {
    val region_tag = get_tag(vaddr)
    val low = region_tag(BLK_ADDR_RAW_WIDTH - 1, 0)
    val high = region_tag(BLK_ADDR_RAW_WIDTH - 1 + 3 * VADDR_HASH_WIDTH, BLK_ADDR_RAW_WIDTH)
    val high_hash = _vaddr_hash(high)
    Cat(high_hash, low)
  }

  def region_to_block_addr(tag: UInt, index: UInt): UInt = {
    Cat(tag, index)
  }

  def toBinary(n: Int): String = n match {
    case 0 | 1 => s"$n"
    case _ => s"${toBinary(n / 2)}${n % 2}"
  }
}

class PrefetchReq(implicit p: Parameters) extends PrefetchBundle {
  val tag = UInt(fullTagBits.W)
  val set = UInt(setBits.W)
  // NOTE: the vaddr is the train address for response update, not virtual address of prefetch paddr.
  val vaddr = vaddrBitsOpt.map(_ => UInt(vaddrBitsOpt.get.W))
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
  val pfSource = UInt(MemReqSource.reqSourceBits.W)

  def addr: UInt = Cat(tag, set, 0.U(offsetBits.W))
  def setaddr: UInt = Cat(tag, set)
  def isBOP:Bool = pfSource === MemReqSource.Prefetch2L2BOP.id.U
  def isPBOP:Bool = pfSource === MemReqSource.Prefetch2L2PBOP.id.U
  def isSMS:Bool = pfSource === MemReqSource.Prefetch2L2SMS.id.U
  def isTP:Bool = pfSource === MemReqSource.Prefetch2L2TP.id.U
  def isNL:Bool = pfSource === MemReqSource.Prefetch2L2NL.id.U
  def needAck:Bool = pfSource === MemReqSource.Prefetch2L2BOP.id.U || pfSource === MemReqSource.Prefetch2L2PBOP.id.U
  def fromL2:Bool =
    pfSource === MemReqSource.Prefetch2L2BOP.id.U ||
      pfSource === MemReqSource.Prefetch2L2PBOP.id.U ||
      pfSource === MemReqSource.Prefetch2L2SMS.id.U ||
      pfSource === MemReqSource.Prefetch2L2TP.id.U  ||
      pfSource === MemReqSource.Prefetch2L2NL.id.U
}

class PrefetchResp(implicit p: Parameters) extends PrefetchBundle {
  // val id = UInt(sourceIdBits.W)
  val tag = UInt(fullTagBits.W)
  val set = UInt(setBits.W)
  val vaddr = vaddrBitsOpt.map(_ => UInt(vaddrBitsOpt.get.W))
  val pfSource = UInt(MemReqSource.reqSourceBits.W)

  def addr = Cat(tag, set, 0.U(offsetBits.W))
  def isBOP: Bool = pfSource === MemReqSource.Prefetch2L2BOP.id.U
  def isPBOP: Bool = pfSource === MemReqSource.Prefetch2L2PBOP.id.U
  def isSMS: Bool = pfSource === MemReqSource.Prefetch2L2SMS.id.U
  def isTP: Bool = pfSource === MemReqSource.Prefetch2L2TP.id.U
  def isNL: Bool = pfSource === MemReqSource.Prefetch2L2NL.id.U
  def fromL2: Bool =
    pfSource === MemReqSource.Prefetch2L2BOP.id.U ||
      pfSource === MemReqSource.Prefetch2L2PBOP.id.U ||
      pfSource === MemReqSource.Prefetch2L2SMS.id.U ||
      pfSource === MemReqSource.Prefetch2L2TP.id.U  ||
      pfSource === MemReqSource.Prefetch2L2NL.id.U
}

class PrefetchTrain(implicit p: Parameters) extends PrefetchBundle {
  val tag = UInt(fullTagBits.W)
  val set = UInt(setBits.W)
  val needT = Bool()
  val source = UInt(sourceIdBits.W)
  val vaddr = vaddrBitsOpt.map(_ => UInt(vaddrBitsOpt.get.W))
  val pc = pcBitOpt.map(_ => UInt(pcBitOpt.get.W))
  val hit = Bool()
  val prefetched = Bool()
  val pfsource = UInt(PfSource.pfSourceBits.W)
  val reqsource = UInt(MemReqSource.reqSourceBits.W)

  def addr: UInt = Cat(tag, set, 0.U(offsetBits.W))
}

class PrefetchIO(implicit p: Parameters) extends PrefetchBundle {
  val train = Flipped(DecoupledIO(new PrefetchTrain))
  val tlb_req = new L2ToL1TlbIO(nRespDups= 1)
  val req = DecoupledIO(new PrefetchReq)
  val resp = Flipped(DecoupledIO(new PrefetchResp))
  val recv_addr = Flipped(ValidIO(new Bundle() {
    val addr = UInt(64.W)
    val pfSource = UInt(MemReqSource.reqSourceBits.W)
  }))
}

class PrefetchTopIO(implicit p: Parameters) extends PrefetchBundle {
  val banks = 1 << bankBits
  val train = Vec(banks, Flipped(DecoupledIO(new PrefetchTrain)))
  val tlb_req = new L2ToL1TlbIO(nRespDups= 1)
  val req = Vec(banks, DecoupledIO(new PrefetchReq))
  val resp = Vec(banks, Flipped(DecoupledIO(new PrefetchResp)))
  val recv_addr = Flipped(ValidIO(new Bundle() {
    val addr = UInt(64.W)
    val pfSource = UInt(MemReqSource.reqSourceBits.W)
  }))
}

class Prefetcher(implicit p: Parameters) extends PrefetchModule {
  val io = IO(new PrefetchTopIO)
  val tpio = IO(new Bundle() {
    val tpmeta_port = if (hasTPPrefetcher) Some(new tpmetaPortIO(hartIdLen, fullAddressBits, offsetBits)) else None
  })
  val hartId = IO(Input(UInt(hartIdLen.W)))
  val pfCtrlFromCore = IO(Input(new PrefetchCtrlFromCore))

  // l2 receive need 2 cycles to transmit from core
  val pfRcv_en = RegNextN(pfCtrlFromCore.l2_pf_master_en && pfCtrlFromCore.l2_pf_recv_en, 2, Some(true.B))
  val pbop_en = pfCtrlFromCore.l2_pf_master_en && pfCtrlFromCore.l2_pbop_en
  val vbop_en = pfCtrlFromCore.l2_pf_master_en && pfCtrlFromCore.l2_vbop_en
  val tp_en = pfCtrlFromCore.l2_pf_master_en && pfCtrlFromCore.l2_tp_en
  val delay_latency = pfCtrlFromCore.l2_pf_delay_latency
  val banks = 1 << bankBits

  // =================== Prefetchers =====================
  // TODO: consider separate VBOP and PBOP in prefetch param
  val pbop = if (hasBOP) Some(
    Module(new PBestOffsetPrefetch()(p.alterPartial({
      case L2ParamKey => p(L2ParamKey).copy(prefetch = Seq(BOPParameters(
        virtualTrain = false,
        badScore = 1,
        offsetList = Seq(
          -32, -30, -27, -25, -24, -20, -18, -16, -15,
          -12, -10, -9, -8, -6, -5, -4, -3, -2, -1,
          1, 2, 3, 4, 5, 6, 8, 9, 10,
          12, 15, 16, 18, 20, 24, 25, 27, 30
        )
      )))
    })))
  ) else None

  val vbop = if (hasBOP) Some(
    Module(new VBestOffsetPrefetch()(p.alterPartial({
      case L2ParamKey => p(L2ParamKey).copy(prefetch = Seq(BOPParameters(
        badScore = 2,
        offsetList = Seq(
          -117, -147, -91, 117, 147, 91,
          -256, -250, -243, -240, -225, -216, -200,
          -192, -180, -162, -160, -150, -144, -135, -128,
          -125, -120, -108, -100, -96, -90, -81, -80,
          -75, -72, -64, -60, -54, -50, -48, -45,
          -40, -36, -32, -30, -27, -25, -24, -20,
          -18, -16, -15, -12, -10, -9, -8, -6,
          -5, -4, -3, -2, -1,
          1, 2, 3, 4, 5, 6, 8,
          9, 10, 12, 15, 16, 18, 20, 24,
          25, 27, 30, 32, 36, 40, 45, 48,
          50, 54, 60, 64, 72, 75, 80, 81,
          90, 96, 100, 108, 120, 125, 128, 135,
          144, 150, 160, 162, 180, 192, 200, 216,
          225, 240, 243, 250 /*, 256*/
        )
      )))
    })))
  ) else None

  val tp = if (hasTPPrefetcher) Some(Module(new TemporalPrefetch())) else None
  // define Next-Line Prefetcher
  val nl = if (hasNLPrefetcher) Some(Module(new NextLinePrefetch())) else None
  // prefetch from upper level
  val pfRcv = if (hasReceiver) Some(Module(new PrefetchReceiver())) else None

  val train = Wire(DecoupledIO(new PrefetchTrain))
  val resp = Wire(DecoupledIO(new PrefetchResp))
  fastArb(io.train, train, Some("prefetch_train"))
  fastArb(io.resp, resp, Some("prefetch_resp"))

  // =================== Connection for each Prefetcher =====================
  // Rcv > NL >VBOP > PBOP > TP
  if (hasBOP) {
    vbop.get.io.enable := vbop_en
    vbop.get.io.pfCtrlOfDelayLatency := delay_latency
    vbop.get.io.train <> train
    vbop.get.io.resp <> resp
    vbop.get.io.resp.valid := resp.valid && resp.bits.isBOP
    vbop.get.io.tlb_req <> io.tlb_req
    vbop.get.io.pbopCrossPage := true.B // pbop.io.pbopCrossPage // let vbop have noting to do with pbop

    pbop.get.io.enable := pbop_en
    pbop.get.io.pfCtrlOfDelayLatency := delay_latency
    pbop.get.io.train <> train
    pbop.get.io.resp <> resp
    pbop.get.io.resp.valid := resp.valid && resp.bits.isPBOP
  }
  if (hasReceiver) {
    pfRcv.get.io_enable := pfRcv_en
    pfRcv.get.io.recv_addr := ValidIODelay(io.recv_addr, 2)
    pfRcv.get.io.train.valid := false.B
    pfRcv.get.io.train.bits := 0.U.asTypeOf(new PrefetchTrain)
    pfRcv.get.io.resp.valid := false.B
    pfRcv.get.io.resp.bits := 0.U.asTypeOf(new PrefetchResp)
    pfRcv.get.io.tlb_req.req.ready := true.B
    pfRcv.get.io.tlb_req.resp.valid := false.B
    pfRcv.get.io.tlb_req.resp.bits := DontCare
    pfRcv.get.io.tlb_req.pmp_resp := DontCare
    assert(!pfRcv.get.io.req.valid ||
      pfRcv.get.io.req.bits.pfSource === MemReqSource.Prefetch2L2SMS.id.U ||
      pfRcv.get.io.req.bits.pfSource === MemReqSource.Prefetch2L2Stream.id.U ||
      pfRcv.get.io.req.bits.pfSource === MemReqSource.Prefetch2L2Stride.id.U ||
      pfRcv.get.io.req.bits.pfSource === MemReqSource.Prefetch2L2Berti.id.U
    )
  }

  if (hasNLPrefetcher) {
    nl.get.io.enable := true.B
    nl.get.io.train <> train
    nl.get.io.resp <> resp
  }

  if (hasTPPrefetcher) {
    tp.get.io.enable := tp_en
    tp.get.io.train <> train
    tp.get.io.resp <> resp
    tp.get.io.hartid := hartId

    tp.get.io.tpmeta_port <> tpio.tpmeta_port.get
  }
  private val mbistPl = MbistPipeline.PlaceMbistPipeline(2, "MbistPipeL2Prefetcher", cacheParams.hasMbist && (hasBOP || hasTPPrefetcher))

  // =================== Connection of all Prefetchers =====================
  /* prefetchers -> pftQueue -> pipe -> Slices.SinkA */
  private val SRC_NUM = 5
  private val Seq(rcv_idx, nl_idx, vbop_idx, pbop_idx, tp_idx) = (0 until SRC_NUM).toSeq
  val reqs = Seq(
    if (hasReceiver) Some(pfRcv.get.io.req) else None,
    if (hasNLPrefetcher) Some(nl.get.io.req) else None,
    if (hasBOP) Some(vbop.get.io.req) else None,
    if (hasBOP) Some(pbop.get.io.req) else None,
    if (hasTPPrefetcher) Some(tp.get.io.req) else None
  )
  val reqsValid = reqs.map(_.map(_.valid).getOrElse(false.B))
  val reqsBits = reqs.map(_.map(_.bits).getOrElse(0.U.asTypeOf(new PrefetchReq)))
  val reqsSetAddr = reqsBits.map(_.setaddr)
  val pftQueue = Seq.tabulate(banks) { _ =>
    Module(new OverwriteQueue(
      gen = new PrefetchReq,
      entries = inflightEntries,
      hasFlow = true
    ))
  }
  val pipe = Seq.tabulate(banks) { _ => Module(new Pipeline(new PrefetchReq, 1)) }
  val select = Wire(Vec(banks, Vec(SRC_NUM, Bool())))
  val selectOH = Wire(Vec(banks, Vec(SRC_NUM, Bool())))

  for (i <- 0 until banks) {
    select(i) := VecInit(reqsValid.zip(reqsSetAddr).map {
      case (valid, addr) => valid && bank_eq(addr, i, bankBits)
    })
    selectOH(i) := VecInit(PriorityEncoderOH(select(i).asUInt).asBools)
    pftQueue(i).io.enq.valid := select(i).asUInt.orR
    pftQueue(i).io.enq.bits := ParallelPriorityMux(select(i).asUInt, reqsBits)
    pipe(i).io.in <> pftQueue(i).io.deq
    io.req(i) <> pipe(i).io.out
  }

  for ((reqOpt, j) <- reqs.zipWithIndex) {
    reqOpt.foreach { req =>
      req.ready := (0 until banks).map(i => selectOH(i)(j)).reduce(_ || _)
    }
  }

  val reqsFire = reqs.map(_.map(_.fire).getOrElse(false.B))

  XSPerfAccumulate("prefetch_train_valid", train.valid)
  XSPerfAccumulate("prefetch_train_in_valid", PopCount(io.train.map(_.valid)))
  XSPerfAccumulate("prefetch_resp_valid", resp.valid)
  XSPerfAccumulate("prefetch_resp_in_valid", PopCount(io.resp.map(_.valid)))
  XSPerfAccumulate("prefetch_req_fromL1", reqsValid(rcv_idx))
  XSPerfAccumulate("prefetch_req_fromVBOP", reqsValid(vbop_idx))
  XSPerfAccumulate("prefetch_req_fromPBOP", reqsValid(pbop_idx))
  XSPerfAccumulate("prefetch_req_fromBOP", reqsValid(vbop_idx) || reqsValid(pbop_idx))
  XSPerfAccumulate("prefetch_req_fromTP", reqsValid(tp_idx))
  XSPerfAccumulate("prefetch_req_fromNL", reqsValid(nl_idx))

  XSPerfAccumulate("prefetch_req_selectL1", reqsFire(rcv_idx))
  XSPerfAccumulate("prefetch_req_selectVBOP", reqsFire(vbop_idx))
  XSPerfAccumulate("prefetch_req_selectPBOP", reqsFire(pbop_idx))
  XSPerfAccumulate("prefetch_req_selectBOP", reqsFire(vbop_idx) || reqsFire(pbop_idx))
  XSPerfAccumulate("prefetch_req_selectTP", reqsFire(tp_idx))
  XSPerfAccumulate("prefetch_req_selectNL", reqsFire(nl_idx))
  XSPerfAccumulate("prefetch_req_SMS_other_overlapped",
    reqsValid(rcv_idx) &&
      (reqsValid(vbop_idx) || reqsValid(pbop_idx) || reqsValid(tp_idx) || reqsValid(nl_idx))
  )

  // NOTE: set basicDB false when debug over
  // TODO: change the enable signal to not target the BOP
  class TrainEntry extends Bundle{
    val paddr = UInt(fullAddressBits.W)
    val vaddr = UInt(fullVAddrBits.W)
    val needT = Bool()
    val hit = Bool()
    val prefetched = Bool()
    val source = UInt(sourceIdBits.W)
    val pfsource = UInt(PfSource.pfSourceBits.W)
    val reqsource = UInt(MemReqSource.reqSourceBits.W)
  }
  val trainTT = ChiselDB.createTable("L2PrefetchTrainTable", new TrainEntry, basicDB = false)
  val e1 = Wire(new TrainEntry)
  e1.paddr := train.bits.addr
  e1.vaddr := train.bits.vaddr.getOrElse(0.U) << offsetBits
  e1.needT := train.bits.needT
  e1.hit := train.bits.hit
  e1.prefetched := train.bits.prefetched
  e1.source := train.bits.source
  e1.pfsource := train.bits.pfsource
  e1.reqsource := train.bits.reqsource
  trainTT.log(
    data = e1,
    en = train.valid,
    site = "L2Train",
    clock, reset
  )

  class PrefetchEntry extends Bundle{
    val paddr = UInt(fullAddressBits.W)
    val needT = Bool()
    val pfsource = UInt(MemReqSource.reqSourceBits.W)
  }
  val pfTT = ChiselDB.createTable("L2PrefetchReqTable", new PrefetchEntry, basicDB = false)
  for (i <- 0 until banks) {
    val e2 = Wire(new PrefetchEntry)
    e2.paddr := io.req(i).bits.addr
    e2.needT := io.req(i).bits.needT
    e2.pfsource := io.req(i).bits.pfSource
    pfTT.log(
      data = e2,
      en = io.req(i).fire,
      site = "L2PrefetchReq",
      clock, reset
    )
  }
}
