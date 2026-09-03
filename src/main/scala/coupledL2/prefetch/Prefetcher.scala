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
  def isMatrix: Bool = pfSource === PfSource.matrixMemReqSource.id.U
  def needAck:Bool = pfSource === MemReqSource.Prefetch2L2BOP.id.U || pfSource === MemReqSource.Prefetch2L2PBOP.id.U
  def fromL2:Bool =
    pfSource === MemReqSource.Prefetch2L2BOP.id.U ||
      pfSource === MemReqSource.Prefetch2L2PBOP.id.U ||
      pfSource === MemReqSource.Prefetch2L2SMS.id.U ||
      pfSource === MemReqSource.Prefetch2L2TP.id.U  ||
      pfSource === MemReqSource.Prefetch2L2NL.id.U ||
      pfSource === PfSource.matrixMemReqSource.id.U
}

private object FullPriorityOneHot {
  def apply(bits: Seq[Bool]): UInt = {
    val seen = Wire(Vec(bits.length + 1, Bool()))
    val oneHot = Wire(Vec(bits.length, Bool()))
    seen(0) := false.B
    bits.zipWithIndex.foreach { case (bit, i) =>
      oneHot(i) := bit && !seen(i)
      seen(i + 1) := seen(i) || bit
    }
    oneHot.asUInt
  }
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
  def isMatrix: Bool = pfSource === PfSource.matrixMemReqSource.id.U
  def fromL2: Bool =
    pfSource === MemReqSource.Prefetch2L2BOP.id.U ||
      pfSource === MemReqSource.Prefetch2L2PBOP.id.U ||
      pfSource === MemReqSource.Prefetch2L2SMS.id.U ||
      pfSource === MemReqSource.Prefetch2L2TP.id.U  ||
      pfSource === MemReqSource.Prefetch2L2NL.id.U ||
      pfSource === PfSource.matrixMemReqSource.id.U
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
  val matrixPrefetchTag = Option.when(enableMatrix)(UInt(MatrixPrefetchTagCodec.width.W))

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
  val matrixPrefetch = Input(new MatrixPrefetchControl)
}

/** Walks the cache lines touched by a CUTE strided matrix load.
  *
  * CUTE issues 32-byte TileLink requests while L2 caches 64-byte blocks.  The
  * generator therefore works from the byte range of each row instead of the
  * number of CUTE request beats.  Within a CUTE register group, the loader
  * visits the same K beat across all M rows before moving to the next K beat;
  * the cache-line order below deliberately preserves that traversal.
  */
class MatrixPrefetchAddressGenerator(blockBytes: Int = 64) extends Module {
  require(isPow2(blockBytes))
  private val offsetBits = log2Ceil(blockBytes)

  val io = IO(new Bundle {
    val load = Input(Valid(new MatrixPrefetchDesc))
    val step = Input(Bool())
    val address = Output(UInt(64.W))
    val valid = Output(Bool())
    val done = Output(Bool())
  })

  private def alignBlock(address: UInt): UInt =
    Cat(address(63, offsetBits), 0.U(offsetBits.W))

  val running = RegInit(false.B)
  val baseAddr = RegInit(0.U(64.W))
  val rowBytes = RegInit(0.U(32.W))
  val outerStride = RegInit(0.U(64.W))
  val outerCount = RegInit(0.U(16.W))
  val groupWidth = RegInit(0.U(16.W))
  val groupStart = RegInit(0.U(16.W))
  val lineIndex = RegInit(0.U(32.W))
  val rowInGroup = RegInit(0.U(16.W))

  val rowIndex = groupStart + rowInGroup
  val currentRowBase = baseAddr + rowIndex * outerStride
  val currentAddress = alignBlock(currentRowBase) + (lineIndex << offsetBits)
  val currentRowEnd = currentRowBase + rowBytes
  val currentValid = running && rowIndex < outerCount && currentAddress < currentRowEnd

  // A block-aligned stride keeps every row at the base address's alignment,
  // which is the normal CUTE case.  Otherwise use a safe upper bound and skip
  // candidates beyond each particular row's exact end.
  val exactAlignedStrideLines =
    (baseAddr(offsetBits - 1, 0) + rowBytes + (blockBytes - 1).U) >> offsetBits
  val alignmentIndependentMaxLines =
    1.U + ((rowBytes + (blockBytes - 2).U) >> offsetBits)
  val maxLinesPerRow = Mux(
    outerStride(offsetBits - 1, 0) === 0.U,
    exactAlignedStrideLines,
    alignmentIndependentMaxLines
  )
  val rowsRemaining = outerCount - groupStart
  val rowsInGroup = Mux(rowsRemaining < groupWidth, rowsRemaining, groupWidth)
  val lastRowInGroup = rowInGroup + 1.U >= rowsInGroup
  val lastLineInGroup = lineIndex + 1.U >= maxLinesPerRow

  when(io.load.valid) {
    running := io.load.bits.outerCount =/= 0.U &&
      io.load.bits.rowBytes =/= 0.U && io.load.bits.groupWidth =/= 0.U
    baseAddr := io.load.bits.baseAddr
    rowBytes := io.load.bits.rowBytes
    outerStride := io.load.bits.outerStride
    outerCount := io.load.bits.outerCount
    groupWidth := io.load.bits.groupWidth
    groupStart := 0.U
    lineIndex := 0.U
    rowInGroup := 0.U
  }.elsewhen(running && (io.step || !currentValid)) {
    when(!lastRowInGroup) {
      rowInGroup := rowInGroup + 1.U
    }.otherwise {
      rowInGroup := 0.U
      when(!lastLineInGroup) {
        lineIndex := lineIndex + 1.U
      }.otherwise {
        lineIndex := 0.U
        when(groupStart + groupWidth >= outerCount) {
          running := false.B
        }.otherwise {
          groupStart := groupStart + groupWidth
        }
      }
      when(lastLineInGroup && groupStart + groupWidth >= outerCount) {
        running := false.B
      }
    }
  }

  io.address := currentAddress
  io.valid := currentValid
  io.done := !running
}

class MatrixGuidedPrefetcher(implicit p: Parameters) extends PrefetchModule {
  private val entries = 16
  private val banks = 1 << bankBits

  val io = IO(new Bundle {
    val enable = Input(Bool())
    val enableBOnA = Input(Bool())
    val enableCOnB = Input(Bool())
    val control = Input(new MatrixPrefetchControl)
    val demand = Input(Vec(banks, Valid(new PrefetchTrain)))
    val bankReady = Input(Vec(banks, Bool()))
    val bankBestEffortReady = Input(Vec(banks, Bool()))
    val req = DecoupledIO(new PrefetchReq)
  })

  val entryValid = RegInit(VecInit(Seq.fill(entries)(false.B)))
  val entryActive = RegInit(VecInit(Seq.fill(entries)(false.B)))
  val descs = Reg(Vec(entries, new MatrixPrefetchDesc))
  val demandCount = RegInit(VecInit(Seq.fill(entries)(0.U(32.W))))
  val issuedCount = RegInit(VecInit(Seq.fill(entries)(0.U(32.W))))
  val addressGenerators = Seq.fill(entries)(Module(new MatrixPrefetchAddressGenerator(blockBytes)))

  val cStoreActive = RegInit(false.B)
  val cStoreTaskId = RegInit(0.U(MatrixPrefetchTagCodec.taskIdWidth.W))

  // A normal load descriptor retires before its following C store starts in
  // row-batched GEMM code.  Accumulate the complete K span described by all A
  // loads preceding a C store.  A large-K tile is represented by several
  // adjacent descriptors (for example sixteen 128x64 loads for K=1024), and
  // retaining only the final descriptor would prefetch just 1/K of the next
  // A tile row.
  val aSpanValid = RegInit(false.B)
  val aSpanBase = RegInit(0.U(64.W))
  val aSpanEnd = RegInit(0.U(64.W))
  val aSpanDesc = Reg(new MatrixPrefetchDesc)
  // CUTE allocates the B descriptors beside their A descriptors.  Keep a
  // short history of their bases as a conservative A-buffer boundary: an A
  // stride lookahead which lands on a known B base is past the final A row,
  // not a valid next-A request.
  val bBaseHistory = Reg(Vec(entries, UInt(64.W)))
  val bBaseHistoryValid = RegInit(VecInit(Seq.fill(entries)(false.B)))
  val bBaseHistoryWrite = RegInit(0.U(log2Ceil(entries).W))
  val lowestBBaseValid = RegInit(false.B)
  val lowestBBase = RegInit(0.U(64.W))
  val lastLookaheadSourceValid = RegInit(false.B)
  val lastLookaheadSourceBase = RegInit(0.U(64.W))
  val lastLookaheadSourceEnd = RegInit(0.U(64.W))
  val lastLookaheadABaseValid = RegInit(false.B)
  val lastLookaheadABase = RegInit(0.U(64.W))
  val cStoreLookaheadActive = RegInit(false.B)
  val cStoreLookaheadIssued = RegInit(0.U(32.W))
  val cStoreLookaheadPending = RegInit(false.B)
  val cStoreLookaheadPendingDesc = Reg(new MatrixPrefetchDesc)
  val cStoreLookaheadGenerator = Module(new MatrixPrefetchAddressGenerator(blockBytes))

  // CUTE allocates and activates the paired A_i/B_i tasks in different cycles
  // depending on issue-window pressure.  Keep both events in task-ID-indexed
  // tables and join them after either arrival order.  The live issue window
  // is much smaller than 64 tasks, so the low task-ID bits provide a compact
  // index while the full ID check prevents aliasing after wraparound.
  private val bOnAPairEntries = 64
  private val bOnAPairIndexBits = log2Ceil(bOnAPairEntries)
  val aWaitingForBValid = RegInit(VecInit(Seq.fill(bOnAPairEntries)(false.B)))
  val aWaitingForBTask = Reg(Vec(bOnAPairEntries, UInt(MatrixPrefetchTagCodec.taskIdWidth.W)))
  val bPairValid = RegInit(VecInit(Seq.fill(bOnAPairEntries)(false.B)))
  val bPairDesc = Reg(Vec(bOnAPairEntries, new MatrixPrefetchDesc))
  val bPairRunBase = Reg(Vec(bOnAPairEntries, UInt(64.W)))

  // A descriptor-wide de-duplication table prevents the four M tile rows
  // from issuing the same B slice four times.  It is large enough for the
  // complete set of distinct B K-slices in the 512x512 output workload.
  private val bOnASeenEntries = 128
  val bOnASeenValid = RegInit(VecInit(Seq.fill(bOnASeenEntries)(false.B)))
  val bOnASeenBase = Reg(Vec(bOnASeenEntries, UInt(64.W)))
  val bOnASeenWrite = RegInit(0.U(log2Ceil(bOnASeenEntries).W))

  private val bOnAQueueDepthPerBank = 8
  private val bOnAWalkers = banks
  private val bOnAWalkerIndexBits = log2Ceil(math.max(bOnAWalkers, 2))
  val bOnALookaheadQueues =
    Seq.fill(banks)(Module(new Queue(new MatrixPrefetchDesc, bOnAQueueDepthPerBank)))
  val bOnALookaheadGenerators =
    Seq.fill(bOnAWalkers)(Module(new MatrixPrefetchAddressGenerator(blockBytes)))
  val bOnALookaheadActive = RegInit(VecInit(Seq.fill(bOnAWalkers)(false.B)))
  val bOnALookaheadIssued = RegInit(VecInit(Seq.fill(bOnAWalkers)(0.U(32.W))))
  val bOnACurrentDesc = Reg(Vec(bOnAWalkers, new MatrixPrefetchDesc))

  // C loads are much larger than an A/B slice.  Predict only the next C tile
  // in the same matrix row, start it on the first B load, and abandon the
  // unfinished suffix as soon as the predicted C demand begins.  This avoids
  // carrying speculative C traffic into the next tile and also avoids an
  // unsafe prediction beyond the final M tile row.
  val currentCDescValid = RegInit(false.B)
  val currentCDesc = Reg(new MatrixPrefetchDesc)
  val currentCRunBase = RegInit(0.U(64.W))
  val lastCDescValid = RegInit(false.B)
  val lastCBase = RegInit(0.U(64.W))
  val lastCRowBytes = RegInit(0.U(32.W))
  val cOnBArmed = RegInit(false.B)
  val cOnBLookaheadActive = RegInit(false.B)
  val cOnBLookaheadIssued = RegInit(0.U(32.W))
  val cOnBRateCounter = RegInit(0.U(3.W))
  val cOnBLookaheadGenerator = Module(new MatrixPrefetchAddressGenerator(blockBytes))

  // Consecutive B descriptors within one tile advance by rowBytes.  Remember
  // the first descriptor of that run so the final K slice can be recognized:
  // base + rowBytes == runBase + outerStride.  Skipping that boundary avoids
  // predicting into the next physical B row when the following software task
  // actually jumps to another 128-row B tile.
  val bRunBase = RegInit(0.U(64.W))
  val lastBDescValid = RegInit(false.B)
  val lastBBase = RegInit(0.U(64.W))
  val lastBRowBytes = RegInit(0.U(32.W))

  val allocIsB = io.control.allocate.valid &&
    io.control.allocate.bits.stream === MatrixPrefetchStream.b
  val allocIsC = io.control.allocate.valid &&
    io.control.allocate.bits.stream === MatrixPrefetchStream.cLoad
  val bStartsNewRun = !lastBDescValid ||
    io.control.allocate.bits.baseAddr =/= lastBBase + lastBRowBytes
  val effectiveBRunBase = Mux(bStartsNewRun, io.control.allocate.bits.baseAddr, bRunBase)
  val activateIsA = io.control.activate.valid &&
    MatrixPrefetchTagCodec.valid(io.control.activate.bits) &&
    MatrixPrefetchTagCodec.stream(io.control.activate.bits) === MatrixPrefetchStream.a
  val activateIsB = io.control.activate.valid &&
    MatrixPrefetchTagCodec.valid(io.control.activate.bits) &&
    MatrixPrefetchTagCodec.stream(io.control.activate.bits) === MatrixPrefetchStream.b
  val activateIsC = io.control.activate.valid &&
    MatrixPrefetchTagCodec.valid(io.control.activate.bits) &&
    MatrixPrefetchTagCodec.stream(io.control.activate.bits) === MatrixPrefetchStream.cLoad
  val activateATaskId = MatrixPrefetchTagCodec.taskId(io.control.activate.bits)
  val activateAIdx = activateATaskId(bOnAPairIndexBits - 1, 0)
  val bAllocTaskId = io.control.allocate.bits.taskId
  val bAllocIdx = bAllocTaskId(bOnAPairIndexBits - 1, 0)

  val cStartsNewRun = !lastCDescValid ||
    io.control.allocate.bits.baseAddr =/= lastCBase + lastCRowBytes
  val effectiveCRunBase = Mux(cStartsNewRun, io.control.allocate.bits.baseAddr, currentCRunBase)
  val nextCBase = currentCDesc.baseAddr + currentCDesc.rowBytes
  val currentCHasNextInRow = currentCDescValid && currentCDesc.rowBytes =/= 0.U &&
    currentCDesc.outerStride =/= 0.U &&
    nextCBase < currentCRunBase + currentCDesc.outerStride
  val cOnBLookaheadDesc = WireInit(currentCDesc)
  cOnBLookaheadDesc.baseAddr := nextCBase
  cOnBLookaheadDesc.stream := MatrixPrefetchStream.cLoad
  val cOnBLookaheadCandidate = io.enable && io.enableCOnB && activateIsB && cOnBArmed &&
    currentCHasNextInRow && !currentCDesc.transpose
  val cOnBBoundarySkip = io.enable && io.enableCOnB && activateIsB && cOnBArmed &&
    (!currentCHasNextInRow || currentCDesc.transpose)

  cOnBLookaheadGenerator.io.load.valid := cOnBLookaheadCandidate
  cOnBLookaheadGenerator.io.load.bits := cOnBLookaheadDesc
  cOnBLookaheadGenerator.io.step := false.B

  when(allocIsC) {
    currentCDescValid := true.B
    currentCDesc := io.control.allocate.bits
    currentCRunBase := effectiveCRunBase
    lastCDescValid := true.B
    lastCBase := io.control.allocate.bits.baseAddr
    lastCRowBytes := io.control.allocate.bits.rowBytes
    cOnBArmed := true.B
  }

  when(cOnBLookaheadCandidate || cOnBBoundarySkip) {
    cOnBArmed := false.B
  }

  when(cOnBLookaheadCandidate) {
    cOnBLookaheadActive := true.B
    cOnBLookaheadIssued := 0.U
    cOnBRateCounter := 0.U
  }.elsewhen(activateIsC && cOnBLookaheadActive) {
    // The target C load is now demand traffic. Do not retain an unfinished
    // speculative suffix past its useful window.
    cOnBLookaheadActive := false.B
    cOnBRateCounter := 0.U
  }.elsewhen(cOnBLookaheadActive && cOnBLookaheadGenerator.io.done) {
    cOnBLookaheadActive := false.B
    cOnBRateCounter := 0.U
  }.elsewhen(cOnBLookaheadActive) {
    cOnBRateCounter := cOnBRateCounter + 1.U
  }

  val pairMatch = VecInit((0 until bOnAPairEntries).map { i =>
    val pairedBTask = aWaitingForBTask(i) + 1.U
    val pairedBIdx = pairedBTask(bOnAPairIndexBits - 1, 0)
    aWaitingForBValid(i) && bPairValid(pairedBIdx) &&
      bPairDesc(pairedBIdx).taskId === pairedBTask
  })
  val pairOH = FullPriorityOneHot(pairMatch.toSeq)
  val pairValid = pairMatch.asUInt.orR
  val pairAIdx = OHToUInt(pairOH)
  val pairBTask = aWaitingForBTask(pairAIdx) + 1.U
  val pairBIdx = pairBTask(bOnAPairIndexBits - 1, 0)
  val selectedBDesc = bPairDesc(pairBIdx)
  val selectedBRunBase = bPairRunBase(pairBIdx)

  // A/B/MMA occupy three consecutive task IDs inside a K-slice.  The
  // synthetic descriptor walks the next B K-slice, not the currently active
  // one, so it has useful lead time before the demand B task.
  val bOnALookaheadDesc = WireInit(selectedBDesc)
  val selectedBNextBase = selectedBDesc.baseAddr + selectedBDesc.rowBytes
  val selectedBHasNext = selectedBDesc.rowBytes =/= 0.U &&
    selectedBDesc.outerStride =/= 0.U &&
    selectedBNextBase < selectedBRunBase + selectedBDesc.outerStride
  bOnALookaheadDesc.baseAddr := selectedBNextBase
  bOnALookaheadDesc.taskId := selectedBDesc.taskId + 3.U

  val bOnALookaheadRawCandidate = io.enable && io.enableBOnA && pairValid &&
    !selectedBDesc.transpose && selectedBHasNext
  val bOnASeen = VecInit((0 until bOnASeenEntries).map { i =>
    bOnASeenValid(i) && bOnASeenBase(i) === bOnALookaheadDesc.baseAddr
  }).asUInt.orR
  val bOnALookaheadCandidate = bOnALookaheadRawCandidate && !bOnASeen

  val bOnALookaheadDescBank = if (bankBits == 0) 0.U else
    (bOnALookaheadDesc.baseAddr >> offsetBits)(bankBits - 1, 0)
  for (i <- 0 until banks) {
    bOnALookaheadQueues(i).io.enq.valid :=
      bOnALookaheadCandidate && bOnALookaheadDescBank === i.U
    bOnALookaheadQueues(i).io.enq.bits := bOnALookaheadDesc
  }
  val bOnALookaheadEnqFire = bOnALookaheadQueues.map(_.io.enq.fire).reduce(_ || _)
  val bOnALookaheadTargetReady = (0 until banks).map { i =>
    bOnALookaheadDescBank === i.U && bOnALookaheadQueues(i).io.enq.ready
  }.reduce(_ || _)
  // Boundary and duplicate pairs have no request to enqueue and can retire
  // immediately.  A real candidate retires only after the FIFO accepts it,
  // preserving the descriptor under backpressure.
  val consumeBOnAPair = pairValid && (
    !io.enable || !io.enableBOnA || selectedBDesc.transpose || !selectedBHasNext ||
      bOnASeen || bOnALookaheadEnqFire
  )
  when(consumeBOnAPair) {
    aWaitingForBValid(pairAIdx) := false.B
    bPairValid(pairBIdx) := false.B
  }

  when(activateIsA) {
    assert(!aWaitingForBValid(activateAIdx) || consumeBOnAPair && pairAIdx === activateAIdx,
      "Matrix A activation waiting-table task-ID alias")
    aWaitingForBValid(activateAIdx) := true.B
    aWaitingForBTask(activateAIdx) := activateATaskId
  }
  when(allocIsB) {
    assert(!bPairValid(bAllocIdx) || consumeBOnAPair && pairBIdx === bAllocIdx,
      "Matrix B descriptor pairing-table task-ID alias")
    bPairValid(bAllocIdx) := true.B
    bPairDesc(bAllocIdx) := io.control.allocate.bits
    bPairRunBase(bAllocIdx) := effectiveBRunBase
  }
  when(bOnALookaheadEnqFire) {
    bOnASeenValid(bOnASeenWrite) := true.B
    bOnASeenBase(bOnASeenWrite) := bOnALookaheadDesc.baseAddr
    bOnASeenWrite := bOnASeenWrite + 1.U
  }

  // Keep several B descriptors active and issue from any walker whose current
  // target bank has room.  A stalled bank can no longer hold unrelated B
  // descriptors behind a single address generator.
  val bOnALookaheadCanLoad = VecInit((0 until bOnAWalkers).map { i =>
    !bOnALookaheadActive(i) || bOnALookaheadGenerators(i).io.done
  })
  for (i <- 0 until bOnAWalkers) {
    bOnALookaheadQueues(i).io.deq.ready :=
      io.enable && io.enableBOnA && bOnALookaheadCanLoad(i)
    val loadWalker = bOnALookaheadQueues(i).io.deq.fire
    bOnALookaheadGenerators(i).io.load.valid := loadWalker
    bOnALookaheadGenerators(i).io.load.bits := bOnALookaheadQueues(i).io.deq.bits
    bOnALookaheadGenerators(i).io.step := false.B

    when(loadWalker) {
      bOnALookaheadActive(i) := true.B
      bOnALookaheadIssued(i) := 0.U
      bOnACurrentDesc(i) := bOnALookaheadQueues(i).io.deq.bits
    }.elsewhen(bOnALookaheadActive(i) && bOnALookaheadGenerators(i).io.done) {
      bOnALookaheadActive(i) := false.B
    }
  }

  when(allocIsB) {
    when(bStartsNewRun) {
      bRunBase := io.control.allocate.bits.baseAddr
    }
    lastBDescValid := true.B
    lastBBase := io.control.allocate.bits.baseAddr
    lastBRowBytes := io.control.allocate.bits.rowBytes
  }

  val aTileByteSpan = (aSpanDesc.outerStride * aSpanDesc.outerCount)(63, 0)
  val nextAFullBase = (aSpanBase + aTileByteSpan)(63, 0)
  val fullARowBytes = (aSpanEnd - aSpanBase)(31, 0)
  val cStoreLookaheadDesc = WireInit(aSpanDesc)
  cStoreLookaheadDesc.baseAddr := nextAFullBase
  cStoreLookaheadDesc.rowBytes := fullARowBytes
  cStoreLookaheadDesc.taskId := MatrixPrefetchTagCodec.taskId(io.control.cStoreStart.bits)
  cStoreLookaheadDesc.stream := MatrixPrefetchStream.a

  val cStoreLookaheadHitsKnownB = VecInit((0 until entries).map { i =>
    bBaseHistoryValid(i) && cStoreLookaheadDesc.baseAddr === bBaseHistory(i)
  }).asUInt.orR || (lowestBBaseValid && cStoreLookaheadDesc.baseAddr >= lowestBBase)
  // Later C stores in the same M tile row may see only a suffix of the A
  // descriptors because the issue window has already consumed the prefix.
  // Treat every subspan of the row which launched the current lookahead as
  // the same source row; otherwise a suffix would be mistaken for a shifted
  // next-A target.
  val sameLookaheadSource = lastLookaheadSourceValid &&
    aSpanBase >= lastLookaheadSourceBase && aSpanEnd <= lastLookaheadSourceEnd

  val cStoreLookaheadCandidate = io.enable && io.control.cStoreStart.valid &&
    aSpanValid && !aSpanDesc.transpose && fullARowBytes =/= 0.U &&
    !cStoreLookaheadHitsKnownB && !sameLookaheadSource &&
    (!lastLookaheadABaseValid || lastLookaheadABase =/= nextAFullBase)

  // A large next-A row can take several C-store windows to emit.  Preserve a
  // newly discovered row while the current generator finishes instead of
  // overwriting the unfinished walk at the next C-store boundary.
  val cStoreLookaheadCanLoad = !cStoreLookaheadActive || cStoreLookaheadGenerator.io.done
  val cStoreLookaheadLoadPending = cStoreLookaheadCanLoad && cStoreLookaheadPending
  val cStoreLookaheadLoadCandidate =
    cStoreLookaheadCanLoad && !cStoreLookaheadPending && cStoreLookaheadCandidate
  val cStoreLookaheadStart = cStoreLookaheadLoadPending || cStoreLookaheadLoadCandidate
  val cStoreLookaheadDefer = cStoreLookaheadCandidate && !cStoreLookaheadCanLoad

  cStoreLookaheadGenerator.io.load.valid := cStoreLookaheadStart
  cStoreLookaheadGenerator.io.load.bits := Mux(
    cStoreLookaheadLoadPending,
    cStoreLookaheadPendingDesc,
    cStoreLookaheadDesc
  )
  cStoreLookaheadGenerator.io.step := false.B

  when(cStoreLookaheadDefer) {
    assert(!cStoreLookaheadPending,
      "Matrix prefetch pending A-row descriptor overflow")
    when(!cStoreLookaheadPending) {
      cStoreLookaheadPending := true.B
      cStoreLookaheadPendingDesc := cStoreLookaheadDesc
    }
  }

  when(cStoreLookaheadLoadPending) {
    cStoreLookaheadPending := false.B
  }

  when(cStoreLookaheadStart) {
    cStoreLookaheadActive := true.B
    cStoreLookaheadIssued := 0.U
  }.elsewhen(cStoreLookaheadActive && cStoreLookaheadGenerator.io.done) {
    cStoreLookaheadActive := false.B
  }

  when(cStoreLookaheadLoadCandidate || cStoreLookaheadDefer) {
    lastLookaheadSourceValid := true.B
    lastLookaheadSourceBase := aSpanBase
    lastLookaheadSourceEnd := aSpanEnd
    lastLookaheadABaseValid := true.B
    lastLookaheadABase := nextAFullBase
  }

  private def isCStoreTag(tag: UInt): Bool =
    MatrixPrefetchTagCodec.valid(tag) &&
      MatrixPrefetchTagCodec.stream(tag) === MatrixPrefetchStream.cStore

  private def isYoungerTask(candidate: UInt, reference: UInt): Bool = {
    val distance = candidate - reference
    distance =/= 0.U && !distance(MatrixPrefetchTagCodec.taskIdWidth - 1)
  }

  when(io.control.cStoreStart.valid) {
    assert(isCStoreTag(io.control.cStoreStart.bits),
      "Matrix prefetch C store start has an invalid tag")
    assert(!cStoreActive, "Matrix prefetch observed overlapping C stores")
    cStoreActive := true.B
    cStoreTaskId := MatrixPrefetchTagCodec.taskId(io.control.cStoreStart.bits)
    // Descriptors allocated after this point belong to a later tile.  The
    // current span has already been captured by cStoreLookaheadDesc.
    aSpanValid := false.B
  }

  when(io.control.cStoreEnd.valid) {
    assert(isCStoreTag(io.control.cStoreEnd.bits),
      "Matrix prefetch C store end has an invalid tag")
    assert(cStoreActive, "Matrix prefetch C store ended while inactive")
    assert(MatrixPrefetchTagCodec.taskId(io.control.cStoreEnd.bits) === cStoreTaskId,
      "Matrix prefetch C store start/end task IDs differ")
    cStoreActive := false.B
  }

  private def tagMatches(tag: UInt, desc: MatrixPrefetchDesc): Bool =
    MatrixPrefetchTagCodec.valid(tag) &&
      MatrixPrefetchTagCodec.taskId(tag) === desc.taskId &&
      MatrixPrefetchTagCodec.stream(tag) === desc.stream

  val retireMatch = VecInit((0 until entries).map { i =>
    io.control.retire.valid && entryValid(i) && tagMatches(io.control.retire.bits, descs(i))
  })
  val allocMatch = VecInit((0 until entries).map { i =>
    io.control.allocate.valid && entryValid(i) &&
      io.control.allocate.bits.taskId === descs(i).taskId &&
      io.control.allocate.bits.stream === descs(i).stream
  })
  val reusable = VecInit((0 until entries).map(i => !entryValid(i) || retireMatch(i)))
  // The UInt overload intentionally omits the implicit first choice and is
  // therefore one bit narrower.  Keep the full one-hot width by selecting the
  // Seq[Bool] overload; every descriptor entry must remain addressable.
  val allocMatchOH = FullPriorityOneHot(allocMatch.toSeq)
  val reusableOH = FullPriorityOneHot(reusable.toSeq)
  val allocOH = Mux(allocMatch.asUInt.orR, allocMatchOH, reusableOH)

  for (i <- 0 until entries) {
    addressGenerators(i).io.load.valid := io.control.allocate.valid && allocOH(i)
    addressGenerators(i).io.load.bits := io.control.allocate.bits
    addressGenerators(i).io.step := false.B
  }

  when(io.control.retire.valid) {
    for (i <- 0 until entries) {
      when(retireMatch(i)) {
        entryValid(i) := false.B
        entryActive(i) := false.B
      }
    }
  }

  when(io.control.allocate.valid) {
    assert(reusable.asUInt.orR || allocMatch.asUInt.orR,
      "MatrixGuidedPrefetcher descriptor table overflow")
    for (i <- 0 until entries) {
      when(allocOH(i)) {
        entryValid(i) := true.B
        entryActive(i) := false.B
        descs(i) := io.control.allocate.bits
        demandCount(i) := 0.U
        issuedCount(i) := 0.U
      }
    }
    when(io.control.allocate.bits.stream === MatrixPrefetchStream.a) {
      val descEnd = io.control.allocate.bits.baseAddr + io.control.allocate.bits.rowBytes
      when(!aSpanValid) {
        aSpanValid := true.B
        aSpanBase := io.control.allocate.bits.baseAddr
        aSpanEnd := descEnd
        aSpanDesc := io.control.allocate.bits
      }.otherwise {
        when(io.control.allocate.bits.baseAddr < aSpanBase) {
          aSpanBase := io.control.allocate.bits.baseAddr
        }
        when(descEnd > aSpanEnd) {
          aSpanEnd := descEnd
        }
      }
    }
    when(io.control.allocate.bits.stream === MatrixPrefetchStream.b) {
      bBaseHistory(bBaseHistoryWrite) := io.control.allocate.bits.baseAddr
      bBaseHistoryValid(bBaseHistoryWrite) := true.B
      bBaseHistoryWrite := bBaseHistoryWrite + 1.U
      when(!lowestBBaseValid || io.control.allocate.bits.baseAddr < lowestBBase) {
        lowestBBaseValid := true.B
        lowestBBase := io.control.allocate.bits.baseAddr
      }
    }
  }

  when(io.control.activate.valid) {
    val activateMatch = VecInit((0 until entries).map { i =>
      entryValid(i) && tagMatches(io.control.activate.bits, descs(i))
    })
    // TaskController can enqueue and issue the first matrix task in the same
    // cycle.  The descriptor is written to this table at that edge, so the
    // old entryValid/descs state alone cannot match the accompanying activate
    // pulse.  Include the chosen same-cycle allocation in the match; without
    // it the descriptor remains inactive forever and no guided request can
    // become eligible.
    val activateNewEntry = VecInit((0 until entries).map { i =>
      io.control.allocate.valid && allocOH(i) &&
        io.control.allocate.bits.taskId === MatrixPrefetchTagCodec.taskId(io.control.activate.bits) &&
        io.control.allocate.bits.stream === MatrixPrefetchTagCodec.stream(io.control.activate.bits) &&
        MatrixPrefetchTagCodec.valid(io.control.activate.bits)
    })
    assert(activateMatch.asUInt.orR || activateNewEntry.asUInt.orR,
      "Matrix prefetch activation without descriptor")
    for (i <- 0 until entries) {
      when(activateMatch(i) || activateNewEntry(i)) { entryActive(i) := true.B }
    }
  }

  val demandIncrement = Wire(Vec(entries, UInt(log2Ceil(banks + 1).W)))
  for (i <- 0 until entries) {
    demandIncrement(i) := PopCount(io.demand.map { demand =>
      demand.valid && entryValid(i) &&
        tagMatches(demand.bits.matrixPrefetchTag.getOrElse(0.U), descs(i))
    })
    when(demandIncrement(i) =/= 0.U && !(io.control.allocate.valid && allocOH(i))) {
      demandCount(i) := demandCount(i) + demandIncrement(i)
    }
  }

  val eligible = VecInit((0 until entries).map { i =>
    // A descriptor is available at decode/allocation time.  Consume only A
    // tasks younger than the active C store, so requests are issued inside the
    // store phase and target the following segment rather than the current A.
    cStoreActive && entryValid(i) && !retireMatch(i) &&
      descs(i).stream === MatrixPrefetchStream.a &&
      isYoungerTask(descs(i).taskId, cStoreTaskId) &&
      addressGenerators(i).io.valid
  })
  val taskDistance = VecInit((0 until entries).map(i => descs(i).taskId - cStoreTaskId))
  val selectOH = Wire(Vec(entries, Bool()))
  for (i <- 0 until entries) {
    val outranked = VecInit((0 until entries).map { j =>
      eligible(j) && (
        taskDistance(j) < taskDistance(i) ||
          (taskDistance(j) === taskDistance(i) && (j < i).B)
      )
    })
    selectOH(i) := eligible(i) && !outranked.asUInt.orR
  }
  val selectIdx = OHToUInt(selectOH)
  val selectedAddr = VecInit(addressGenerators.map(_.io.address))(selectIdx)(fullAddressBits - 1, 0)
  val descriptorEligible = eligible.asUInt.orR

  private def targetBankReady(address: UInt): Bool =
    (0 until banks).map { i =>
      bank_eq(address >> offsetBits, i, bankBits) && io.bankReady(i)
    }.reduce(_ || _)

  private def targetBankBestEffortReady(address: UInt): Bool =
    (0 until banks).map { i =>
      bank_eq(address >> offsetBits, i, bankBits) && io.bankBestEffortReady(i)
    }.reduce(_ || _)

  val cStoreLookaheadEligible = cStoreActive && cStoreLookaheadActive &&
    cStoreLookaheadGenerator.io.valid && targetBankReady(cStoreLookaheadGenerator.io.address)
  val bOnALookaheadBankReady = VecInit((0 until bOnAWalkers).map { i =>
    targetBankReady(bOnALookaheadGenerators(i).io.address)
  })
  val bOnALookaheadArb = Module(new RRArbiter(UInt(bOnAWalkerIndexBits.W), bOnAWalkers))
  for (i <- 0 until bOnAWalkers) {
    bOnALookaheadArb.io.in(i).valid := io.enableBOnA && bOnALookaheadActive(i) &&
      bOnALookaheadGenerators(i).io.valid && bOnALookaheadBankReady(i)
    bOnALookaheadArb.io.in(i).bits := i.U
  }
  val bOnALookaheadEligible = bOnALookaheadArb.io.out.valid
  val bOnASelectedWalker = bOnALookaheadArb.io.out.bits
  val bOnASelectedAddress =
    VecInit(bOnALookaheadGenerators.map(_.io.address))(bOnASelectedWalker)
  val bOnASelectedTaskId =
    VecInit(bOnACurrentDesc.map(_.taskId))(bOnASelectedWalker)
  val bOnASelectedIssued = bOnALookaheadIssued(bOnASelectedWalker)
  val cOnBRateSlot = cOnBRateCounter === 0.U
  val cOnBLookaheadEligible = cOnBLookaheadActive && !activateIsC &&
    cOnBLookaheadGenerator.io.valid && cOnBRateSlot &&
    targetBankBestEffortReady(cOnBLookaheadGenerator.io.address)
  // The older descriptor path and the synthetic lookahead describe the same
  // future A row in this C-store-gated policy.  Letting the former resume
  // after the lookahead finishes reissues that row (and then more speculative
  // A descriptors) in the same store window.  C-store mode therefore emits
  // only the single next-A lookahead stream.
  val useCStoreLookahead = cStoreLookaheadEligible
  // Preserve the existing C-store -> next-A policy as the higher-priority
  // stream when its target bank is ready. B-on-A can use a different ready
  // bank instead of waiting behind a blocked A or B address.
  // C-on-B gets at most one reserved slot in eight cycles and only when its
  // target queue is below the best-effort watermark. B uses all other slots.
  val useCOnBLookahead = !useCStoreLookahead && cOnBLookaheadEligible
  val useBOnALookahead = !useCStoreLookahead && !useCOnBLookahead && bOnALookaheadEligible
  val requestEligible = useCStoreLookahead || useCOnBLookahead || useBOnALookahead
  val requestAddr = Mux(
    useCStoreLookahead,
    cStoreLookaheadGenerator.io.address,
    Mux(useCOnBLookahead, cOnBLookaheadGenerator.io.address, bOnASelectedAddress)
  )(fullAddressBits - 1, 0)
  val (requestTag, requestSet, _) = parseFullAddress(requestAddr)

  class MatrixPrefetchTraceEntry extends Bundle {
    val cStoreTaskId = UInt(MatrixPrefetchTagCodec.taskIdWidth.W)
    val taskId = UInt(MatrixPrefetchTagCodec.taskIdWidth.W)
    val stream = UInt(MatrixPrefetchStream.width.W)
    val address = UInt(fullAddressBits.W)
    val issued = UInt(32.W)
    val demand = UInt(32.W)
  }
  val matrixPrefetchTraceTable =
    ChiselDB.createTable("L2MatrixPrefetchTrace", new MatrixPrefetchTraceEntry, basicDB = true)
  val matrixPrefetchTrace = WireInit(0.U.asTypeOf(new MatrixPrefetchTraceEntry))
  matrixPrefetchTrace.cStoreTaskId := cStoreTaskId
  matrixPrefetchTrace.taskId := Mux(
    useCStoreLookahead,
    cStoreTaskId,
    Mux(useCOnBLookahead, currentCDesc.taskId, bOnASelectedTaskId)
  )
  matrixPrefetchTrace.stream := Mux(
    useCStoreLookahead,
    MatrixPrefetchStream.a,
    Mux(useCOnBLookahead, MatrixPrefetchStream.cLoad, MatrixPrefetchStream.b)
  )
  matrixPrefetchTrace.address := requestAddr
  matrixPrefetchTrace.issued := Mux(
    useCStoreLookahead,
    cStoreLookaheadIssued,
    Mux(useCOnBLookahead, cOnBLookaheadIssued, bOnASelectedIssued)
  )
  matrixPrefetchTrace.demand := 0.U

  // The lossless per-bank Matrix queues provide backpressure.  Keep the
  // current address stable while stalled and otherwise accept one line per
  // cycle; the old fixed cooldown capped throughput below what a K=1024 row
  // needs to finish inside its four C-store windows.
  io.req.valid := io.enable && requestEligible
  io.req.bits := 0.U.asTypeOf(new PrefetchReq)
  io.req.bits.tag := requestTag
  io.req.bits.set := requestSet
  io.req.bits.vaddr.foreach(_ := 0.U)
  io.req.bits.needT := false.B
  io.req.bits.source := 0.U
  io.req.bits.pfSource := PfSource.matrixMemReqSource.id.U
  bOnALookaheadArb.io.out.ready := io.enable && useBOnALookahead && io.req.ready

  when(io.req.fire) {
    when(useCStoreLookahead) {
      cStoreLookaheadGenerator.io.step := true.B
      cStoreLookaheadIssued := cStoreLookaheadIssued + 1.U
    }.elsewhen(useCOnBLookahead) {
      cOnBLookaheadGenerator.io.step := true.B
      cOnBLookaheadIssued := cOnBLookaheadIssued + 1.U
    }.elsewhen(useBOnALookahead) {
      for (i <- 0 until bOnAWalkers) {
        when(bOnASelectedWalker === i.U) {
          bOnALookaheadGenerators(i).io.step := true.B
          bOnALookaheadIssued(i) := bOnALookaheadIssued(i) + 1.U
        }
      }
    }
  }

  matrixPrefetchTraceTable.log(matrixPrefetchTrace, io.req.fire, "PrefetchRequest", clock, reset)

  XSPerfAccumulate("matrix_prefetch_desc_allocate", io.control.allocate.valid)
  XSPerfAccumulate("matrix_prefetch_desc_activate", io.control.activate.valid)
  XSPerfAccumulate("matrix_prefetch_desc_retire", io.control.retire.valid)
  XSPerfAccumulate("matrix_prefetch_cstore_start", io.control.cStoreStart.valid)
  XSPerfAccumulate("matrix_prefetch_cstore_end", io.control.cStoreEnd.valid)
  XSPerfAccumulate("matrix_prefetch_cstore_cycle", cStoreActive)
  XSPerfAccumulate("matrix_prefetch_cstore_lookahead_candidate", cStoreLookaheadCandidate)
  XSPerfAccumulate("matrix_prefetch_cstore_lookahead_start", cStoreLookaheadStart)
  XSPerfAccumulate("matrix_prefetch_cstore_lookahead_defer", cStoreLookaheadDefer)
  XSPerfAccumulate("matrix_prefetch_cstore_lookahead_pending_cycle", cStoreLookaheadPending)
  XSPerfAccumulate("matrix_prefetch_cstore_lookahead_eligible", cStoreLookaheadEligible)
  XSPerfAccumulate("matrix_prefetch_cstore_lookahead_req", io.req.fire && useCStoreLookahead)
  XSPerfAccumulate("matrix_prefetch_b_on_a_pair", consumeBOnAPair)
  XSPerfAccumulate("matrix_prefetch_b_on_a_candidate", bOnALookaheadEnqFire)
  XSPerfAccumulate("matrix_prefetch_b_on_a_boundary_skip",
    consumeBOnAPair && io.enable && io.enableBOnA &&
      !selectedBDesc.transpose && !selectedBHasNext)
  XSPerfAccumulate("matrix_prefetch_b_on_a_duplicate_skip",
    consumeBOnAPair && io.enable && io.enableBOnA && selectedBHasNext && bOnASeen)
  XSPerfAccumulate("matrix_prefetch_b_on_a_queue_full",
    bOnALookaheadCandidate && !bOnALookaheadTargetReady)
  XSPerfAccumulate("matrix_prefetch_b_on_a_start",
    PopCount(bOnALookaheadQueues.map(_.io.deq.fire)))
  XSPerfAccumulate("matrix_prefetch_b_on_a_req", io.req.fire && useBOnALookahead)
  XSPerfAccumulate("matrix_prefetch_b_on_a_active_walkers", PopCount(bOnALookaheadActive))
  XSPerfAccumulate("matrix_prefetch_b_on_a_bank_blocked", PopCount((0 until bOnAWalkers).map { i =>
    bOnALookaheadActive(i) && bOnALookaheadGenerators(i).io.valid && !bOnALookaheadBankReady(i)
  }))
  XSPerfAccumulate("matrix_prefetch_c_on_b_candidate", cOnBLookaheadCandidate)
  XSPerfAccumulate("matrix_prefetch_c_on_b_boundary_skip", cOnBBoundarySkip)
  XSPerfAccumulate("matrix_prefetch_c_on_b_req", io.req.fire && useCOnBLookahead)
  XSPerfAccumulate("matrix_prefetch_c_on_b_complete",
    cOnBLookaheadActive && cOnBLookaheadGenerator.io.done && !activateIsC)
  XSPerfAccumulate("matrix_prefetch_c_on_b_cancel",
    activateIsC && cOnBLookaheadActive && !cOnBLookaheadGenerator.io.done)
  XSPerfAccumulate("matrix_prefetch_c_on_b_rate_limited",
    cOnBLookaheadActive && cOnBLookaheadGenerator.io.valid && !cOnBRateSlot)
  XSPerfAccumulate("matrix_prefetch_c_on_b_queue_throttled",
    cOnBLookaheadActive && cOnBLookaheadGenerator.io.valid && cOnBRateSlot &&
      !targetBankBestEffortReady(cOnBLookaheadGenerator.io.address))
  XSPerfAccumulate("matrix_prefetch_demand", PopCount(io.demand.map(_.valid)))
  XSPerfAccumulate("matrix_prefetch_req", io.req.fire)
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
  val matrixPf = if (enableMatrix) Some(Module(new MatrixGuidedPrefetcher)) else None

  matrixPf.foreach { matrix =>
    val matrixEnable = Constantin.createRecord(
      s"l2_matrix_prefetch_enable${cacheParams.hartId}",
      initValue = if (p(MatrixPrefetchDefaultEnable)) 1 else 0
    ).orR
    val matrixBOnAEnable = Constantin.createRecord(
      s"l2_matrix_b_on_a_enable${cacheParams.hartId}",
      initValue = 0
    ).orR
    val matrixCOnBEnable = Constantin.createRecord(
      s"l2_matrix_c_on_b_enable${cacheParams.hartId}",
      initValue = 0
    ).orR
    matrix.io.enable := matrixEnable
    matrix.io.enableBOnA := matrixBOnAEnable
    matrix.io.enableCOnB := matrixCOnBEnable
    matrix.io.control := io.matrixPrefetch
    for (i <- 0 until banks) {
      matrix.io.demand(i).valid := io.train(i).fire &&
        MatrixPrefetchTagCodec.valid(io.train(i).bits.matrixPrefetchTag.getOrElse(0.U))
      matrix.io.demand(i).bits := io.train(i).bits
    }
  }

  val train = Wire(DecoupledIO(new PrefetchTrain))
  val resp = Wire(DecoupledIO(new PrefetchResp))
  fastArb(io.train, train, Some("prefetch_train"))
  fastArb(io.resp, resp, Some("prefetch_resp"))

  // The matrix-guided engine observes the per-bank train inputs directly and
  // does not need a TLB or response consumer.  When it is the only selected
  // prefetcher, terminate the shared legacy interfaces explicitly so the
  // arbiter still accepts demand training and FIRRTL has no floating sinks.
  if (!(hasBOP || hasNLPrefetcher || hasTPPrefetcher)) {
    train.ready := true.B
    resp.ready := true.B
  }
  if (!hasBOP) {
    io.tlb_req.req.valid := false.B
    io.tlb_req.req.bits := DontCare
    io.tlb_req.req_kill := false.B
    io.tlb_req.resp.ready := true.B
  }

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
  /* prefetchers -> per-source queues -> pipe -> Slices.SinkA */
  private val SRC_NUM = 6
  private val Seq(rcv_idx, nl_idx, vbop_idx, pbop_idx, tp_idx, matrix_idx) = (0 until SRC_NUM).toSeq
  val reqs = Seq(
    if (hasReceiver) Some(pfRcv.get.io.req) else None,
    if (hasNLPrefetcher) Some(nl.get.io.req) else None,
    if (hasBOP) Some(vbop.get.io.req) else None,
    if (hasBOP) Some(pbop.get.io.req) else None,
    if (hasTPPrefetcher) Some(tp.get.io.req) else None,
    matrixPf.map(_.io.req)
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
  // Matrix descriptors describe a complete future A row, so silently
  // replacing an old request creates permanent coverage holes. Keep Matrix
  // traffic in a lossless queue and let ready backpressure the address walker.
  // Legacy speculative prefetchers retain their overwrite behavior.
  val matrixQueue = matrixPf.map { _ =>
    Seq.tabulate(banks) { _ =>
      Module(new Queue(
        gen = new PrefetchReq,
        entries = matrixInflightEntries,
        pipe = false,
        flow = true
      ))
    }
  }
  val pipe = Seq.tabulate(banks) { _ => Module(new Pipeline(new PrefetchReq, 1)) }
  val select = Wire(Vec(banks, Vec(SRC_NUM, Bool())))
  val legacySelect = Wire(Vec(banks, Vec(SRC_NUM, Bool())))
  val selectOH = Wire(Vec(banks, Vec(SRC_NUM, Bool())))

  for (i <- 0 until banks) {
    select(i) := VecInit(reqsValid.zip(reqsSetAddr).map {
      case (valid, addr) => valid && bank_eq(addr, i, bankBits)
    })
    legacySelect(i) := select(i)
    legacySelect(i)(matrix_idx) := false.B
    selectOH(i) := VecInit(FullPriorityOneHot(legacySelect(i).toSeq).asBools)
    pftQueue(i).io.enq.valid := legacySelect(i).asUInt.orR
    pftQueue(i).io.enq.bits := ParallelPriorityMux(legacySelect(i).asUInt, reqsBits)

    matrixQueue match {
      case Some(queues) =>
        queues(i).io.enq.valid := select(i)(matrix_idx)
        queues(i).io.enq.bits := reqsBits(matrix_idx)
        val queueArb = Module(new RRArbiter(new PrefetchReq, 2))
        queueArb.io.in(0) <> pftQueue(i).io.deq
        queueArb.io.in(1) <> queues(i).io.deq
        pipe(i).io.in <> queueArb.io.out
      case None =>
        pipe(i).io.in <> pftQueue(i).io.deq
    }
    io.req(i) <> pipe(i).io.out
  }

  for ((reqOpt, j) <- reqs.zipWithIndex) {
    if (j != matrix_idx) {
      reqOpt.foreach { req =>
        req.ready := (0 until banks).map(i => selectOH(i)(j)).reduce(_ || _)
      }
    }
  }
  matrixPf.foreach { matrix =>
    val queues = matrixQueue.get
    matrix.io.bankReady := VecInit(queues.map(_.io.enq.ready))
    val matrixBestEffortWatermark = math.max(matrixInflightEntries / 4, 1)
    matrix.io.bankBestEffortReady :=
      VecInit(queues.map(_.io.count < matrixBestEffortWatermark.U))
    matrix.io.req.ready := (0 until banks).map { i =>
      select(i)(matrix_idx) && queues(i).io.enq.ready
    }.reduce(_ || _)
  }

  val reqsFire = reqs.map(_.map(_.fire).getOrElse(false.B))

  XSPerfAccumulate("prefetch_train_valid", train.valid)
  XSPerfAccumulate("prefetch_train_in_valid", PopCount(io.train.map(_.valid)))
  XSPerfAccumulate("prefetch_resp_valid", resp.valid)
  XSPerfAccumulate("prefetch_resp_in_valid", PopCount(io.resp.map(_.valid)))
  matrixPf.foreach { matrix =>
    XSPerfAccumulate("matrix_prefetch_eligible", matrix.io.req.valid)
    XSPerfAccumulate("matrix_prefetch_ready", matrix.io.req.ready)
    XSPerfAccumulate("matrix_prefetch_fire", matrix.io.req.fire)
  }
  matrixQueue.foreach { queues =>
    XSPerfAccumulate("matrix_prefetch_queue_enq", PopCount(queues.map(_.io.enq.fire)))
    XSPerfAccumulate("matrix_prefetch_queue_deq", PopCount(queues.map(_.io.deq.fire)))
    XSPerfAccumulate("matrix_prefetch_queue_full", PopCount(queues.map(_.io.count === matrixInflightEntries.U)))
    for ((queue, i) <- queues.zipWithIndex) {
      XSPerfAccumulate(s"matrix_prefetch_queue_enq_bank$i", queue.io.enq.fire)
      XSPerfAccumulate(s"matrix_prefetch_queue_deq_bank$i", queue.io.deq.fire)
      XSPerfAccumulate(s"matrix_prefetch_queue_full_bank$i", queue.io.count === matrixInflightEntries.U)
    }
  }
  XSPerfAccumulate("prefetch_req_fromL1", reqsValid(rcv_idx))
  XSPerfAccumulate("prefetch_req_fromVBOP", reqsValid(vbop_idx))
  XSPerfAccumulate("prefetch_req_fromPBOP", reqsValid(pbop_idx))
  XSPerfAccumulate("prefetch_req_fromBOP", reqsValid(vbop_idx) || reqsValid(pbop_idx))
  XSPerfAccumulate("prefetch_req_fromTP", reqsValid(tp_idx))
  XSPerfAccumulate("prefetch_req_fromNL", reqsValid(nl_idx))
  XSPerfAccumulate("prefetch_req_fromMatrix", reqsValid(matrix_idx))

  XSPerfAccumulate("prefetch_req_selectL1", reqsFire(rcv_idx))
  XSPerfAccumulate("prefetch_req_selectVBOP", reqsFire(vbop_idx))
  XSPerfAccumulate("prefetch_req_selectPBOP", reqsFire(pbop_idx))
  XSPerfAccumulate("prefetch_req_selectBOP", reqsFire(vbop_idx) || reqsFire(pbop_idx))
  XSPerfAccumulate("prefetch_req_selectTP", reqsFire(tp_idx))
  XSPerfAccumulate("prefetch_req_selectNL", reqsFire(nl_idx))
  XSPerfAccumulate("prefetch_req_selectMatrix", reqsFire(matrix_idx))
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
    val bank = UInt(log2Ceil(math.max(banks, 2)).W)
  }
  val pfTT = ChiselDB.createTable("L2PrefetchReqTable", new PrefetchEntry, basicDB = true)
  for (i <- 0 until banks) {
    val e2 = Wire(new PrefetchEntry)
    e2.paddr := io.req(i).bits.addr
    e2.needT := io.req(i).bits.needT
    e2.pfsource := io.req(i).bits.pfSource
    e2.bank := i.U
    pfTT.log(
      data = e2,
      en = io.req(i).fire,
      site = "L2PrefetchReq",
      clock, reset
    )
  }
}
