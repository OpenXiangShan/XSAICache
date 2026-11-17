package oceanus.l2

import chisel3._
import chisel3.util._
import utility._
import oceanus.chi.bundle._
import oceanus.compactchi._
import oceanus.compactchi.CCHIOpcode._
import oceanus.l2._
import oceanus.l2.L2Common._
import oceanus.l2.L2Directory._
import oceanus.l2.L2DataStorage._
import oceanus.l2.tshr._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.util.RotateVector.left
import freechips.rocketchip.util.SeqToAugmentedSeq
import oceanus.l2.tshr.L2TSHRDirectoryProxy
import xscache.oceanus.compactchi.HasCCHIParameters


/* Per-instance identities routed as ports instead of constructor constants,
   so that all TSHR/slice instances elaborate to identical modules and
   firtool can deduplicate them into a single definition. */
class L2SliceConsts(val sliceNum: Int)(implicit val p: Parameters) extends Bundle with HasL2Params {
  val sliceIdx = UInt(log2Ceil(math.max(sliceNum, 2)).W)
  val sliceNID = UInt(8.W) // CCHI flit SrcID width, see oceanus.compactchi Flit bundles
  val nodeId = UInt(paramCHI.nodeIdWidth.W)
  val clientNID = UInt(8.W) // NID of the (single) coherent upstream client, see L2ClientTable
}

class L2TSHRConsts(sliceNum: Int)(implicit p: Parameters) extends L2SliceConsts(sliceNum) {
  val tshrId = UInt(mshrIndexWidth.W)
}

class L2TSHR(val sliceNum: Int, val tshrId: Int)(implicit val p: Parameters) extends Module 
    with HasL2Params
    with HasCCHIParameters {

  val io = IO(new Bundle {

    val consts = Input(new L2TSHRConsts(sliceNum))

    val toDir = Output(new L2Directory.PathToDirectory)
    val fromDir = Input(new L2Directory.PathFromDirectory)

    val toDS = Output(new L2DataStorage.PathTSHRToDataStorage)
    val fromDS = Input(new L2DataStorage.PathDataStorageToTSHR)

    val toAlloc = Output(new L2TSHRAlloc.PathFromTSHR)
    val fromAlloc = Input(new L2TSHRAlloc.PathToTSHR)

    val UpRXEVT = Input(new FlitEVT)                    // L1 EVT
    val DnRXSNP = Input(new CHIBundleSNP)               // HN SNP
    val UpRXEVB = Input(new L2VPipeREQ.FlitEVB)         // L2 EVB
    val UpRXREQ = Input(new FlitREQ)                    // L1/L2 REQ

    val DnTXREQ = Decoupled(new CHIBundleREQ)           // HN REQ

    val UpTXSNP = Decoupled(new FlitSNP)                // SNP to L1

    val UpTXEVB = Decoupled(new L2VPipeREQ.FlitEVB)   // EVB from L2 to L2

    val UpRXRSP = Flipped(Valid(new FlitUpRSP))       // RSP from L1
    val UpRXDAT = Flipped(Valid(new FlitUpDAT))       // DAT from L1
    val UpTXRSP = Decoupled(new FlitDnRSP)            // RSP to L1
    val UpTXDAT = Decoupled(new FlitDnDAT)            // DAT to L1

    val DnRXRSP = Flipped(Valid(new CHIBundleRSP))    // RSP from HN
    val DnRXDAT = Flipped(Valid(new CHIBundleDAT))    // DAT from HN
    val DnTXRSP = Decoupled(new CHIBundleRSP)         // RSP to HN
    val DnTXDAT = Decoupled(new CHIBundleDAT)         // DAT to HN

    val toPCreditPool = Valid(new L2PCreditPool.Entry)
    val fromPCreditPool = Input(Bool())

    val toClientTableREQ = Output(UInt(paramCCHI.UpstreamNodeID_Width.W))
    val fromClientTableREQ = Input(Vec(1, Bool())) // TODO: parameterize with coherent l2 client count

    val toClientTableEVT = Output(UInt(paramCCHI.UpstreamNodeID_Width.W))
    val fromClientTableEVT = Input(Vec(1, Bool())) // TODO: parameterize with coherent l2 client count

    val peer_unlock_dir = Output(Vec(paramL2.mshrSize, Bool()))
    val peer_unlock_ds = Output(Vec(paramL2.mshrSize, Bool()))

    val self_unlock_dir = Input(Bool())
    val self_unlock_dir_tshrId = Input(UInt(mshrIndexWidth.W))
    val self_unlock_ds = Input(Bool())

    val peer_unlock_ack = Output(Bool())
    val peer_unlock_ack_tshrId = Output(UInt(mshrIndexWidth.W))
    val self_unlock_dir_ack = Input(Bool())

    val valid = Output(Bool())
  })

  // TSHR valid
  val tshr_alloc = io.fromAlloc.alloc.asUInt.orR
  val tshr_reuse = io.fromAlloc.reuse.asUInt.orR
  val tshr_enter = tshr_alloc || tshr_reuse

  val tshr_enter_EVT = io.fromAlloc.alloc.EVT || io.fromAlloc.reuse.EVT
  val tshr_enter_SNP = io.fromAlloc.alloc.SNP || io.fromAlloc.reuse.SNP
  val tshr_enter_EVB = io.fromAlloc.alloc.EVB || io.fromAlloc.reuse.EVB
  val tshr_enter_REQ = io.fromAlloc.alloc.REQ || io.fromAlloc.reuse.REQ

  val tshr_enter_EVT_WayValid_Evict = tshr_enter_EVT && io.UpRXEVT.WayValid && io.UpRXEVT.Opcode === Evict.U
  val tshr_enter_EVT_WayValid_WriteBackFull = tshr_enter_EVT && io.UpRXEVT.WayValid && io.UpRXEVT.Opcode === WriteBackFull.U

  val tshr_enter_dirRead = tshr_enter &&
                           !tshr_enter_EVT_WayValid_Evict &&
                           !tshr_enter_EVT_WayValid_WriteBackFull

  val rrip_acquire = tshr_enter_REQ && (io.UpRXREQ.Opcode === ReadShared.U ||
                                        io.UpRXREQ.Opcode === ReadUnique.U ||
                                        io.UpRXREQ.Opcode === MakeUnique.U)

  val rrip_hint = tshr_enter_REQ && (io.UpRXREQ.Opcode === StashShared.U ||
                                     io.UpRXREQ.Opcode === StashUnique.U)

  val rrip_update_next = paramL2.replacement match {
    case "plru" => true.B
    case "brrip" => rrip_acquire
    case _ => rrip_acquire || rrip_hint
  }
  val rrip_update = RegInit(false.B)
  val rrip_prefetch = RegInit(false.B)
  val rrip_refill_prefetch = RegInit(false.B)
  when (tshr_enter) {
    rrip_update := rrip_update_next
    rrip_prefetch := rrip_hint
  }
  when (tshr_alloc) {
    rrip_refill_prefetch := rrip_hint
  }

  // TSHR payloads
  val tshr_paddr = Reg(UInt(paramL2.physicalAddrWidth.W))

  when (tshr_alloc) {
    tshr_paddr := io.fromAlloc.paddr
  }

  io.toAlloc.paddr := tshr_paddr

  val tshr_inactive_rbe = Wire(Bool())
  val tshr_inactive_vpipe = Wire(Bool())
  val tshr_inactive = tshr_inactive_rbe && tshr_inactive_vpipe

  // combinational one-cycle-early prediction of the TSHR going inactive, 
  // resolving one cycle ahead of tshr_inactive.
  val tshr_inactivate = Wire(Bool())

  val tshr_wb_done_dir = Wire(Bool())
  val tshr_wb_done_ds = Wire(Bool())

  val tshr_dealloc = tshr_inactive && tshr_wb_done_dir && tshr_wb_done_ds && !tshr_enter

  val tshr_valid = RegInit(false.B)

  when (tshr_alloc) {
    tshr_valid := true.B
  }

  when (tshr_dealloc) {
    tshr_valid := false.B
  }

  io.valid := tshr_valid


  // meta
  val dirResult = Reg(new L2Directory.MetaReadResult)
  val replResult = Reg(new L2Directory.ReplReadResult)

  /* NOTICE: For current design, any partial write to meta would never assert 'meta_valid'.
             Any later read request on full meta line would result in a Directory Read if no any read done yet.
             Because it was extremely rare that partial meta write could be merged into a full meta line.
             And the later result from Directory Read would not override modified fields except 'way' and 'hit' fields,
             which were only possible to be accurate after a Directory Read. */
  val meta = dirResult
  val meta_valid = Wire(Bool())

  val meta_drop = Wire(Bool()) // modified meta drop by REQ L2 eviction

  val meta_modify = WireInit(false.B)
  val meta_modified = RegInit(L2Directory.MetaWriteMask.empty)
  val tag_modify = WireInit(false.B)
  val tag_modified = RegInit(false.B)

  val meta_write_SA_meta = Wire(new L2Directory.Meta)
  val meta_write_SA_mask = Wire(new L2Directory.MetaWriteMask)

  val meta_write_EVT_meta = Wire(new L2Directory.Meta)
  val meta_write_EVT_mask = Wire(new L2Directory.MetaWriteMask)

  val meta_write_SNP_meta = Wire(new L2Directory.Meta)
  val meta_write_SNP_mask = Wire(new L2Directory.MetaWriteMask)

  val meta_write_REQ_meta = Wire(new L2Directory.Meta)
  val meta_write_REQ_mask = Wire(new L2Directory.MetaWriteMask)
  val tag_write_REQ_mask = Wire(Bool())

  val meta_commit_valid = Wire(Bool())

  when (meta_commit_valid) {
    meta_modified := L2Directory.MetaWriteMask.empty
    tag_modified := false.B
  }

  when (io.fromDir.DirRdResp && io.fromDir.TSHRID === io.consts.tshrId) {
    meta_modified.unmaskAndWrite(meta, io.fromDir.META)
    meta.way := io.fromDir.META.way
    meta.hit := io.fromDir.META.hit
  }
  
  when (io.fromDir.ReplRdResp && io.fromDir.TSHRID === io.consts.tshrId) {
    replResult := io.fromDir.REPL
    meta.way := io.fromDir.REPL.way
  }

  meta_write_SA_mask.maskAndWrite(meta, meta_modified, meta_write_SA_meta)
  meta_write_EVT_mask.maskAndWrite(meta, meta_modified, meta_write_EVT_meta)
  meta_write_SNP_mask.maskAndWrite(meta, meta_modified, meta_write_SNP_meta)
  meta_write_REQ_mask.maskAndWrite(meta, meta_modified, meta_write_REQ_meta)

  meta_modify := meta_write_SA_mask.asUInt.orR ||
                 meta_write_EVT_mask.asUInt.orR || 
                 meta_write_SNP_mask.asUInt.orR || 
                 meta_write_REQ_mask.asUInt.orR

  // Keep meta.hit in sync with allocating pipe writes: once any vPipe writes a
  // non-Invalid state, the directory tracks this line (committed via DirWb), so
  // meta.hit must no longer report the stale miss of the original DirRdResp.
  val meta_write_allocates =
    (meta_write_SA_mask.state  && meta_write_SA_meta.state  =/= L2Directory.MetaState.I) ||
    (meta_write_REQ_mask.state && meta_write_REQ_meta.state =/= L2Directory.MetaState.I) ||
    (meta_write_SNP_mask.state && meta_write_SNP_meta.state =/= L2Directory.MetaState.I) ||
    (meta_write_EVT_mask.state && meta_write_EVT_meta.state =/= L2Directory.MetaState.I)

  val meta_write_invalidates = 
    (meta_write_SA_mask.state  && meta_write_SA_meta.state  === L2Directory.MetaState.I) ||
    (meta_write_REQ_mask.state && meta_write_REQ_meta.state === L2Directory.MetaState.I) ||
    (meta_write_SNP_mask.state && meta_write_SNP_meta.state === L2Directory.MetaState.I) ||
    (meta_write_EVT_mask.state && meta_write_EVT_meta.state === L2Directory.MetaState.I)

  when (meta_write_allocates) {
    meta.hit := true.B
  }

  when (meta_write_invalidates) {
    meta.hit := false.B
  }

  when (tag_write_REQ_mask) {
    tag_modify := true.B
    tag_modified := true.B
  }

  when (meta_drop) {
    meta_modify := false.B
    meta_modified := L2Directory.MetaWriteMask.empty
    tag_modify := false.B
    tag_modified := false.B
  }

  assert(PopCount(Seq(meta_write_SA_mask, meta_write_EVT_mask, meta_write_SNP_mask, meta_write_REQ_mask).map(_.state)) <= 1.U, 
    "TSHR @ %m multiple active write on meta.state")
  assert(PopCount(Seq(meta_write_SA_mask, meta_write_EVT_mask, meta_write_SNP_mask, meta_write_REQ_mask).map(_.dirty)) <= 1.U, 
    "TSHR @ %m multiple active write on meta.dirty")

  assert(!(tshr_dealloc && meta_modified.asUInt.orR), "TSHR @ %m deallocated with un-committed modified meta")

  assert(!(meta_valid && meta.state =/= L2Directory.MetaState.I && !meta.hit),
    "TSHR @ %m tracked state with stale miss hit-flag")

  
  // TSHR Buffer
  val tshr_buffer_0 = Reg(UInt(256.W))
  val tshr_buffer_2 = Reg(UInt(256.W))

  val tshr_buffer_wen_UpRXDAT_0 = io.UpRXDAT.fire && io.UpRXDAT.bits.DataID === 0.U
  val tshr_buffer_wen_UpRXDAT_2 = io.UpRXDAT.fire && io.UpRXDAT.bits.DataID === 1.U // upstream DataID: packed beat index {0,1}

  val tshr_buffer_wen_DnRXDAT_0 = io.DnRXDAT.fire && io.DnRXDAT.bits.DataID.get === 0.U
  val tshr_buffer_wen_DnRXDAT_2 = io.DnRXDAT.fire && io.DnRXDAT.bits.DataID.get === 2.U

  val tshr_buffer_wen_RXDAT_0 = tshr_buffer_wen_UpRXDAT_0 || tshr_buffer_wen_DnRXDAT_0
  val tshr_buffer_wen_RXDAT_2 = tshr_buffer_wen_UpRXDAT_2 || tshr_buffer_wen_DnRXDAT_2

  val tshr_buffer_commit = WireInit(false.B)

  val tshr_buffer_drop = WireInit(false.B)

  val tshr_buffer_halfWritten_0_q = RegInit(false.B)
  val tshr_buffer_halfWritten_2_q = RegInit(false.B)

  val tshr_buffer_fullModified_q = RegInit(false.B)
  val tshr_buffer_halfModified = tshr_buffer_halfWritten_0_q || tshr_buffer_halfWritten_2_q
  val tshr_buffer_modified = tshr_buffer_fullModified_q || tshr_buffer_halfModified

  val tshr_buffer_rxdat_after_ds_arb_q = RegInit(false.B)
  when (io.fromDS.DSBufRdArbComp) {
    tshr_buffer_rxdat_after_ds_arb_q := false.B
  }
  when (tshr_buffer_wen_RXDAT_0 || tshr_buffer_wen_RXDAT_2) {
    tshr_buffer_rxdat_after_ds_arb_q := true.B
  }
  when (io.fromDS.DSBufRdResp || tshr_dealloc) {
    tshr_buffer_rxdat_after_ds_arb_q := false.B
  }

  val tshr_buffer_wen_DS = io.fromDS.DSBufRdResp && !tshr_buffer_modified && !tshr_buffer_rxdat_after_ds_arb_q

  val tshr_buffer_wen_last = WireInit(false.B)

  when (tshr_buffer_wen_DS) {
    tshr_buffer_0 := io.fromDS.DATA(255, 0)
    tshr_buffer_2 := io.fromDS.DATA(511, 256)
  }

  assert(!(tshr_dealloc && tshr_buffer_modified), "TSHR @ %m deallocated with un-committed modified data")

  when (tshr_buffer_wen_UpRXDAT_0) { tshr_buffer_0 := io.UpRXDAT.bits.Data }
  when (tshr_buffer_wen_UpRXDAT_2) { tshr_buffer_2 := io.UpRXDAT.bits.Data }

  when (tshr_buffer_wen_DnRXDAT_0) { tshr_buffer_0 := io.DnRXDAT.bits.Data.get }
  when (tshr_buffer_wen_DnRXDAT_2) { tshr_buffer_2 := io.DnRXDAT.bits.Data.get }

  /*
  tshr_buffer_halfWritten_x_q: Indicating that the TSHR Buffer was partially/halfy written.

  tshr_buffer_fullModified_q: Indicating that there were dirty data in the TSHR Buffer, but not asserted 
                              on TSHR Buffer was partially/halfly written, preventing writing to Data Storage
                              too early.

  tshr_buffer_wen_last: Indicating that the whole line TSHR Buffer is getting ready, and going to be no more
                        partially/halfly written.
  */

  when (tshr_buffer_wen_RXDAT_0) {
    when (tshr_buffer_halfWritten_2_q) {
      tshr_buffer_halfWritten_2_q := false.B
      tshr_buffer_fullModified_q := true.B
      tshr_buffer_wen_last := true.B
    }.otherwise {
      tshr_buffer_halfWritten_0_q := true.B
      tshr_buffer_fullModified_q := false.B
    }
  }

  when (tshr_buffer_wen_RXDAT_2) {
    when (tshr_buffer_halfWritten_0_q) {
      tshr_buffer_halfWritten_0_q := false.B
      tshr_buffer_fullModified_q := true.B
      tshr_buffer_wen_last := true.B
    }.otherwise {
      tshr_buffer_halfWritten_2_q := true.B
      tshr_buffer_fullModified_q := false.B
    }
  }

  when (tshr_buffer_commit || tshr_buffer_drop) {
    tshr_buffer_fullModified_q := false.B
  }

  /*
  *NOTICE: Multiple write on a partial/half entry of TSHR Buffer is not supported for now, while this could be
           easy to implement with seperate counters.
           Consider this support on future changes or assertions.
  */
  assert(!(tshr_buffer_halfWritten_0_q && tshr_buffer_wen_RXDAT_0), "double write on buffer DataID 0 from RXDAT")
  assert(!(tshr_buffer_halfWritten_2_q && tshr_buffer_wen_RXDAT_2), "double write on buffer DataID 2 from RXDAT")

  /*
  *NOTICE: Multiple source concurrent write on TSHR Buffer is not supported for now. 
  */
  private val tshr_buffer_halfWritten_UpRXDAT_0_q = RegInit(false.B) // Debug only for now
  private val tshr_buffer_halfWritten_UpRXDAT_2_q = RegInit(false.B) // Debug only for now
  private val tshr_buffer_halfWritten_DnRXDAT_0_q = RegInit(false.B) // Debug only for now
  private val tshr_buffer_halfWritten_DnRXDAT_2_q = RegInit(false.B) // Debug only for now

  when (tshr_buffer_wen_UpRXDAT_0) {
    when (tshr_buffer_halfWritten_UpRXDAT_2_q) {
      tshr_buffer_halfWritten_UpRXDAT_2_q := false.B
    }.otherwise {
      tshr_buffer_halfWritten_UpRXDAT_0_q := true.B
    }
  }
  when (tshr_buffer_wen_UpRXDAT_2) {
    when (tshr_buffer_halfWritten_UpRXDAT_0_q) {
      tshr_buffer_halfWritten_UpRXDAT_0_q := false.B
    }.otherwise {
      tshr_buffer_halfWritten_UpRXDAT_2_q := true.B
    }
  }
  when (tshr_buffer_wen_DnRXDAT_0) {
    when (tshr_buffer_halfWritten_DnRXDAT_2_q) {
      tshr_buffer_halfWritten_DnRXDAT_2_q := false.B
    }.otherwise {
      tshr_buffer_halfWritten_DnRXDAT_0_q := true.B
    }
  }
  when (tshr_buffer_wen_DnRXDAT_2) {
    when (tshr_buffer_halfWritten_DnRXDAT_0_q) {
      tshr_buffer_halfWritten_DnRXDAT_0_q := false.B
    }.otherwise {
      tshr_buffer_halfWritten_DnRXDAT_2_q := true.B
    }
  }

  assert(!(tshr_buffer_wen_UpRXDAT_0 && tshr_buffer_halfWritten_UpRXDAT_0_q), "double write on buffer from UpRXDAT (0, 0)")
  assert(!(tshr_buffer_wen_UpRXDAT_0 && tshr_buffer_halfWritten_DnRXDAT_0_q), "multiple source write on buffer from DnRXDAT then UpRXDAT (0, 0)")
  assert(!(tshr_buffer_wen_UpRXDAT_0 && tshr_buffer_halfWritten_DnRXDAT_2_q), "multiple source write on buffer from DnRXDAT then UpRXDAT (2, 0)")
  assert(!(tshr_buffer_wen_UpRXDAT_2 && tshr_buffer_halfWritten_UpRXDAT_2_q), "double write on buffer from UpRXDAT (2, 2)")
  assert(!(tshr_buffer_wen_UpRXDAT_2 && tshr_buffer_halfWritten_DnRXDAT_0_q), "multiple source write on buffer from DnRXDAT then UpRXDAT (0, 2)")
  assert(!(tshr_buffer_wen_UpRXDAT_2 && tshr_buffer_halfWritten_DnRXDAT_2_q), "multiple source write on buffer from DnRXDAT then UpRXDAT (2, 2)")
  assert(!(tshr_buffer_wen_DnRXDAT_0 && tshr_buffer_halfWritten_DnRXDAT_0_q), "double write on buffer from DnRXDAT (0, 0)")
  assert(!(tshr_buffer_wen_DnRXDAT_0 && tshr_buffer_halfWritten_UpRXDAT_0_q), "multiple source write on buffer from UpRXDAT then DnRXDAT (0, 0)")
  assert(!(tshr_buffer_wen_DnRXDAT_0 && tshr_buffer_halfWritten_UpRXDAT_2_q), "multiple source write on buffer from UpRXDAT then DnRXDAT (2, 0)")
  assert(!(tshr_buffer_wen_DnRXDAT_2 && tshr_buffer_halfWritten_DnRXDAT_2_q), "double write on buffer from DnRXDAT (2, 2)")
  assert(!(tshr_buffer_wen_DnRXDAT_2 && tshr_buffer_halfWritten_UpRXDAT_0_q), "multiple source write on buffer from UpRXDAT then DnRXDAT (0, 2)")
  assert(!(tshr_buffer_wen_DnRXDAT_2 && tshr_buffer_halfWritten_UpRXDAT_2_q), "multiple source write on buffer from UpRXDAT then DnRXDAT (2, 2)")
  

  // RBEs
  val rbeEVT = Module(new L2RBE(new FlitEVT /*TODO: strip PA here*/))
  val rbeSNP = Module(new L2RBE(new CHIBundleSNP /*TODO: strip PA here*/))
  val rbeEVB = Module(new L2RBE(new L2VPipeREQ.FlitEVB))
  val rbeREQ = Module(new L2RBE(new FlitREQStripped))

  io.toAlloc.busy.EVT := !rbeEVT.io.in.ready
  io.toAlloc.busy.SNP := !rbeSNP.io.in.ready
  io.toAlloc.busy.EVB := !rbeEVB.io.in.ready
  io.toAlloc.busy.REQ := !rbeREQ.io.in.ready

  // RBE drain priority chain: EVT > SNP > EVB > REQ.
  // NOTE: keyed on io.valid (queue occupancy, registered) rather than io.out.valid
  // (drain fire) — the latter closes a combinational cycle through vPipeSNP.blockRBE.EVT
  rbeSNP.io.blockFromTSHR := rbeEVT.io.valid
  rbeEVB.io.blockFromTSHR := rbeEVT.io.valid || rbeSNP.io.valid
  rbeREQ.io.blockFromTSHR := rbeEVT.io.valid || rbeSNP.io.valid || rbeEVB.io.valid

  // PathVPipeBlock.EVB is only meaningful on the vPipe (blockRBE) side;
  // there is no EVB vPipe to block an RBE, so tie it off here
  Seq(rbeEVT, rbeSNP, rbeEVB, rbeREQ).foreach(_.io.blockFromVPipe.EVB := false.B)

  rbeEVT.io.in.bits := io.UpRXEVT
  rbeSNP.io.in.bits := io.DnRXSNP
  rbeEVB.io.in.bits := io.UpRXEVB
  rbeREQ.io.in.bits := io.UpRXREQ

  rbeEVT.io.in.valid := tshr_enter_EVT
  rbeSNP.io.in.valid := tshr_enter_SNP
  rbeEVB.io.in.valid := tshr_enter_EVB
  rbeREQ.io.in.valid := tshr_enter_REQ

  rbeEVT.io.directoryReadNeed := !((rbeEVT.io.out.bits.Opcode === Evict.U || rbeEVT.io.out.bits.Opcode === WriteBackFull.U) && rbeEVT.io.out.bits.WayValid)
  rbeSNP.io.directoryReadNeed := true.B
  rbeEVB.io.directoryReadNeed := true.B
  rbeREQ.io.directoryReadNeed := true.B

  tshr_inactive_rbe := !rbeEVT.io.valid && !rbeSNP.io.valid && !rbeEVB.io.valid && !rbeREQ.io.valid &&
                       !rbeEVT.io.in.valid && !rbeSNP.io.in.valid && !rbeEVB.io.in.valid && !rbeREQ.io.in.valid

  val dirResult_with_fastpath = if (paramL2.hitFastPathDirRespFlow) {
    val dirRdResp_for_me = io.fromDir.DirRdResp && io.fromDir.TSHRID === io.consts.tshrId
    val dirReq_fire_fastpath = dirRdResp_for_me && rbeREQ.io.out.fire
    Mux(dirReq_fire_fastpath, io.fromDir.META, dirResult)
  } else {
    dirResult
  }

  // Post RBE Data Storage Read Decision
  val ds_read_rbeEVT_en = Wire(Bool())
  val ds_read_rbeSNP_en = Wire(Bool())
  val ds_read_rbeREQ_en = Wire(Bool())

  ds_read_rbeEVT_en := false.B
  ds_read_rbeSNP_en := false.B
  ds_read_rbeREQ_en := false.B
  
  // TODO


  // -- vPipes and TSHR local modules
  val vPipeEVT = Module(new L2VPipeEVT(Seq(/*TODO: client devices*/), sliceNum))
  val vPipeSNP = Module(new L2VPipeSNP(Seq(/*TODO: client devices*/), sliceNum))
  val vPipeREQ = Module(new L2VPipeREQ(Seq(/*TODO: client devices*/), sliceNum))
  val snoopAgent = Module(new L2SnoopAgent(sliceNum))

  vPipeEVT.io.consts := io.consts
  vPipeSNP.io.consts := io.consts
  vPipeREQ.io.consts := io.consts
  snoopAgent.io.consts := io.consts

  tshr_inactive_vpipe := vPipeEVT.io.free && vPipeSNP.io.free && vPipeREQ.io.free

  // TSHR inactivation exact prediction (hitFastPathEarlyInactive)
  // The proxies use this to resolve their write FSMs one cycle early;
  // tshr_dealloc still keys on the real tshr_inactive.
  tshr_inactivate := (if (paramL2.hitFastPathEarlyInactive)
                        vPipeREQ.io.free_next && vPipeEVT.io.free && vPipeSNP.io.free && tshr_inactive_rbe
                      else false.B)

  // connections between SNP vPipe and Snoop Agent
  snoopAgent.io.uopFromSNP.valid := vPipeSNP.io.toSA.SnpMakeInvalid ||
                                    vPipeSNP.io.toSA.SnpToInvalid ||
                                    vPipeSNP.io.toSA.SnpToShared ||
                                    vPipeSNP.io.toSA.SnpToClean
  snoopAgent.io.uopFromSNP.bits := vPipeSNP.io.toSA
  vPipeSNP.io.fromSA := snoopAgent.io.fromSAForSNP

  // connections between REQ vPipe and Snoop Agent
  snoopAgent.io.uopFromREQ.valid := vPipeREQ.io.toSA.SnpMakeInvalid ||
                                    vPipeREQ.io.toSA.SnpToInvalid ||
                                    vPipeREQ.io.toSA.SnpToShared ||
                                    vPipeREQ.io.toSA.SnpToClean
  snoopAgent.io.uopFromREQ.bits := vPipeREQ.io.toSA
  vPipeREQ.io.fromSA := snoopAgent.io.fromSAForREQ

  // connections between TSHR local / RX and Snoop Agent
  snoopAgent.io.tshr_paddr := tshr_paddr
  snoopAgent.io.tshr_dirResult := dirResult_with_fastpath
  snoopAgent.io.UpRXRSP := io.UpRXRSP
  snoopAgent.io.UpRXDAT := io.UpRXDAT

  // connections between RBEs / RX and EVT vPipe
  rbeEVT.io.blockFromVPipe.EVT := vPipeEVT.io.blockRBE.EVT
  rbeSNP.io.blockFromVPipe.EVT := vPipeEVT.io.blockRBE.SNP
  rbeEVB.io.blockFromVPipe.EVT := vPipeEVT.io.blockRBE.EVB
  rbeREQ.io.blockFromVPipe.EVT := vPipeEVT.io.blockRBE.REQ

  vPipeEVT.io.UpRXEVT := rbeEVT.io.out

  vPipeEVT.io.UpRXDAT := io.UpRXDAT

  // connections between RBEs / RX and SNP vPipe
  rbeEVT.io.blockFromVPipe.SNP := vPipeSNP.io.blockRBE.EVT
  rbeSNP.io.blockFromVPipe.SNP := vPipeSNP.io.blockRBE.SNP
  rbeEVB.io.blockFromVPipe.SNP := vPipeSNP.io.blockRBE.EVB
  rbeREQ.io.blockFromVPipe.SNP := vPipeSNP.io.blockRBE.REQ

  vPipeSNP.io.DnRXSNP := rbeSNP.io.out

  // connections between RBEs / RX and REQ vPipe
  rbeEVT.io.blockFromVPipe.REQ := vPipeREQ.io.blockRBE.EVT
  rbeSNP.io.blockFromVPipe.REQ := vPipeREQ.io.blockRBE.SNP
  rbeEVB.io.blockFromVPipe.REQ := vPipeREQ.io.blockRBE.EVB
  rbeREQ.io.blockFromVPipe.REQ := vPipeREQ.io.blockRBE.REQ

  vPipeREQ.io.UpRXREQ := rbeREQ.io.out
  vPipeREQ.io.UpRXEVB := rbeEVB.io.out

  vPipeREQ.io.DnRXRSP := io.DnRXRSP
  vPipeREQ.io.DnRXDAT := io.DnRXDAT

  vPipeREQ.io.UpRXRSP := io.UpRXRSP
  vPipeREQ.io.UpRXDAT := io.UpRXDAT

  // connections between TSHR local and Snoop Agent
  meta_write_SA_mask := snoopAgent.io.tshr_meta_write_en
  meta_write_SA_meta := snoopAgent.io.tshr_meta_write_meta

  // connections between TSHR local and EVT vPipe
  vPipeEVT.io.tshr_paddr := tshr_paddr
  vPipeEVT.io.tshr_dirResult := dirResult
  vPipeEVT.io.tshr_meta := meta
  vPipeEVT.io.peer_meta_write_state := vPipeREQ.io.tshr_meta_write_en.state ||
                                       vPipeSNP.io.tshr_meta_write_en.state

  meta_write_EVT_mask := vPipeEVT.io.tshr_meta_write_en
  meta_write_EVT_meta := vPipeEVT.io.tshr_meta_write_meta

  io.toClientTableEVT := 0.U // connect this when supports multiple coherent upstreams

  // connections between TSHR local and SNP vPipe
  vPipeSNP.io.tshr_paddr := tshr_paddr
  vPipeSNP.io.tshr_dirResult := dirResult
  vPipeSNP.io.tbuf_modified := tshr_buffer_modified
  vPipeSNP.io.tbuf_half0_ready := tshr_buffer_halfWritten_0_q || tshr_buffer_fullModified_q || vPipeSNP.io.ds_read_done
  vPipeSNP.io.tbuf_half2_ready := tshr_buffer_halfWritten_2_q || tshr_buffer_fullModified_q || vPipeSNP.io.ds_read_done

  meta_write_SNP_mask := vPipeSNP.io.tshr_meta_write_en
  meta_write_SNP_meta := vPipeSNP.io.tshr_meta_write_meta

  vPipeSNP.io.EVT_active := vPipeEVT.io.EVT_active
  vPipeSNP.io.REQ_evict := vPipeREQ.io.L2EVT_opcode

  // connections between TSHR local and REQ vPipe
  vPipeREQ.io.tshr_paddr := tshr_paddr
  vPipeREQ.io.tshr_dirResult := dirResult_with_fastpath
  vPipeREQ.io.tshr_dealloc := tshr_dealloc
  vPipeREQ.io.tshr_meta_modified := meta_modified.asUInt.orR
  vPipeREQ.io.tshr_tag_modified := tag_modified
  vPipeREQ.io.tbuf_modified := tshr_buffer_modified
  vPipeREQ.io.tbuf_data0_valid := tshr_buffer_halfWritten_0_q || tshr_buffer_fullModified_q || vPipeREQ.io.ds_rd_done
  vPipeREQ.io.tbuf_data2_valid := tshr_buffer_halfWritten_2_q || tshr_buffer_fullModified_q || vPipeREQ.io.ds_rd_done

  meta_write_REQ_mask := vPipeREQ.io.tshr_meta_write_en
  meta_write_REQ_meta := vPipeREQ.io.tshr_meta_write_meta
  tag_write_REQ_mask := vPipeREQ.io.tshr_tag_write_en

  meta_drop := vPipeREQ.io.dir_wb_cancel
  tshr_buffer_drop := vPipeREQ.io.ds_wb_cancel || vPipeSNP.io.ds_wb_cancel

  io.toPCreditPool := vPipeREQ.io.toPCreditPool
  vPipeREQ.io.fromPCreditPool := io.fromPCreditPool

  io.toClientTableREQ := vPipeREQ.io.toClientTable
  vPipeREQ.io.fromClientTable := io.fromClientTableREQ

  io.peer_unlock_dir := vPipeREQ.io.peer_unlock_dir
  io.peer_unlock_ds := vPipeREQ.io.peer_unlock_ds

  vPipeREQ.io.self_unlock_dir := io.self_unlock_dir
  vPipeREQ.io.self_unlock_dir_tshrId := io.self_unlock_dir_tshrId
  vPipeREQ.io.self_unlock_ds := io.self_unlock_ds

  io.peer_unlock_ack := vPipeREQ.io.peer_unlock_ack
  io.peer_unlock_ack_tshrId := vPipeREQ.io.peer_unlock_ack_tshrId
  vPipeREQ.io.self_unlock_dir_ack := io.self_unlock_dir_ack

  vPipeREQ.io.L1EVT_active := vPipeEVT.io.EVT_active
  vPipeREQ.io.L1EVT_data_pending := vPipeEVT.io.EVT_data_pending
  vPipeREQ.io.L1EVT_wbFull := vPipeEVT.io.UpRXEVT.valid &&
                              vPipeEVT.io.UpRXEVT.bits.Opcode === CCHIOpcode.WriteBackFull.U

  // connections between TX channels and TSHR local modules
  io.DnTXREQ <> vPipeREQ.io.DnTXREQ

  io.UpTXSNP <> snoopAgent.io.txSnp

  fastArb(Seq(vPipeEVT.io.UpTXRSP, vPipeREQ.io.UpTXRSP), io.UpTXRSP, Some("UpTXRSP"))

  io.UpTXDAT <> vPipeREQ.io.UpTXDAT
  io.UpTXDAT.bits.Data := Mux(io.UpTXDAT.bits.DataID === 0.U, tshr_buffer_0, tshr_buffer_2)

  fastArb(Seq(vPipeSNP.io.DnTXRSP, vPipeREQ.io.DnTXRSP), io.DnTXRSP, Some("DnTXRSP"))

  fastArb(Seq(vPipeSNP.io.DnTXDAT, vPipeREQ.io.DnTXDAT), io.DnTXDAT, Some("DnTXDAT"))
  val dataBeat = Mux(io.DnTXDAT.bits.DataID.get === 0.U, tshr_buffer_0, tshr_buffer_2)
  io.DnTXDAT.bits.Data.get := VecInit(
    (0 until paramCHI.datBEWidth).map { byteIndex =>
      Mux(
        io.DnTXDAT.bits.BE.get(byteIndex),
        dataBeat(byteIndex * 8 + 7, byteIndex * 8),
        0.U(8.W))
    }
  ).asUInt

  io.UpTXEVB <> vPipeREQ.io.UpTXEVB

  // ----------------------------------------------------------------

  // Directory Proxy
  val proxyDir = Module(new L2TSHRDirectoryProxy(tshrId))

  proxyDir.io.tshrId := io.consts.tshrId

  io.toDir := proxyDir.io.toDir
  proxyDir.io.fromDir := io.fromDir

  proxyDir.io.tshr_valid := tshr_valid
  proxyDir.io.tshr_paddr := tshr_paddr

  proxyDir.io.tshr_alloc := tshr_alloc
  proxyDir.io.tshr_reuse := tshr_reuse
  proxyDir.io.tshr_inactive := tshr_inactive
  proxyDir.io.tshr_inactivate := tshr_inactivate
  proxyDir.io.tshr_dealloc := tshr_dealloc

  proxyDir.io.read_arbed := false.B // TODO: S0 Directory Arbitration from L2TSHRCtrl
  proxyDir.io.read_en := tshr_enter_dirRead
  proxyDir.io.repl_en := vPipeREQ.io.repl_en
  proxyDir.io.repl_reset := vPipeREQ.io.repl_reset

  proxyDir.io.meta := meta
  proxyDir.io.meta_way := meta.way
  proxyDir.io.meta_modify := meta_modify
  proxyDir.io.meta_modified := meta_modified
  proxyDir.io.tag_modify := tag_modify
  proxyDir.io.tag_modified := tag_modified
  proxyDir.io.rrip_update := rrip_update
  proxyDir.io.rrip_prefetch := rrip_prefetch
  proxyDir.io.rrip_refill_prefetch := rrip_refill_prefetch

  proxyDir.io.repl_release.zip(vPipeEVT.io.tshr_repl_release).foreach { case (in, out) => in := out }
  proxyDir.io.repl_way.zip(vPipeEVT.io.tshr_repl_way).foreach { case (in, out) => in := out }

  // Local meta is the latest copy for this address. Capture release provenance
  // in the proxy, then mark it consumed locally without forcing a DirWb. The
  // validity indicator is the live proxy output, never a TSHR-side register.
  vPipeEVT.io.tshr_repl_release.foreach { release =>
    when (release && (proxyDir.io.rd_done || meta_modified.prefetch)) {
      meta.prefetch := false.B
    }
  }

  rbeEVT.io.directoryReadDone := proxyDir.io.rd_done
  rbeSNP.io.directoryReadDone := proxyDir.io.rd_done
  rbeEVB.io.directoryReadDone := proxyDir.io.rd_done
  rbeREQ.io.directoryReadDone := (if (paramL2.hitFastPathDirRespFlow) proxyDir.io.rd_done_flow
                                  else                               proxyDir.io.rd_done)

  vPipeREQ.io.repl_retry := proxyDir.io.repl_retry
  vPipeREQ.io.repl_done := proxyDir.io.repl_done
  vPipeREQ.io.repl_resp := replResult
  vPipeREQ.io.repl_meta := replResult.meta

  proxyDir.io.repl_lock_candidate := vPipeREQ.io.repl_lock_candidate
  proxyDir.io.repl_lock_req := vPipeREQ.io.repl_lock_req
  proxyDir.io.repl_lock_way := vPipeREQ.io.repl_lock_way
  vPipeREQ.io.repl_lock_conflict := proxyDir.io.repl_lock_conflict
  proxyDir.io.repl_lock_release := vPipeREQ.io.repl_lock_release

  proxyDir.io.wb_aux := vPipeREQ.io.dir_wb_aux

  meta_commit_valid := proxyDir.io.wb_accept

  meta_valid := proxyDir.io.rd_done
  tshr_wb_done_dir := proxyDir.io.wb_done && proxyDir.io.rel_idle
  rbeEVT.io.blockFromTSHR := !proxyDir.io.rel_idle

  // Data Storage Proxy
  val proxyDS = Module(new L2TSHRDataStorageProxy(tshrId))

  proxyDS.io.tshrId := io.consts.tshrId

  proxyDS.io.fromDir := io.fromDir

  io.toDS := proxyDS.io.toDS
  proxyDS.io.fromDS := io.fromDS

  proxyDS.io.tshr_valid := tshr_valid
  proxyDS.io.tshr_paddr := tshr_paddr

  proxyDS.io.meta_valid := meta_valid
  proxyDS.io.meta_way := meta.way
  proxyDS.io.meta_state := meta.state

  proxyDS.io.tbuf_wen_last := tshr_buffer_wen_last
  proxyDS.io.tbuf_modified := tshr_buffer_modified
  proxyDS.io.tbuf_data_0 := tshr_buffer_0
  proxyDS.io.tbuf_data_2 := tshr_buffer_2

  proxyDS.io.tshr_inactive := tshr_inactive
  proxyDS.io.tshr_inactivate := tshr_inactivate
  proxyDS.io.tshr_dealloc := tshr_dealloc

  proxyDS.io.ds_read_ahead_en := false.B // TODO: S0 Data Storage Ahead Read from TSHRCtrl
  proxyDS.io.ds_read_ahead_way := 0.U // TODO: S0 Data Storage Ahead Read from TSHRCtrl
  proxyDS.io.ds_read_ahead_arbed := false.B // TODO: S0 Data Storage Ahead Read from TSHRCtrl

  proxyDS.io.ds_read_rbeEVT_en := false.B // TODO: Decode and decide with DirResult in L2TSHR/L2RBEDSRead
  proxyDS.io.ds_read_rbeSNP_en := false.B // TODO: Decode and decide with DirResult in L2TSHR/L2RBEDSRead
  proxyDS.io.ds_read_rbeREQ_en := false.B // TODO: Decode and decide with DirResult in L2TSHR/L2RBEDSRead

  proxyDS.io.ds_read_vPipeEVT_en := false.B // EVT vPipe never reads DS
  proxyDS.io.ds_read_vPipeSNP_en := vPipeSNP.io.ds_read_en
  proxyDS.io.ds_read_vPipeREQ_en := vPipeREQ.io.ds_rd_en
  proxyDS.io.ds_read_aux_en := false.B

  proxyDS.io.ds_read_EVT_cancel := false.B
  proxyDS.io.ds_read_SNP_cancel := false.B
  proxyDS.io.ds_read_REQ_cancel := vPipeREQ.io.ds_rd_cancel

  vPipeSNP.io.ds_read_done := proxyDS.io.rd_done
  // (hitFastPathDsRespFlow): the REQ vPipe ends its DS wait on the
  // combinational DSBufRdResp pulse instead of the registered Done state.
  vPipeREQ.io.ds_rd_done := (if (paramL2.hitFastPathDsRespFlow) proxyDS.io.rd_done_flow
                             else                               proxyDS.io.rd_done)

  tshr_buffer_commit := proxyDS.io.wb_accept

  tshr_wb_done_ds := proxyDS.io.wb_done

  proxyDS.io.RXDAT_fire := io.UpRXDAT.fire || io.DnRXDAT.fire


  // 'wb_locked' refuses Directory Write & Data Storage Write on write-ready state
  // 'wb_cancel' drops non-arbiterated Directory Write & Data Storage write
  proxyDir.io.wb_locked := vPipeREQ.io.dir_wb_locked
  proxyDir.io.snoop_active := snoopAgent.io.snoop_active
  proxyDS.io.wb_locked := vPipeREQ.io.ds_wb_locked

  vPipeREQ.io.dir_wb_accept := proxyDir.io.wb_accept
  vPipeREQ.io.dir_wb_done := proxyDir.io.wb_done

  proxyDir.io.wb_cancel := vPipeREQ.io.dir_wb_cancel
  proxyDS.io.wb_cancel := vPipeREQ.io.ds_wb_cancel || vPipeSNP.io.ds_wb_cancel
  
  proxyDS.io.wb_aux := vPipeREQ.io.ds_wb_aux
}
