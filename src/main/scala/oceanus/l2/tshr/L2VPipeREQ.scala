package oceanus.l2.tshr

import chisel3._
import chisel3.util._
import oceanus.l2._
import oceanus.compactchi._
import oceanus.chi.bundle._
import oceanus.chi.opcode._
import utility._
import freechips.rocketchip.util._
import org.chipsalliance.cde.config.Parameters
import oceanus.l2.L2Directory.MetaState
import oceanus.chi.field.CHIFieldSize._
import oceanus.chi.field.CHIFieldOrder._
import oceanus.chi.field.CHIFieldMemAttr._
import oceanus.chi.field.CHIFieldResp
import xscache.oceanus.compactchi.HasCCHIParameters

object L2VPipeREQ {

  class FlitEVB(implicit val p: Parameters) extends Bundle with HasL2Params {
     val TshrId = UInt(mshrIndexWidth.W)
     val Addr = UInt(paramL2.physicalAddrWidth.W)
     val Way = UInt(wayBits.W)
  }
}

class L2VPipeREQ(clientComponents: Seq[CCHIComponent], 
                 val sliceNum: Int)(implicit val p: Parameters) 
    extends Module 
    with CHIRNFOpcodesREQ 
    with CHIRNFOpcodesRSP
    with CHIRNFOpcodesDAT
    with HasL2Params
    with HasCCHIParameters {

  val io = IO(new Bundle {
    val consts = Input(new L2TSHRConsts(sliceNum))

    val UpRXREQ = Flipped(Valid(new FlitREQStripped))
    val UpRXEVB = Flipped(Valid(new L2VPipeREQ.FlitEVB))

    val DnTXREQ = Decoupled(new CHIBundleREQ)

    val DnRXRSP = Flipped(Valid(new CHIBundleRSP))
    val DnRXDAT = Flipped(Valid(new CHIBundleDAT))

    val DnTXRSP = Decoupled(new CHIBundleRSP)
    val DnTXDAT = Decoupled(new CHIBundleDAT)

    val UpRXRSP = Flipped(Valid(new FlitUpRSP))
    val UpRXDAT = Flipped(Valid(new FlitUpDAT))

    val UpTXRSP = Decoupled(new FlitDnRSP)
    val UpTXDAT = Decoupled(new FlitDnDAT)

    val UpTXEVB = Decoupled(new L2VPipeREQ.FlitEVB)

    val tshr_paddr = Input(UInt(paramL2.physicalAddrWidth.W))

    val tshr_dirResult = Input(new L2Directory.MetaReadResult)

    // Pulses when this TSHR deallocates; per-life bookkeeping registers that
    // are not part of the free state must be cleared on it, so that nothing
    // leaks into the entry's next life.
    val tshr_dealloc = Input(Bool())

    val tshr_tag_write_en = Output(Bool())
    val tshr_meta_write_en = Output(new L2Directory.MetaWriteMask)
    val tshr_meta_write_meta = Output(new L2Directory.Meta)
    val tshr_meta_modified = Input(Bool())
    val tshr_tag_modified = Input(Bool())

    val tbuf_modified = Input(Bool())
    val tbuf_data0_valid = Input(Bool())
    val tbuf_data2_valid = Input(Bool())

    val toSA = Output(new L2SnoopAgent.PathToSnoopAgent)
    val fromSA = Input(new L2SnoopAgent.PathFromSnoopAgent)

    val blockRBE = Output(new L2RBE.PathVPipeBlock)
    val free = Output(Bool())
    // The vPipe is free, or provably becomes free at this cycle's edge.
    val free_next = Output(Bool())

    val ds_rd_en = Output(Bool())
    val ds_rd_cancel = Output(Bool())
    val ds_rd_done = Input(Bool())

    val repl_en = Output(Bool())
    val repl_reset = Output(Bool())
    val repl_retry = Input(Bool())
    val repl_done = Input(Bool())
    val repl_resp = Input(new L2Directory.ReplReadResult)
    val repl_meta = Input(new L2Directory.Meta)   // victim way meta latched at ReplRdResp

    val repl_lock_req = Output(Bool())
    val repl_lock_way = Output(UInt(wayBits.W))
    val repl_lock_conflict = Input(Bool())
    val repl_lock_release = Output(Bool())

    val dir_wb_locked = Output(Bool())
    val dir_wb_cancel = Output(Bool())
    val dir_wb_aux = Output(Bool())
    val dir_wb_accept = Input(Bool())
    val dir_wb_done = Input(Bool())

    val ds_wb_locked = Output(Bool())
    val ds_wb_cancel = Output(Bool())
    val ds_wb_aux = Output(Bool())

    val toPCreditPool = Valid(new L2PCreditPool.Entry)
    val fromPCreditPool = Input(Bool())

    val toClientTable = Output(UInt(paramCCHI.UpstreamNodeID_Width.W))
    val fromClientTable = Input(Vec(1, Bool())) // TODO: parameterize with coherent l2 client count

    val peer_unlock_dir = Output(Vec(paramL2.mshrSize, Bool()))
    val peer_unlock_ds = Output(Vec(paramL2.mshrSize, Bool()))

    val self_unlock_dir = Input(Bool())
    val self_unlock_dir_tshrId = Input(UInt(mshrIndexWidth.W))
    val self_unlock_ds = Input(Bool())

    val peer_unlock_ack = Output(Bool())
    val peer_unlock_ack_tshrId = Output(UInt(mshrIndexWidth.W))

    val self_unlock_dir_ack = Input(Bool()) // the peer refiller's Directory commit landed

    val L1EVT_active = Input(Bool())

    // Peer EVT pipe is servicing a WriteBackFull whose data beats have not
    // fully arrived yet (see L2VPipeEVT.EVT_data_pending). Gates the
    // downstream EvictBack CopyBackWrData issue so the L3 is never written
    // with the stale DS-read copy ahead of the L1's authoritative beats.
    val L1EVT_data_pending = Input(Bool())

    // Incoming EVT is WriteBackFull; clean Evict may bypass conflict-hit prefill.
    val L1EVT_wbFull = Input(Bool())

    val L2EVT_opcode = Valid(UInt(paramCHI.reqOpcodeWidth.W))
  })

  val dirResult = io.tshr_dirResult

  val dnTxnID = (io.consts.tshrId << log2Ceil(sliceNum)) | io.consts.sliceIdx

  val configNonAgedDirArb = true
  
  val configEnableMakeReadUnique = false
  val configInclusiveReadOnce = true

  // Whether, for a L1 ReadShared, the returning permission was promoted from Shared to Unique
  // when the transaction hit local US state with zero client.
  // * NOTICE: Might be necessary when Coherent L1 I-Cache (Type 2 components) supported.
  val configReadSharedPromotionFromUS = true.B

  // Whether, for a L1 ReadShared, the returning permission was demoted from Unique to Shared
  // when the transaction hit local/remote UU state with zero client, with local transition of UU to US
  // * NOTICE: Might be necessary when Coherent L1 I-Cache (Type 2 components) supported.
  val configReadSharedDemotionFromUU = false.B

  // Whether, for a L1 StashShared, the returning permission was demoted from Unique to Shared
  // when the transaction hit local/remote UU state with zero client, with local transition of UU to US
  // * NOTICE: Might be necessary when Coherent L1 I-Cache (Type 2 components) supported.
  val configStashSharedDemotionFromUU = false.B
  
  // Whether, for a L1 ReadShared, the returning permission was promoted from Shared to Unique
  // when the transaction missed with HN returning Unique
  val configReadSharedPromotionFromI = true.B

  //
  val configReadUniqueHitSPreferReadDS = false.B

  // Changes the eviction behaviour (HN transaction type selection) when EvictBack hits a clean cacheline:
  //                    ByWriteEvictFull    ByWriteEvictOrEvict Dynamic
  // Evict              0                   0                   0
  // WriteEvictFull     1                   0                   0
  // WriteEvictOrEvict  x                   1                   0
  // Trained Dynamic    x                   x                   1
  val configEvictBackByWriteEvictFull = false.B
  val configEvictBackByWriteEvictOrEvict = true.B
  val configEvictBackDynamic = false.B

  val configEvictBackRetryCancel = true.B

  // Refill locking Directory and Data Storage write if:
  // 1. A total miss read has not issued any ReplRd
  // 2. ReplRd returned with a non-Invalid way


  // -- Enchantment modules and signals of entrance-time
  val rxreq = io.UpRXREQ.bits
  val rxreq_fire = io.UpRXREQ.fire

  val rxreq_opcode = Module(new CCHIREQOpcodeDecoder)
  rxreq_opcode.io.valid := rxreq_fire
  rxreq_opcode.io.opcode := rxreq.Opcode

  val rxreq_readunique = rxreq_opcode.is(CCHIOpcode.ReadUnique)
  val rxreq_readshared = rxreq_opcode.is(CCHIOpcode.ReadShared)
  val rxreq_readonce = rxreq_opcode.is(CCHIOpcode.ReadOnce)
  val rxreq_makeunique = rxreq_opcode.is(CCHIOpcode.MakeUnique)
  val rxreq_stashshared = rxreq_opcode.is(CCHIOpcode.StashShared)
  val rxreq_stashunique = rxreq_opcode.is(CCHIOpcode.StashUnique)

  // CCHI Stash is supported only as a fire-and-forget prefetch hint: with ExpCompStash=1 the
  // completer owes the requester a CompStash on DnRSP, which this vPipe never issues.
  assert(!(rxreq_fire && (rxreq_stashshared || rxreq_stashunique) && rxreq.ExpCompStash),
    "TSHR @ %m REQ vPipe received StashShared/StashUnique with ExpCompStash=1 (CompStash response unsupported)")

  val rxevb = io.UpRXEVB.bits
  val rxevb_evictback = io.UpRXEVB.fire

  def satisfied(valid: Bool, state: UInt): (Bool, Bool) = (
    valid && (dirResult.state >= state && dirResult.hit), 
    valid && (dirResult.state < state || !dirResult.hit))

  def satisfied(opcode: CCHIOpcode, state: UInt): (Bool, Bool) =
    satisfied(rxreq_opcode.is(opcode), state)

  val (rxreq_satisfied_stashshared, rxreq_unsatisfied_stashshared) = satisfied(CCHIOpcode.StashShared, MetaState.S)
  val (rxreq_satisfied_stashunique, rxreq_unsatisfied_stashunique) = satisfied(CCHIOpcode.StashUnique, MetaState.US)
  val (rxreq_satisfied_readonce, rxreq_unsatisfied_readonce) = satisfied(CCHIOpcode.ReadOnce, MetaState.S)
  val (rxreq_satisfied_readshared, rxreq_unsatisfied_readshared) = satisfied(CCHIOpcode.ReadShared, MetaState.S)
  val (rxreq_satisfied_writeuniqueptl, rxreq_unsatisfied_writeuniqueptl) = satisfied(CCHIOpcode.WriteUniquePtl, MetaState.US)
  val (rxreq_satisfied_writeuniquefull, rxreq_unsatisfied_writeuniquefull) = satisfied(CCHIOpcode.WriteUniqueFull, MetaState.US)
  val (rxreq_satisfied_readunique, rxreq_unsatisfied_readunique) = satisfied(CCHIOpcode.ReadUnique, MetaState.US)
  val (rxreq_satisfied_makeunique, rxreq_unsatisfied_makeunique) = satisfied(CCHIOpcode.MakeUnique, MetaState.US)

  val rxreq_unsatisfied_unique = rxreq_unsatisfied_readunique ||
                                 rxreq_unsatisfied_makeunique ||
                                 rxreq_unsatisfied_stashunique

  val rxreq_hit_lock_conflict = rxreq_unsatisfied_unique &&
                                dirResult.hit && io.repl_lock_conflict

  val rxreq_hit_lock_accept = rxreq_unsatisfied_unique &&
                              dirResult.hit && !io.repl_lock_conflict

  val p_hit_lock_way = Reg(UInt(wayBits.W))

  val p_prefill_miss = RegInit(false.B)
  val p_prefill_hit = RegInit(false.B)
  val p_prefill_any = p_prefill_miss || p_prefill_hit

  assert(!(p_prefill_miss && p_prefill_hit),
    "REQ vPipe miss-prefill and conflict-hit prefill overlapped")

  val rxevb_unsatisfied_evictback = rxevb_evictback &&
                                    dirResult.state > MetaState.I && dirResult.hit &&
                                    dirResult.way === rxevb.Way &&
                                    !p_prefill_any

  val rxevb_satisfied_evictback = rxevb_evictback &&
                                  (dirResult.state === MetaState.I || !dirResult.hit || 
                                   dirResult.way =/= rxevb.Way ||
                                   p_prefill_any)
  // ----------------------------------------------------------------

  // -- Enchantment modules and signals of downstream RX channels
  val dn_rxrsp_opcode = Module(new RSPOpcodeDecoder)
  dn_rxrsp_opcode.io.valid := io.DnRXRSP.fire
  dn_rxrsp_opcode.io.opcode := io.DnRXRSP.bits.Opcode.get

  val dn_rxdat_opcode = Module(new DATOpcodeDecoder)
  dn_rxdat_opcode.io.valid := io.DnRXDAT.fire
  dn_rxdat_opcode.io.opcode := io.DnRXDAT.bits.Opcode.get
  // ----------------------------------------------------------------

  // -- Enchantment modules and signals of upstream RX channels
  val up_rxrsp_opcode = Module(new CCHIUpRSPOpcodeDecoder)
  up_rxrsp_opcode.io.valid := io.UpRXRSP.fire
  up_rxrsp_opcode.io.opcode := io.UpRXRSP.bits.Opcode

  val up_rxdat_opcode = Module(new CCHIUpDATOpcodeDecoder)
  up_rxdat_opcode.io.valid := io.UpRXDAT.fire
  up_rxdat_opcode.io.opcode := io.UpRXDAT.bits.Opcode
  // ----------------------------------------------------------------

  // -- Private payload registers and enchantment signals
  val p_rxreq = Reg(new FlitREQStripped)
  val p_rxreq_opcode = p_rxreq.Opcode
  val p_rxreq_stashshared       = p_rxreq_opcode === CCHIOpcode.StashShared.U
  val p_rxreq_stashunique       = p_rxreq_opcode === CCHIOpcode.StashUnique.U
  val p_rxreq_readnosnp         = p_rxreq_opcode === CCHIOpcode.ReadNoSnp.U
  val p_rxreq_readonce          = p_rxreq_opcode === CCHIOpcode.ReadOnce.U
  val p_rxreq_readshared        = p_rxreq_opcode === CCHIOpcode.ReadShared.U
  val p_rxreq_writenosnpptl     = p_rxreq_opcode === CCHIOpcode.WriteNoSnpPtl.U
  val p_rxreq_writenosnpfull    = p_rxreq_opcode === CCHIOpcode.WriteNoSnpFull.U
  val p_rxreq_writeuniqueptl    = p_rxreq_opcode === CCHIOpcode.WriteUniquePtl.U
  val p_rxreq_writeuniquefull   = p_rxreq_opcode === CCHIOpcode.WriteUniqueFull.U
  val p_rxreq_cleanshared       = p_rxreq_opcode === CCHIOpcode.CleanShared.U
  val p_rxreq_cleaninvalid      = p_rxreq_opcode === CCHIOpcode.CleanInvalid.U
  val p_rxreq_makeinvalid       = p_rxreq_opcode === CCHIOpcode.MakeInvalid.U
  val p_rxreq_readunique        = p_rxreq_opcode === CCHIOpcode.ReadUnique.U
  val p_rxreq_makeunique        = p_rxreq_opcode === CCHIOpcode.MakeUnique.U
  val p_rxreq_evictback         = p_rxreq_opcode === CCHIOpcode.EvictBack.U

  when (rxreq_fire) {
    p_rxreq := rxreq
  }

  // *NOTE: EvictBack opcode is not formally used in REQ channel,
  //        and this update is only intented to clear the previous opcode state.
  when (rxevb_unsatisfied_evictback) {
    p_rxreq.Opcode := CCHIOpcode.EvictBack.U
  }

  val p_rxevb = Reg(new L2VPipeREQ.FlitEVB)
  val p_rxevb_unsatisfied = RegInit(false.B)

  when (rxevb_evictback) {
    p_rxevb := io.UpRXEVB.bits
    p_rxevb_unsatisfied := rxevb_unsatisfied_evictback
  }

  val p_txreq_reissue = RegInit(false.B)
  val p_txreq_issued_opcode = Reg(UInt(paramCHI.reqOpcodeWidth.W))

  val p_retryack_pcrdtype = Reg(UInt(paramCHI.rspPCrdTypeWidth.W))
  val p_retryack_srcid = Reg(UInt(paramCHI.nodeIdWidth.W))

  val p_evict_cancelled = RegInit(false.B)

  val p_dbid = Reg(UInt(paramCHI.rspDBIDWidth.W))
  val p_homenid = Reg(UInt(paramCHI.nodeIdWidth.W))

  val p_cbwrdata_meta = Reg(new L2Directory.Meta) 

  val p_unlock_source = Reg(UInt(mshrIndexWidth.W))

  val p_prefill_miss_meta = Reg(new L2Directory.Meta)

  // --------------------------------
  val issue_up_rd_compdata_first = Wire(Bool())
  val issue_up_rd_comp = Wire(Bool())

  // ----------------------------------------------------------------

  // -- Interaction with Client Table
  io.toClientTable := Mux(rxreq_fire, rxreq.SrcID, p_rxreq.SrcID)

  val rxreq_client = io.fromClientTable
  val p_rxreq_client = io.fromClientTable

  val rxreq_client_present = (rxreq_client.asUInt & dirResult.clients.asUInt).orR
  val p_rxreq_client_present = (p_rxreq_client.asUInt & dirResult.clients.asUInt).orR

  val rxreq_peer_present = (~rxreq_client.asUInt & dirResult.clients.asUInt).orR
  val p_rxreq_peer_present = (~p_rxreq_client.asUInt & dirResult.clients.asUInt).orR
  // ----------------------------------------------------------------

  // -- State bit registers
  val w_snpresp0 = RegInit(false.B) // Waiting for response from Snoop Agent (DataID = 0)
  val w_snpresp2 = RegInit(false.B) // Waiting for response from Snoop Agent (DataID = 2)

  val s_snpcompack = RegInit(false.B) // Scheduling SnpCompAck to Snoop Agent

  val s_dn_txreq = RegInit(false.B) // Scheduling downstream TXREQ
  val w_dn_pcrdgrant = RegInit(false.B) // Waiting for P-Credit from downstream after receiving downstream RetryAck

  val w_ds_resp = RegInit(false.B) // Waiting for response from Data Storage

  val w_s_rd_dn_compack = RegInit(false.B) // Waiting to schedule downstream TXRSP CompAck of Read and normal subsequence
  val s_rd_dn_compack = RegInit(false.B) // Scheduling downstream TXRSP CompAck of Read and normal subsequence

  val w_rd_dn_data0 = RegInit(false.B) // Waiting for downstream RXDAT CompData/DataSepResp (DataID = 0)
  val w_rd_dn_data2 = RegInit(false.B) // Waiting for downstream RXDAT CompData/DataSepResp (DataID = 2)
  val w_rd_dn_comp = RegInit(false.B) // Waiting for downstream RXRSP Comp/RespSepData

  val w_rd_up_compack = RegInit(false.B) // Waiting for Upstream RXRSP CompAck

  val w_s_rd_up_compdata0 = RegInit(false.B) // Waiting to schedule upstream TXDAT CompData (DataID = 0) of Read subsequence
  val w_s_rd_up_compdata2 = RegInit(false.B) // Waiting to schedule upstream TXDAT CompData (DataID = 2) of Read subsequence
  val s_rd_up_compdata0 = RegInit(false.B) // Scheduling upstream TXDAT CompData (DataID = 0) of Read subsequence
  val s_rd_up_compdata2 = RegInit(false.B) // Scheduling upstream TXDAT CompData (DataID = 2) of Read subsequence

  val w_s_rd_up_comp = RegInit(false.B) // Waiting to schedule upstream TXRSP Comp of Read subsequence
  val s_rd_up_comp = RegInit(false.B) // Scheduling upstream TXRSP Comp for Read subsequence

  val w_unlock_dir = RegInit(false.B) // Waiting for unlocking Directory Write-Back
  val w_unlock_ds = RegInit(false.B) // Waiting for unlocking Data Storage Write-Back

  val w_s_unlock_ack = RegInit(false.B) // Waiting to schedule peer unlocking ack

  val w_s_repl = RegInit(false.B) // Waiting to schedule Directory Replacer Read
  val s_repl = RegInit(false.B) // Scheduling Directory Replacer Read

  val w_hit_lock_release = RegInit(false.B) // Waiting for the allocating response of a hit-way reservation
  val s_hit_lock_release = RegInit(false.B) // Scheduling hit-way reservation release with final allocating Directory Write-Back

  val w_s_evict_up_evict = RegInit(false.B) // Waiting to schedule local eviction RXREQ EvictBack
  val s_evict_up_evict = RegInit(false.B) // Scheduling local eviction RXREQ EvictBack

  val w_evict_s_dn_txreq = RegInit(false.B) // Waiting to schedule downstream TXREQ of EvictBack subsequence

  val w_evict_dn_comp = RegInit(false.B) // Waiting for downstream RXRSP Comp terminal of EvictBack subsequence
  val w_evict_dn_compdbid = RegInit(false.B) // Waiting for downstream RXRSP Comp/CompDBIDResp with DBID of EvictBack subsequence

  val w_s_evict_dn_cbwrdata0 = RegInit(false.B) // Waiting to schedule downstream TXDAT CopyBackWrData (DataID = 0) of CHI WriteBackFull subsequence
  val w_s_evict_dn_cbwrdata2 = RegInit(false.B) // Waiting to schedule downstream TXDAT CopyBackWrData (DataID = 2) of CHI WriteBackFull subsequence
  val s_evict_dn_cbwrdata0 = RegInit(false.B) // Scheduling downstream TXDAT CopyBackWrData (DataID = 0) of CHI WriteBackFull subsequence
  val s_evict_dn_cbwrdata2 = RegInit(false.B) // Scheduling downstream TXDAT CopyBackWrData (DataID = 2) of CHI WriteBackFull subsequence

  val s_evict_dn_compack = RegInit(false.B) // Scheduling downstream TXRSP CompAck of CHI WriteBackFull subsequence

  val w_s_evict_peer_unlock_dir = RegInit(false.B) // Waiting to schedule peer TSHR unlocking Directory Write-Back of EvictBack subsequence
  val w_s_evict_peer_unlock_ds = RegInit(false.B) // Waiting to schedule peer TSHR unlocking Data Storage Write-Back of EvictBack subsequence

  val w_evict_peer_commit_dir = RegInit(false.B) // Waiting for the refilling peer TSHR's Directory commit ack of EvictBack subsequence

  // TODO: more state bits here

  assert(!(rxreq_fire && w_snpresp0), "RXREQ fired on valid 'w_snpresp0' in TSHR @ %m REQ vPipe")
  assert(!(rxreq_fire && w_snpresp2), "RXREQ fired on valid 'w_snpresp2' in TSHR @ %m REQ vPipe")
  assert(!(rxreq_fire && s_snpcompack), "RXREQ fired on valid 's_snpcompack' in TSHR @ %m REQ vPipe")
  // ----------------------------------------------------------------

  // -- State enchantment signals
  // active_tailless: all activity bits except the upstream CompAck tail
  // (w_rd_up_compack); 'active' adds the tail back.
  val active_tailless = w_snpresp0 || w_snpresp2 || s_snpcompack ||
                        s_dn_txreq || w_dn_pcrdgrant ||
                        w_ds_resp ||
                        w_s_rd_dn_compack || s_rd_dn_compack || w_rd_dn_data0 || w_rd_dn_data2 || w_rd_dn_comp ||
                        w_s_rd_up_compdata0 || s_rd_up_compdata0 || w_s_rd_up_compdata2 || s_rd_up_compdata2 ||
                        w_s_rd_up_comp || s_rd_up_comp ||
                        w_unlock_dir || w_unlock_ds || w_s_unlock_ack ||
                        w_s_repl || s_repl || w_s_evict_up_evict || s_evict_up_evict ||
                        w_evict_s_dn_txreq ||
                        w_evict_dn_comp || w_evict_dn_compdbid ||
                        w_s_evict_dn_cbwrdata0 || w_s_evict_dn_cbwrdata2 ||
                        s_evict_dn_cbwrdata0 || s_evict_dn_cbwrdata2 ||
                        s_evict_dn_compack ||
                        w_s_evict_peer_unlock_dir || w_s_evict_peer_unlock_ds ||
                        w_evict_peer_commit_dir ||
                        w_hit_lock_release || s_hit_lock_release

  val active = active_tailless || w_rd_up_compack

  io.free := !active

  // ----------------------------------------------------------------

  // -- Interactions with Snoop Agent
  // decode upstream REQ opcodes to upstream SNP opcodes
  val sa_snpresp = io.fromSA.SnpResp
  val sa_snprespdata0 = io.fromSA.SnpRespData0
  val sa_snprespdata2 = io.fromSA.SnpRespData2

  val sa_resp_decision = sa_snpresp ||
                         sa_snprespdata0 && w_snpresp0 && w_snpresp2 ||
                         sa_snprespdata2 && w_snpresp0 && w_snpresp2

  val sa_resp_last = sa_snpresp ||
                     sa_snprespdata0 && w_snpresp0 && !w_snpresp2 ||
                     sa_snprespdata2 && !w_snpresp0 && w_snpresp2

  val sa_respdata_first = sa_snprespdata0 && w_snpresp0 && w_snpresp2 ||
                          sa_snprespdata2 && w_snpresp0 && w_snpresp2

  val sa_resp_decided = !(w_snpresp0 && w_snpresp2)

  io.toSA.SnpMakeInvalid := rxreq_opcode.is(
    CCHIOpcode.MakeInvalid
  )

  io.toSA.SnpToInvalid := rxreq_opcode.is(
    CCHIOpcode.WriteUniquePtl,
    CCHIOpcode.WriteUniqueFull,
    CCHIOpcode.CleanInvalid,
    CCHIOpcode.ReadUnique,
    CCHIOpcode.MakeUnique
  ) || rxevb_unsatisfied_evictback

  io.toSA.SnpToShared := rxreq_opcode.is(
    CCHIOpcode.ReadShared
  )

  io.toSA.SnpToClean := rxreq_opcode.is(
    CCHIOpcode.ReadOnce,
    CCHIOpcode.CleanShared
  )

  // set and unset SnpCompAck scheduling
  //  - SnpCompAck should be issued with scheduling bit set, and after all data beats to upstream/downstream
  //    were sent.
  when (io.toSA.SnpToClean) {
    // s_snpcompack := true.B
  }

  when (io.toSA.SnpCompAck) {
    // s_snpcompack := false.B
  }

  io.toSA.SnpCompAck := false.B // TODO: interact with upstream data sending

  io.toSA.CLIENTS := io.fromClientTable
  val rxreq_aliasable = rxreq_readunique || rxreq_readshared || rxreq_makeunique
  io.toSA.ALIAS := Mux(rxreq_aliasable, rxreq.TagAlias, dirResult.alias)

  io.toSA.isL2Evict := rxevb_evictback

  // waiting state transitions
  //  - SnpResp/SnpRespData0/SnpRespData2 were all allowed to be received on the same cycle of the issue of
  //    SnpMakeInvalid/SnpToInvalid/SnpToShared/SnpToClean.
  when (io.toSA.SnpMakeInvalid || io.toSA.SnpToInvalid || io.toSA.SnpToShared || io.toSA.SnpToClean) {
    w_snpresp0 := true.B
    w_snpresp2 := true.B
  }

  when (sa_snpresp || sa_snprespdata0) {
    w_snpresp0 := false.B
  }

  when (sa_snpresp || sa_snprespdata2) {
    w_snpresp2 := false.B
  }
  // ----------------------------------------------------------------

  // -- Interactions with downstream TXREQ and downstream Retry Mechanism
  // TODO: complete other CCHI opcodes

  val allow_txreq_evictback = sa_resp_decision

  val sched_txreq_evictback = rxevb_unsatisfied_evictback && !sa_resp_decision

  when (sched_txreq_evictback) {
    w_evict_s_dn_txreq := true.B
  }

  val issue_txreq_stashshared = rxreq_unsatisfied_stashshared
  val issue_txreq_stashunique = rxreq_unsatisfied_stashunique
  val issue_txreq_readonce = rxreq_unsatisfied_readonce
  val issue_txreq_readshared = rxreq_unsatisfied_readshared
  val issue_txreq_readunique = rxreq_unsatisfied_readunique
  val issue_txreq_makeunique = rxreq_unsatisfied_makeunique

  val issue_txreq_evictback = w_evict_s_dn_txreq && allow_txreq_evictback ||
                              rxevb_unsatisfied_evictback && sa_resp_decision

  val issue_txreq = issue_txreq_stashshared ||
                    issue_txreq_stashunique ||
                    issue_txreq_readonce ||
                    issue_txreq_readshared ||
                    issue_txreq_readunique ||
                    issue_txreq_makeunique ||
                    issue_txreq_evictback

  val sched_compack_txreq_rd = issue_txreq_readshared ||
                               issue_txreq_readunique ||
                               issue_txreq_makeunique ||
                               issue_txreq_stashshared ||
                               issue_txreq_stashunique ||
                               issue_txreq_readonce && configInclusiveReadOnce.B

  val reissue_txreq = io.fromPCreditPool

  val reissue_evict_cancel = configEvictBackRetryCancel &&
                             reissue_txreq &&
                             p_rxreq_evictback && // this predication might not be necessary
                             (!dirResult.hit || dirResult.state === MetaState.I)

  when (rxreq_fire || rxevb_unsatisfied_evictback) {
    // any new REQ/EVB phase: retry bookkeeping belongs to the previous life
    p_txreq_reissue := false.B
    p_evict_cancelled := false.B
  }

  when (io.DnTXREQ.fire) {
    s_dn_txreq := false.B
    p_txreq_issued_opcode := io.DnTXREQ.bits.Opcode.get
  }

  when (reissue_evict_cancel) {
    p_evict_cancelled := true.B
  }

  when (issue_txreq_evictback) {
    w_evict_s_dn_txreq := false.B
    p_evict_cancelled := false.B
  }

  when (issue_txreq || reissue_txreq) {
    s_dn_txreq := true.B
    p_txreq_reissue := reissue_txreq
  }

  when (io.tshr_dealloc) {
    p_txreq_reissue := false.B
    p_evict_cancelled := false.B
  }

  when (sched_compack_txreq_rd) {
    w_s_rd_dn_compack := true.B
  }

  when (dn_rxrsp_opcode.is(CHI_RetryAck)) {
    w_dn_pcrdgrant := true.B
    p_retryack_pcrdtype := io.DnRXRSP.bits.PCrdType.get
    p_retryack_srcid := io.DnRXRSP.bits.SrcID.get
  }

  when (io.fromPCreditPool) {
    w_dn_pcrdgrant := false.B
  }

  io.toPCreditPool.valid := w_dn_pcrdgrant
  io.toPCreditPool.bits.pCrdType := p_retryack_pcrdtype
  io.toPCreditPool.bits.srcId := p_retryack_srcid

  assert(!(io.fromPCreditPool && !w_dn_pcrdgrant), "P-Credit granted on non-retry state in TSHR @ %m REQ vPipe")
  // ----------------------------------------------------------------

  // -- Interactions with downstream CHI TXREQ channel
  val readonce_txreq_opcode = { if (configInclusiveReadOnce) CHI_ReadNotSharedDirty.U else CHI_ReadOnce.U } // TODO: better policy

  val xunique_txreq_opcode = {
    if (configEnableMakeReadUnique)
      Mux(p_txreq_reissue, 
        p_txreq_issued_opcode,
        Mux(dirResult.state === MetaState.S, CHI_MakeReadUnique.U, CHI_ReadUnique.U)
      )
    else
      CHI_ReadUnique.U
  }

  val stashunique_txreq_opcode = xunique_txreq_opcode
  val readunique_txreq_opcode = xunique_txreq_opcode

  val cleanshared_txreq_opcode = 0.U // TODO: switch between WriteCleanFull and CleanShared

  val cleaninvalid_txreq_opcode = 0.U // TODO: switch between WriteBackFull and CleanInvalid

  val makeinvalid_txreq_opcode = CHI_MakeInvalid.U // silent eviction

  val evictback_txreq_opcode = Mux(p_txreq_reissue,
                                 Mux(p_evict_cancelled, CHI_PCrdReturn.U, p_txreq_issued_opcode),
                                 Mux(dirResult.dirty, CHI_WriteBackFull.U,
                                   Mux(configEvictBackDynamic || configEvictBackByWriteEvictOrEvict, CHI_WriteEvictOrEvict.U,
                                   Mux(configEvictBackByWriteEvictFull, CHI_WriteEvictFull.U,
                                   CHI_Evict.U)))
                               )

  val txreq_opcode = ParallelMux(Seq(
    (p_rxreq_stashshared,     CHI_ReadNotSharedDirty.U),
    (p_rxreq_stashunique,     stashunique_txreq_opcode),
    (p_rxreq_readnosnp,       CHI_ReadNoSnp.U),
    (p_rxreq_readonce,        readonce_txreq_opcode),
    (p_rxreq_readshared,      CHI_ReadNotSharedDirty.U),
    (p_rxreq_writenosnpptl,   CHI_WriteNoSnpPtl.U),
    (p_rxreq_writenosnpfull,  CHI_WriteNoSnpFull.U),
    (p_rxreq_writeuniqueptl,  CHI_WriteUniquePtl.U), // TODO: only happens after L1 SA complete
    (p_rxreq_writeuniquefull, CHI_WriteUniqueFull.U), // TODO: only happens after L1 SA complete
    (p_rxreq_cleanshared,     cleanshared_txreq_opcode),
    (p_rxreq_cleaninvalid,    cleaninvalid_txreq_opcode),
    (p_rxreq_makeinvalid,     makeinvalid_txreq_opcode),
    (p_rxreq_readunique,      readunique_txreq_opcode),
    (p_rxreq_makeunique,      CHI_MakeUnique.U),
    (p_rxreq_evictback,       evictback_txreq_opcode) // TODO: only happens after L1 SA complete
  ))

  val txreq_pcrdreturn = txreq_opcode === CHI_PCrdReturn.U

  // Field 'Size':
  //    [ CCHI Opcode ]     [ CHI Opcode ]        [ Value / Source ]
  //  -  ReadShared          ReadNotSharedDirty    64B
  //  -  ReadUnique          ReadUnique            64B
  //                         MakeReadUnique        64B
  //  -  MakeUnique          MakeUnique            64B
  //  -  StashShared         ReadNotSharedDirty    64B
  //  -  StashUnique         ReadUnique            64B
  //                         MakeReadUnique        64B
  //  -  ReadOnce            ReadOnce              64B
  //                         ReadNotSharedDirty    64B (configInclusiveReadOnce)
  //  -  EvictBack           WriteBackFull         64B
  //                         WriteEvictOrEvict     64B
  //                         WriteEvictFull        64B
  //                         Evict                 64B
  //  -  *                   PCrdReturn            0 (forced at the Size assignment below)
  val txreq_size = ParallelMux(Seq(
    (p_rxreq_readshared,      Size64B.U),
    (p_rxreq_readunique,      Size64B.U),
    (p_rxreq_makeunique,      Size64B.U),
    (p_rxreq_stashshared,     Size64B.U),
    (p_rxreq_stashunique,     Size64B.U),
    (p_rxreq_readonce,        Size64B.U),
    (p_rxreq_evictback,       Size64B.U)
  ))

  // Field 'LikelyShared':
  //    [ CCHI Opcode ]     [ CHI Opcode ]        [ Value / Source ]
  //  -  ReadShared          ReadNotSharedDirty    0 (1 not utilized for now)
  //  -  ReadUnique          ReadUnique            0
  //                         MakeReadUnique        0
  //  -  MakeUnique          MakeUnique            0
  //  -  StashShared         ReadNotSharedDirty    0 (1 not utilized for now)
  //  -  StashUnique         ReadUnique            0
  //                         MakeReadUnique        0
  //  -  EvictBack           WriteBackFull         0
  //                         WriteEvictOrEvict     0
  //                         WriteEvictFull        0
  //                         Evict                 0 (required 0 for dataless)
  val txreq_likelyshared = ParallelMux(Seq(
    (p_rxreq_readshared,      false.B),
    (p_rxreq_readunique,      false.B),
    (p_rxreq_makeunique,      false.B),
    (p_rxreq_stashshared,     false.B),
    (p_rxreq_stashunique,     false.B),
    (p_rxreq_readonce,        false.B),
    (p_rxreq_evictback,       false.B)
  ))

  // Field 'Order':
  //    [ CCHI Opcode ]     [ CHI Opcode ]        [ Value / Source ]
  //  -  ReadShared          ReadNotSharedDirty    0b00 (No Ordering)
  //  -  ReadUnique          ReadUnique            0b00 (No Ordering)
  //                         MakeReadUnique        0b00 (No Ordering)
  //  -  MakeUnique          MakeUnique            0b00 (No Ordering)
  //  -  StashShared         ReadNotSharedDirty    0b00 (No Ordering)
  //  -  StashUnique         ReadUnique            0b00 (No Ordering)
  //                         MakeReadUnique        0b00 (No Ordering)
  //  -  EvictBack           WriteBackFull         0b00 (No Ordering)
  //                         WriteEvictOrEvict     0b00 (No Ordering)
  //                         WriteEvictFull        0b00 (No Ordering)
  //                         Evict                 0b00 (No Ordering)
  val txreq_order = ParallelMux(Seq(
    (p_rxreq_readshared,      NoOrdering.U),
    (p_rxreq_readunique,      NoOrdering.U),
    (p_rxreq_makeunique,      NoOrdering.U),
    (p_rxreq_stashshared,     NoOrdering.U),
    (p_rxreq_stashunique,     NoOrdering.U),
    (p_rxreq_readonce,        NoOrdering.U),
    (p_rxreq_evictback,       NoOrdering.U)
  ))

  // Field 'MemAttr':
  //    [ CCHI Opcode ]     [ CHI Opcode ]        [ Value / Source ]
  //  -  ReadShared          ReadNotSharedDirty    Cacheable + EWA + Allocate
  //  -  ReadUnique          ReadUnique            Cacheable + EWA + Allocate
  //                         MakeReadUnique        Cacheable + EWA + Allocate
  //  -  MakeUnique          MakeUnique            Cacheable + EWA
  //  -  StashShared         ReadNotSharedDirty    Cacheable + EWA + Allocate
  //  -  StashUnique         ReadUnique            Cacheable + EWA + Allocate
  //                         MakeReadUnique        Cacheable + EWA + Allocate
  //  -  ReadOnce            ReadOnce              Cacheable + EWA
  //                         ReadNotSharedDirty    Cacheable + EWA (configInclusiveReadOnce; non-allocating)
  //  -  EvictBack           WriteBackFull         Cacheable + EWA + Allocate
  //                         WriteEvictOrEvict     Cacheable + EWA + Allocate
  //                         WriteEvictFull        Cacheable + EWA + Allocate
  //                         Evict                 Cacheable + EWA + Allocate
  val txreq_memattr = ParallelMux(Seq(
    (p_rxreq_readshared,      Cacheable.U | EWA.U | Allocate.U),
    (p_rxreq_readunique,      Cacheable.U | EWA.U | Allocate.U),
    (p_rxreq_makeunique,      Cacheable.U | EWA.U),
    (p_rxreq_stashshared,     Cacheable.U | EWA.U | Allocate.U),
    (p_rxreq_stashunique,     Cacheable.U | EWA.U | Allocate.U),
    (p_rxreq_readonce,        Cacheable.U | EWA.U),
    (p_rxreq_evictback,       Cacheable.U | EWA.U | Allocate.U)
  ))

  // Field 'SnpAttr':
  //    [ CCHI Opcode ]     [ CHI Opcode ]        [ Value / Source ]
  //  -  ReadShared          ReadNotSharedDirty    1
  //  -  ReadUnique          ReadUnique            1
  //                         MakeReadUnique        1
  //  -  MakeUnique          MakeUnique            1
  //  -  StashShared         ReadNotSharedDirty    1
  //  -  StashUnique         ReadUnique            1
  //                         MakeReadUnique        1
  //  -  ReadOnce            ReadOnce              1
  //                         ReadNotSharedDirty    1 (configInclusiveReadOnce)
  //  -  EvictBack           WriteBackFull         1
  //                         WriteEvictOrEvict     1
  //                         WriteEvictFull        1
  //                         Evict                 1
  val txreq_snpattr = ParallelMux(Seq(
    (p_rxreq_readshared,      true.B),
    (p_rxreq_readunique,      true.B),
    (p_rxreq_makeunique,      true.B),
    (p_rxreq_stashshared,     true.B),
    (p_rxreq_stashunique,     true.B),
    (p_rxreq_readonce,        true.B),
    (p_rxreq_evictback,       true.B)
  ))

  // Field 'ExpCompAck':
  //    [ CCHI Opcode ]     [ CHI Opcode ]        [ Value / Source ]
  //  -  ReadShared          ReadNotSharedDirty    1
  //  -  ReadUnique          ReadUnique            1
  //                         MakeReadUnique        1
  //  -  MakeUnique          MakeUnique            1
  //  -  StashShared         ReadNotSharedDirty    1
  //  -  StashUnique         ReadUnique            1
  //                         MakeReadUnique        1
  //  -  ReadOnce            ReadOnce              0
  //                         ReadNotSharedDirty    1 (configInclusiveReadOnce)
  //  -  EvictBack           WriteEvictOrEvict     1
  //                         WriteBackFull         0
  //                         WriteEvictFull        0
  //                         Evict                 0
  val txreq_expcompack = ParallelMux(Seq(
    (p_rxreq_readshared,      true.B),
    (p_rxreq_readunique,      true.B),
    (p_rxreq_makeunique,      true.B),
    (p_rxreq_stashshared,     true.B),
    (p_rxreq_stashunique,     true.B),
    (p_rxreq_readonce,        configInclusiveReadOnce.B),
    (p_rxreq_evictback && (evictback_txreq_opcode === CHI_WriteEvictOrEvict.U), true.B)
  ))

  io.DnTXREQ.valid := s_dn_txreq
  io.DnTXREQ.bits.QoS.get := 14.U // Default at 14, (**DOT NOT use 15**, maybe better policy in future)
  io.DnTXREQ.bits.TgtID.get := 0.U // Support E-SAM only currently
  io.DnTXREQ.bits.SrcID.get := io.consts.nodeId
  io.DnTXREQ.bits.TxnID.get := Mux(txreq_pcrdreturn, 0.U, dnTxnID)
  io.DnTXREQ.bits.ReturnNID_StashNID_SLCRepHint.get := 0.U // Not providing SLCRepHint/StashNID, default to 0
  io.DnTXREQ.bits.StashNIDValid_Endian_Deep.get := 0.U
  io.DnTXREQ.bits.ReturnTxnID_StashLPIDValid_StashLPID.get := 0.U
  io.DnTXREQ.bits.Opcode.get := txreq_opcode
  io.DnTXREQ.bits.Size.get := Mux(txreq_pcrdreturn, 0.U, txreq_size)
  assert(!(io.DnTXREQ.fire && io.DnTXREQ.bits.Opcode.get =/= CHI_PCrdReturn.U && io.DnTXREQ.bits.Size.get === 0.U),
    "TSHR @ %m REQ vPipe issuing non-PCrdReturn TXREQ with Size=0")
  io.DnTXREQ.bits.Addr.get := Mux(txreq_pcrdreturn, 0.U, io.tshr_paddr)
  io.DnTXREQ.bits.NS.get := 0.U // TODO: confirm default NS value or NS mechanism
  io.DnTXREQ.bits.LikelyShared.get := Mux(txreq_pcrdreturn, 0.U, txreq_likelyshared)
  io.DnTXREQ.bits.AllowRetry.get := !p_txreq_reissue
  io.DnTXREQ.bits.Order.get := Mux(txreq_pcrdreturn, 0.U, txreq_order)
  io.DnTXREQ.bits.PCrdType.get := Mux(!p_txreq_reissue, 0.U, p_retryack_pcrdtype)
  io.DnTXREQ.bits.MemAttr.get := Mux(txreq_pcrdreturn, 0.U, txreq_memattr)
  io.DnTXREQ.bits.SnpAttr_DoDWT.get := Mux(txreq_pcrdreturn, 0.U, txreq_snpattr)
  io.DnTXREQ.bits.LPID_PGroupID_StashGroupID_TagGroupID.get := 0.U // Not supporting Persistence and MTE
  io.DnTXREQ.bits.Excl_SnoopMe.get := 0.U
  io.DnTXREQ.bits.ExpCompAck.get := Mux(txreq_pcrdreturn, 0.U, txreq_expcompack)
  io.DnTXREQ.bits.TagOp.get := 0.U // Not supporting MTE
  io.DnTXREQ.bits.TraceTag.get := 0.U // TODO: maybe wire with L1-L2 TraceTag
  io.DnTXREQ.bits.MPAM.foreach(_ := 0.U) // TODO: Not supporting MTE, wire up with NS bit
  io.DnTXREQ.bits.RSVDC.foreach(_ := 0.U)

  val fire_txreq_pcrdreturn = io.DnTXREQ.fire && io.DnTXREQ.bits.Opcode.get === CHI_PCrdReturn.U

  val fire_txreq_evict = io.DnTXREQ.fire && io.DnTXREQ.bits.Opcode.get === CHI_Evict.U
  val fire_txreq_writeevictfull = io.DnTXREQ.fire && io.DnTXREQ.bits.Opcode.get === CHI_WriteEvictFull.U
  val fire_txreq_writeevictorevict = io.DnTXREQ.fire && io.DnTXREQ.bits.Opcode.get === CHI_WriteEvictOrEvict.U
  val fire_txreq_writebackfull = io.DnTXREQ.fire && io.DnTXREQ.bits.Opcode.get === CHI_WriteBackFull.U
  // ----------------------------------------------------------------

  // -- Interactions with downstream CHI RXRSP, RXDAT channel
  // TODO: handle RespErr here or somewhere else
  val dn_rxdat_compdata = dn_rxdat_opcode.is(CHI_CompData)
  val dn_rxdat_compdata_first = dn_rxdat_compdata && w_rd_dn_data0 && w_rd_dn_data2
  val dn_rxdat_datasepresp = dn_rxdat_opcode.is(CHI_DataSepResp)
  val dn_rxdat_datasepresp_first = dn_rxdat_datasepresp && w_rd_dn_data0 && w_rd_dn_data2

  val dn_rxrsp_comp = dn_rxrsp_opcode.is(CHI_Comp)
  val dn_rxrsp_respsepdata = dn_rxrsp_opcode.is(CHI_RespSepData)
  val dn_rxrsp_compdbidresp = dn_rxrsp_opcode.is(CHI_CompDBIDResp)

  // Unsatisfied ReadShared/ReadUnique/StashShared/StashUnique/ReadOnce
  // expects a downstream Comp or CompData / DataSepResp + RespSepData
  val expect_dn_rd_comp_and_data = rxreq_unsatisfied_readshared ||
                                   rxreq_unsatisfied_readunique ||
                                   rxreq_unsatisfied_stashshared ||
                                   rxreq_unsatisfied_stashunique ||
                                   rxreq_unsatisfied_readonce

  // MakeUnique expects a dataless downstream Comp only (no data beats will follow)
  val expect_dn_rd_comp_only = rxreq_unsatisfied_makeunique

  val expect_dn_evict_comp = fire_txreq_evict
  val expect_dn_evict_compdbid = fire_txreq_writebackfull ||
                                 fire_txreq_writeevictfull ||
                                 fire_txreq_writeevictorevict

  val cancel_dn_evict_comp = w_evict_dn_comp && fire_txreq_pcrdreturn
  val cancel_dn_evict_compdbid = w_evict_dn_compdbid && fire_txreq_pcrdreturn

  val dn_rd_decided = !(w_rd_dn_data0 && w_rd_dn_data2)

  when (expect_dn_rd_comp_and_data) {
    w_rd_dn_data0 := true.B
    w_rd_dn_data2 := true.B
    w_rd_dn_comp := true.B
  }

  when (expect_dn_rd_comp_only) {
    w_rd_dn_comp := true.B
  }

  when (expect_dn_evict_comp) {
    w_evict_dn_comp := true.B
  }

  when (expect_dn_evict_compdbid) {
    w_evict_dn_compdbid := true.B
  }

  when (dn_rxrsp_comp) {
    p_homenid := io.DnRXRSP.bits.SrcID.get
    p_dbid := io.DnRXRSP.bits.DBID.get
    w_rd_dn_data0 := false.B
    w_rd_dn_data2 := false.B
    w_rd_dn_comp := false.B
    w_evict_dn_comp := false.B
    w_evict_dn_compdbid := false.B
  }

  when (dn_rxrsp_compdbidresp) {
    p_homenid := io.DnRXRSP.bits.SrcID.get
    p_dbid := io.DnRXRSP.bits.DBID.get
    w_evict_dn_compdbid := false.B
  }

  when (cancel_dn_evict_comp) {
    w_evict_dn_comp := false.B
  }

  when (cancel_dn_evict_compdbid) {
    w_evict_dn_compdbid := false.B
  }

  when (dn_rxdat_compdata) {
    p_homenid := io.DnRXDAT.bits.HomeNID.get
    p_dbid := io.DnRXDAT.bits.DBID.get
    w_rd_dn_comp := false.B
    when (io.DnRXDAT.bits.DataID.get === 0.U) {
      w_rd_dn_data0 := false.B
    }.otherwise {
      w_rd_dn_data2 := false.B
    }
  }

  when (dn_rxrsp_respsepdata) {
    p_homenid := io.DnRXRSP.bits.SrcID.get
    p_dbid := io.DnRXRSP.bits.DBID.get
    w_rd_dn_comp := false.B
  }

  when (dn_rxdat_datasepresp) {
    p_homenid := io.DnRXDAT.bits.HomeNID.get
    p_dbid := io.DnRXDAT.bits.DBID.get
    when (io.DnRXDAT.bits.DataID.get === 0.U) {
      w_rd_dn_data0 := false.B
    }.otherwise {
      w_rd_dn_data2 := false.B
    }
  }

  assert(!(w_evict_dn_comp && dn_rxrsp_compdbidresp), "")
  // ----------------------------------------------------------------

  // -- Interactions with downstream CHI TXRSP channel
  val dn_txrsp_compack = io.DnTXRSP.fire && io.DnTXRSP.bits.Opcode.get === CHI_CompAck.U

  val allow_dn_rd_compack = dn_rxdat_compdata_first ||
                            dn_rxrsp_comp ||
                            dn_rxrsp_respsepdata

  val issue_dn_rd_compack = w_s_rd_dn_compack && allow_dn_rd_compack

  val issue_dn_evict_compack = w_evict_dn_compdbid && dn_rxrsp_comp

  when (issue_dn_rd_compack) {
    w_s_rd_dn_compack := false.B
    s_rd_dn_compack := true.B
  }

  when (issue_dn_evict_compack) {
    s_evict_dn_compack := true.B
  }

  when (dn_txrsp_compack) {
    s_rd_dn_compack := false.B
    s_evict_dn_compack := false.B
  }

  io.DnTXRSP.valid := s_rd_dn_compack || s_evict_dn_compack
  io.DnTXRSP.bits.QoS.get := 14.U // Default at 14, (**DOT NOT use 15**, maybe better policy in future)
  io.DnTXRSP.bits.TgtID.get := p_homenid
  io.DnTXRSP.bits.SrcID.get := io.consts.nodeId
  io.DnTXRSP.bits.TxnID.get := p_dbid
  io.DnTXRSP.bits.Opcode.get := CHI_CompAck.U
  io.DnTXRSP.bits.RespErr.get := 0.U
  io.DnTXRSP.bits.Resp.get := 0.U
  io.DnTXRSP.bits.FwdState(0.U)
  io.DnTXRSP.bits.CBusy.get := 0.U
  io.DnTXRSP.bits.DBID(0.U)
  io.DnTXRSP.bits.PCrdType.get := 0.U
  io.DnTXRSP.bits.TagOp.get := 0.U
  io.DnTXRSP.bits.TraceTag.get := 0.U // TODO: maybe wire with L2-L3 TraceTag
  // ----------------------------------------------------------------
  
  // -- Interactions with downstream CHI TXDAT channel
  val dn_txdat_cbwrdata0 = io.DnTXDAT.fire && io.DnTXDAT.bits.Opcode.get === CHI_CopyBackWrData.U && io.DnTXDAT.bits.DataID.get === 0.U
  val dn_txdat_cbwrdata2 = io.DnTXDAT.fire && io.DnTXDAT.bits.Opcode.get === CHI_CopyBackWrData.U && io.DnTXDAT.bits.DataID.get === 2.U

  // When the eviction snoop is answered by a separate WriteBackFull, the L1's
  // authoritative beats are still in flight while the EVT pipe has them
  // pending: hold the downstream issue so the L3 is never written with the
  // stale DS-read copy ahead of those beats (lbm_6382 gsim-struct abort).
  // Clean Evict episodes never assert the gate, so they are not stalled.
  val allow_dn_evict_cbwrdata0 = !w_snpresp0 && !w_ds_resp && io.tbuf_data0_valid && !io.L1EVT_data_pending
  val allow_dn_evict_cbwrdata2 = !w_snpresp2 && !w_ds_resp && io.tbuf_data2_valid && !io.L1EVT_data_pending

  val sched_dn_evict_cbwrdata = w_evict_dn_compdbid && dn_rxrsp_compdbidresp

  val issue_dn_evict_cbwrdata0 = w_s_evict_dn_cbwrdata0 && allow_dn_evict_cbwrdata0
  val issue_dn_evict_cbwrdata2 = w_s_evict_dn_cbwrdata2 && allow_dn_evict_cbwrdata2

  when (sched_dn_evict_cbwrdata) {
    p_cbwrdata_meta := io.tshr_dirResult
    w_s_evict_dn_cbwrdata0 := true.B
    w_s_evict_dn_cbwrdata2 := true.B
  }

  when (issue_dn_evict_cbwrdata0) {
    w_s_evict_dn_cbwrdata0 := false.B
    s_evict_dn_cbwrdata0 := true.B
  }

  when (issue_dn_evict_cbwrdata2) {
    w_s_evict_dn_cbwrdata2 := false.B
    s_evict_dn_cbwrdata2 := true.B
  }

  when (dn_txdat_cbwrdata0) {
    s_evict_dn_cbwrdata0 := false.B
  }

  when (dn_txdat_cbwrdata2) {
    s_evict_dn_cbwrdata2 := false.B
  }

  io.DnTXDAT.valid := s_evict_dn_cbwrdata0 || s_evict_dn_cbwrdata2
  io.DnTXDAT.bits.QoS.get := 14.U // Default at 14, (**DOT NOT use 15**, maybe better policy in future)
  io.DnTXDAT.bits.TgtID.get := p_homenid
  io.DnTXDAT.bits.SrcID.get := io.consts.nodeId
  io.DnTXDAT.bits.TxnID.get := p_dbid
  io.DnTXDAT.bits.HomeNID.get := 0.U
  io.DnTXDAT.bits.Opcode.get := CHI_CopyBackWrData.U
  io.DnTXDAT.bits.RespErr.get := 0.U // TODO: RespErr
  io.DnTXDAT.bits.Resp.get := Mux(
    p_cbwrdata_meta.state === MetaState.US || p_cbwrdata_meta.state === MetaState.UU,
    Mux(p_cbwrdata_meta.dirty, CHIFieldResp.CopyBackWrData_UD_PD.U, CHIFieldResp.CopyBackWrData_UC.U),
    Mux(p_cbwrdata_meta.state === MetaState.S, CHIFieldResp.CopyBackWrData_SC.U, CHIFieldResp.CopyBackWrData_I.U)
  )
  io.DnTXDAT.bits.FwdState(0.U)
  io.DnTXDAT.bits.CBusy.get := 0.U
  io.DnTXDAT.bits.DBID.get := 0.U
  io.DnTXDAT.bits.CCID.get := 0.U
  io.DnTXDAT.bits.DataID.get := Mux(s_evict_dn_cbwrdata0, 0.U, 2.U)
  io.DnTXDAT.bits.TagOp.get := 0.U
  io.DnTXDAT.bits.Tag.get := 0.U
  io.DnTXDAT.bits.TU.get := 0.U
  io.DnTXDAT.bits.TraceTag.get := 0.U // TODO: maybe wire with L2-L3 TraceTag
  io.DnTXDAT.bits.RSVDC.foreach(_ := 0.U)
  io.DnTXDAT.bits.BE.get := Fill(paramCHI.datBEWidth, p_cbwrdata_meta.state =/= MetaState.I)
  io.DnTXDAT.bits.Data.get := DontCare
  io.DnTXDAT.bits.DataCheck.foreach(_ := DontCare)
  io.DnTXDAT.bits.Poison.foreach(_ := DontCare)
  // *NOTICE: Data, DataCheck, Poison is assigned in TSHR top
  // ----------------------------------------------------------------

  // -- Interactions with upstream CCHI RXRSP channel
  val up_rxrsp_compack = up_rxrsp_opcode.is(CCHIOpcode.CompAck)

  val expect_up_rd_compack_sat = rxreq_satisfied_readunique ||
                                 rxreq_satisfied_readshared ||
                                 rxreq_satisfied_makeunique

  val expect_up_rd_compack_unsat = (p_rxreq_readunique || p_rxreq_readshared || p_rxreq_makeunique) &&
                                   (dn_rxrsp_comp || dn_rxdat_compdata_first || dn_rxdat_datasepresp_first)

  val expect_up_rd_compack = expect_up_rd_compack_sat || expect_up_rd_compack_unsat

  when (expect_up_rd_compack) {
    w_rd_up_compack := true.B
  }

  when (up_rxrsp_compack) {
    w_rd_up_compack := false.B
  }

  assert(!(up_rxrsp_compack && !w_rd_up_compack), "Receiving upstream RXRSP CompAck on non-valid 'w_rd_up_compack' in TSHR @ %m REQ vPipe")

  // T1-d (hitFastPathEarlyInactive): exact one-cycle-ahead free prediction: if
  // no bit is set the vPipe is free already; if the sole remaining bit is
  // w_rd_up_compack and the CompAck arrives combinationally this cycle, every
  // bit is clear at the edge. Any other last-bit case conservatively keeps the
  // unpredicted timing.
  io.free_next := !active || (!active_tailless && w_rd_up_compack && up_rxrsp_compack)
  assert(!(dn_rxrsp_comp && !(w_rd_dn_comp || w_evict_dn_comp || w_evict_dn_compdbid)),
    "TSHR @ %m REQ vPipe received downstream Comp on non-valid expectation")
  assert(!(dn_rxdat_compdata && !(w_rd_dn_data0 || w_rd_dn_data2)),
    "TSHR @ %m REQ vPipe received downstream CompData with no data expectation (mesh/protocol anomaly)")
  // ----------------------------------------------------------------

  // -- Interactions with TSHR local meta
  val dn_rxrsp_comp_UD_PD = dn_rxrsp_comp && io.DnRXRSP.bits.Resp.get === CHIFieldResp.Comp_UD_PD.U

  val dn_rxdat_compdata_first_I = dn_rxdat_compdata_first && io.DnRXDAT.bits.Resp.get === CHIFieldResp.CompData_I.U
  val dn_rxdat_compdata_first_UC = dn_rxdat_compdata_first && io.DnRXDAT.bits.Resp.get === CHIFieldResp.CompData_UC.U
  val dn_rxdat_compdata_first_UD_PD = dn_rxdat_compdata_first && io.DnRXDAT.bits.Resp.get === CHIFieldResp.CompData_UD_PD.U
  val dn_rxdat_compdata_first_SC = dn_rxdat_compdata_first && io.DnRXDAT.bits.Resp.get === CHIFieldResp.CompData_SC.U

  val dn_rxdat_datasepresp_first_UC = dn_rxdat_datasepresp_first && io.DnRXDAT.bits.Resp.get === CHIFieldResp.DataSepResp_UC.U
  val dn_rxdat_datasepresp_first_UD_PD = dn_rxdat_datasepresp_first && io.DnRXDAT.bits.Resp.get === CHIFieldResp.DataSepResp_UD_PD.U
  val dn_rxdat_datasepresp_first_SC = dn_rxdat_datasepresp_first && io.DnRXDAT.bits.Resp.get === CHIFieldResp.DataSepResp_SC.U

  when (p_rxreq_readonce && dn_rxdat_compdata_first) {
    if (configInclusiveReadOnce) {
      assert(dn_rxdat_compdata_first_SC || dn_rxdat_compdata_first_UC || dn_rxdat_compdata_first_UD_PD,
        "The subsequent of upstream ReadOnce received CompData with unexpected Resp from downstream (expecting SC, UC, UD_PD)")
    } else {
      assert(dn_rxdat_compdata_first_I || dn_rxdat_compdata_first_UC,
        "The subsequent of upstream ReadOnce received CompData with unexpected Resp from downstream (expecting I, UC)")
    }
  }

  when (p_rxreq_readonce && dn_rxdat_datasepresp_first) {
    if (configInclusiveReadOnce) {
      assert(dn_rxdat_datasepresp_first_SC || dn_rxdat_datasepresp_first_UC || dn_rxdat_datasepresp_first_UD_PD,
        "The subsequent of upstream ReadOnce received DataSepResp with unexpected Resp from downstream (expecting SC, UC, UD_PD)")
    } else {
      assert(dn_rxdat_datasepresp_first_UC,
        "The subsequent of upstream ReadOnce received DataSepResp with unexpected Resp from downstream (expecting UC)")
    }
  }

  assert(!(p_rxreq_readonce && dn_rxrsp_comp),
    "The subsequent of upstream ReadOnce received an unexpected dataless Comp from downstream")

  // - MakeUnique related meta/tag updates
  val meta_wr_state_makeunique_UU_sat = active && p_rxreq_makeunique &&
                                        sa_resp_decision &&
                                        dirResult.state === MetaState.US

  val meta_wr_state_makeunique_UU_unsat = active && p_rxreq_makeunique &&
                                          dn_rxrsp_comp

  val meta_wr_state_makeunique_UU = meta_wr_state_makeunique_UU_sat || meta_wr_state_makeunique_UU_unsat

  val meta_wr_dirty_makeunique_set = active && p_rxreq_makeunique &&
                                     dn_rxrsp_comp

  val meta_wr_client_makeunique_set = active && p_rxreq_makeunique &&
                                      issue_up_rd_comp

  val meta_wr_alias_makeunique = meta_wr_client_makeunique_set
  // --------------------------------

  // - ReadUnique related meta/tag updates
  val meta_wr_state_readunique_UU_sat = active && p_rxreq_readunique &&
                                        sa_resp_decision &&
                                        dirResult.state === MetaState.US

  // 'Resp' not checked for this, because it was already constrained by CHI specification, and only goes to UU
  // with assertions provided
  val meta_wr_state_readunique_UU_unsat = active && p_rxreq_readunique && /*(
                                            dn_rxdat_compdata_first_UC ||
                                            dn_rxdat_compdata_first_UD_PD ||
                                            dn_rxdat_datasepresp_first_UC ||
                                            dn_rxdat_datasepresp_first_UD_PD
                                          ) && */ (dn_rxdat_compdata_first || dn_rxdat_datasepresp_first || dn_rxrsp_comp)

  assert(!(meta_wr_state_readunique_UU_unsat && dn_rxdat_compdata_first && !dn_rxdat_compdata_first_UC && !dn_rxdat_compdata_first_UD_PD),
    "The subsequent of upstream ReadUnique received CompData with unexpected Resp from downstream (expecting UC, UD_PD)")
  assert(!(meta_wr_state_readunique_UU_unsat && dn_rxdat_datasepresp_first && !dn_rxdat_datasepresp_first_UC && !dn_rxdat_datasepresp_first_UD_PD),
    "The subsequent of upstream ReadUnique received DataSepResp with unexpected Resp from downstream (expecting UC, UD_PD)")

  val meta_wr_state_readunique_UU = meta_wr_state_readunique_UU_sat || meta_wr_state_readunique_UU_unsat

  val meta_wr_dirty_readunique_set = active && p_rxreq_readunique &&
                                     (dn_rxdat_compdata_first_UD_PD || dn_rxdat_datasepresp_first_UD_PD || dn_rxrsp_comp_UD_PD)

  val meta_wr_client_readunique_set = active && p_rxreq_readunique &&
                                      (issue_up_rd_comp || issue_up_rd_compdata_first)

  val meta_wr_alias_readunique = meta_wr_client_readunique_set
  // --------------------------------

  // - ReadShared related meta/tag updates
  val meta_wr_state_readshared_UU_sat = active && p_rxreq_readshared &&
                                        sa_resp_decision &&
                                        dirResult.state === MetaState.US &&
                                        !p_rxreq_peer_present &&
                                        configReadSharedPromotionFromUS

  val meta_wr_state_readshared_US_sat = active && p_rxreq_readshared &&
                                        sa_resp_decision &&
                                        dirResult.state === MetaState.UU &&
                                        (configReadSharedDemotionFromUU || p_rxreq_peer_present)

  val meta_wr_state_readshared_UU_unsat = active && p_rxreq_readshared && (
                                            dn_rxdat_compdata_first_UC ||
                                            dn_rxdat_compdata_first_UD_PD ||
                                            dn_rxdat_datasepresp_first_UC ||
                                            dn_rxdat_datasepresp_first_UD_PD
                                          ) && !configReadSharedDemotionFromUU

  val meta_wr_state_readshared_US_unsat = active && p_rxreq_readshared && (
                                            dn_rxdat_compdata_first_UC ||
                                            dn_rxdat_compdata_first_UD_PD ||
                                            dn_rxdat_datasepresp_first_UC ||
                                            dn_rxdat_datasepresp_first_UD_PD
                                          ) && configReadSharedDemotionFromUU

  val meta_wr_state_readshared_S_unsat = active && p_rxreq_readshared && (
                                            dn_rxdat_compdata_first_SC ||
                                            dn_rxdat_datasepresp_first_SC  
                                         )

  val meta_wr_state_readshared_UU = meta_wr_state_readshared_UU_sat ||
                                    meta_wr_state_readshared_UU_unsat
  val meta_wr_state_readshared_US = meta_wr_state_readshared_US_sat ||
                                    meta_wr_state_readshared_US_unsat
  val meta_wr_state_readshared_S = meta_wr_state_readshared_S_unsat

  val meta_wr_dirty_readshared_set = active && p_rxreq_readshared &&
                                     (dn_rxdat_compdata_first_UD_PD || dn_rxdat_datasepresp_first_UD_PD)

  val meta_wr_client_readshared_set = active && p_rxreq_readshared &&
                                      (issue_up_rd_comp || issue_up_rd_compdata_first)

  val meta_wr_alias_readshared = meta_wr_client_readshared_set
  // --------------------------------

  // - ReadOnce related meta/tag updates
  val sa_resp_unique = io.fromSA.RESP === CCHIResp.UC.U ||
                       io.fromSA.RESP === CCHIResp.UC_PD.U

  val meta_wr_state_readonce_US_sat = active && p_rxreq_readonce &&
                                      sa_resp_decision &&
                                      dirResult.state === MetaState.UU &&
                                      dirResult.clients.asUInt.orR &&
                                      !sa_resp_unique

  val meta_wr_state_readonce_UU_unsat = active && p_rxreq_readonce &&
                                        configInclusiveReadOnce.B && (
                                          dn_rxdat_compdata_first_UC ||
                                          dn_rxdat_compdata_first_UD_PD ||
                                          dn_rxdat_datasepresp_first_UC ||
                                          dn_rxdat_datasepresp_first_UD_PD
                                        )

  val meta_wr_state_readonce_S_unsat = active && p_rxreq_readonce &&
                                       configInclusiveReadOnce.B && (
                                         dn_rxdat_compdata_first_SC ||
                                         dn_rxdat_datasepresp_first_SC
                                       )

  val meta_wr_state_readonce_UU = meta_wr_state_readonce_UU_unsat
  val meta_wr_state_readonce_US = meta_wr_state_readonce_US_sat
  val meta_wr_state_readonce_S  = meta_wr_state_readonce_S_unsat

  val meta_wr_dirty_readonce_set = active && p_rxreq_readonce &&
                                   configInclusiveReadOnce.B && (
                                     dn_rxdat_compdata_first_UD_PD ||
                                     dn_rxdat_datasepresp_first_UD_PD
                                   )
  // --------------------------------

  // - StashUnique related meta/tag updates
  val meta_wr_state_stashunique_UU_unsat = active && p_rxreq_stashunique &&
                                           (dn_rxdat_compdata_first || dn_rxdat_datasepresp_first || dn_rxrsp_comp) &&
                                           !(dirResult.hit && dirResult.clients.orR)

  val meta_wr_state_stashunique_US_unsat = active && p_rxreq_stashunique &&
                                           (dn_rxdat_compdata_first || dn_rxdat_datasepresp_first || dn_rxrsp_comp) &&
                                           dirResult.hit && dirResult.clients.orR

  assert(!(meta_wr_state_stashunique_US_unsat && dn_rxdat_compdata_first && !dn_rxdat_compdata_first_UC && !dn_rxdat_compdata_first_UD_PD),
    "The subsequent of upstream StashUnique received CompData with unexpected Resp from downstream (expecting UC, UD_PD)")
  assert(!(meta_wr_state_stashunique_US_unsat && dn_rxdat_datasepresp_first && !dn_rxdat_datasepresp_first_UC && !dn_rxdat_datasepresp_first_UD_PD),
    "The subsequent of upstream StashUnique received DataSepResp with unexpected Resp from downstream (expecting UC, UD_PD)")

  val meta_wr_state_stashunique_UU = meta_wr_state_stashunique_UU_unsat
  val meta_wr_state_stashunique_US = meta_wr_state_stashunique_US_unsat

  val meta_wr_dirty_stashunique_set = active && p_rxreq_stashunique &&
                                      (dn_rxdat_compdata_first_UD_PD || dn_rxdat_datasepresp_first_UD_PD || dn_rxrsp_comp_UD_PD)
  // --------------------------------

  // Hold an unsatisfied initial-hit way until the original request's final
  // allocating response has been committed by a final allocating DirWb.
  val hit_lock_response = w_hit_lock_release && (
                            (p_rxreq_makeunique && dn_rxrsp_comp) ||
                            ((p_rxreq_readunique || p_rxreq_stashunique) && (
                              dn_rxdat_compdata_first ||
                              dn_rxdat_datasepresp_first ||
                              dn_rxrsp_comp
                            ))
                          )

  when (hit_lock_response) {
    w_hit_lock_release := false.B
    s_hit_lock_release := true.B
  }

  when (io.dir_wb_accept && s_hit_lock_release) {
    w_hit_lock_release := false.B
    s_hit_lock_release := false.B
  }

  assert(!(rxreq_fire && (w_hit_lock_release || s_hit_lock_release)),
    "RXREQ entered while a previous hit-way reservation was still active")

  when (io.tshr_dealloc) {
    assert(!(w_hit_lock_release || s_hit_lock_release),
      "TSHR deallocated with a live hit-way reservation")
    w_hit_lock_release := false.B
    s_hit_lock_release := false.B
  }
  // --------------------------------

  // - StashShared related meta/tag updates
  val meta_wr_state_stashshared_UU_unsat = active && p_rxreq_stashshared && (
                                             dn_rxdat_compdata_first_UC ||
                                             dn_rxdat_compdata_first_UD_PD ||
                                             dn_rxdat_datasepresp_first_UC ||
                                             dn_rxdat_datasepresp_first_UD_PD
                                           ) && !configStashSharedDemotionFromUU

  val meta_wr_state_stashshared_US_unsat = active && p_rxreq_stashshared && (
                                             dn_rxdat_compdata_first_UC ||
                                             dn_rxdat_compdata_first_UD_PD ||
                                             dn_rxdat_datasepresp_first_UC ||
                                             dn_rxdat_datasepresp_first_UD_PD
                                           ) && configStashSharedDemotionFromUU

  val meta_wr_state_stashshared_S_unsat = active && p_rxreq_stashshared && (
                                            dn_rxdat_compdata_first_SC ||
                                            dn_rxdat_datasepresp_first_SC
                                          )

  val meta_wr_state_stashshared_UU = meta_wr_state_stashshared_UU_unsat
  val meta_wr_state_stashshared_US = meta_wr_state_stashshared_US_unsat
  val meta_wr_state_stashshared_S = meta_wr_state_stashshared_S_unsat

  val meta_wr_dirty_stashshared_set = active && p_rxreq_stashshared &&
                                      (dn_rxdat_compdata_first_UD_PD || dn_rxdat_datasepresp_first_UD_PD)
  // --------------------------------

  // - EvictBack related meta/tag updates
  val meta_wr_state_evictback_I_evict = fire_txreq_evict

  val meta_wr_state_evictback_I_write = w_evict_dn_compdbid && 
                                        (dn_rxrsp_comp || dn_rxrsp_compdbidresp)

  val meta_wr_state_evictback_I = meta_wr_state_evictback_I_evict ||
                                  meta_wr_state_evictback_I_write

  val meta_wr_dirty_evictback_clr = meta_wr_state_evictback_I
  // --------------------------------

  val meta_wr_dirty_sa_set = sa_resp_decision && io.fromSA.PASSDIRTY
  // --------------------------------

  val meta_wr_state_UU = meta_wr_state_readunique_UU ||
                         meta_wr_state_readshared_UU ||
                         meta_wr_state_readonce_UU ||
                         meta_wr_state_makeunique_UU ||
                         meta_wr_state_stashunique_UU ||
                         meta_wr_state_stashshared_UU

  val meta_wr_state_US = meta_wr_state_readshared_US ||
                         meta_wr_state_readonce_US ||
                         meta_wr_state_stashunique_US ||
                         meta_wr_state_stashshared_US

  val meta_wr_state_S = meta_wr_state_readshared_S ||
                        meta_wr_state_readonce_S ||
                        meta_wr_state_stashshared_S

  val meta_wr_state_I = meta_wr_state_evictback_I

  val meta_wr_state = meta_wr_state_UU || meta_wr_state_US || meta_wr_state_S || meta_wr_state_I

  val meta_wr_dirty_set = meta_wr_dirty_sa_set ||
                          meta_wr_dirty_readunique_set ||
                          meta_wr_dirty_readonce_set ||
                          meta_wr_dirty_readshared_set ||
                          meta_wr_dirty_makeunique_set ||
                          meta_wr_dirty_stashunique_set ||
                          meta_wr_dirty_stashshared_set

  val meta_wr_dirty_clr = meta_wr_dirty_evictback_clr

  val meta_wr_dirty = meta_wr_dirty_set || meta_wr_dirty_clr

  val meta_wr_client_set = meta_wr_client_readunique_set ||
                           meta_wr_client_readshared_set ||
                           meta_wr_client_makeunique_set

  val meta_wr_client_clr = false.B

  val meta_wr_client = meta_wr_client_set || meta_wr_client_clr

  val meta_wr_alias = meta_wr_alias_readunique ||
                      meta_wr_alias_readshared ||
                      meta_wr_alias_makeunique

  // --------------------------------
  val prefill_meta_wr_state = p_prefill_any && (meta_wr_state_UU ||
                                            meta_wr_state_US ||
                                            meta_wr_state_S || 
                                            meta_wr_state_I)

  val prefill_meta_wr_dirty = p_prefill_any && (meta_wr_dirty_set ||
                                            meta_wr_dirty_clr)
  
  val prefill_meta_wr_client = p_prefill_any && (meta_wr_client_set ||
                                             meta_wr_client_clr)

  val prefill_meta_wr_alias = p_prefill_any && meta_wr_alias

  when (prefill_meta_wr_state) {
    p_prefill_miss_meta.state := ParallelMux(Seq(
      (meta_wr_state_UU, MetaState.UU),
      (meta_wr_state_US, MetaState.US),
      (meta_wr_state_S , MetaState.S ),
      (meta_wr_state_I , MetaState.I )
    ))
  }

  when (prefill_meta_wr_dirty) {
    p_prefill_miss_meta.dirty := meta_wr_dirty_set
  }

  when (prefill_meta_wr_client) {
    p_prefill_miss_meta.clients.zip(p_rxreq_client).foreach { case (meta, client) =>
      meta := client && meta_wr_client_set
    }
  }

  when (prefill_meta_wr_alias) {
    p_prefill_miss_meta.alias := p_rxreq.TagAlias
  }

  // --------------------------------
  val repl_tshr_meta_wr = p_prefill_any && io.repl_done
  val tshr_meta_wr_state_UU = meta_wr_state_UU && !p_prefill_miss
  val tshr_meta_wr_state_US = meta_wr_state_US && !p_prefill_miss
  val tshr_meta_wr_state_S = meta_wr_state_S && !p_prefill_miss
  val tshr_meta_wr_state_I = meta_wr_state_I && !p_prefill_miss

  // Miss-prefill defers local allocation until ReplRd completes. Conflict-hit
  // prefill applies the allocating response immediately so later snoops see
  // the actual local owner, then commits the latest local state at repl_done.
  val tshr_meta_wr_state = !p_prefill_miss && (meta_wr_state_UU ||
                                          meta_wr_state_US ||
                                          meta_wr_state_S ||
                                          meta_wr_state_I) ||
                           repl_tshr_meta_wr

  val tshr_meta_wr_dirty = !p_prefill_miss && (meta_wr_dirty_set ||
                                          meta_wr_dirty_clr) ||
                           repl_tshr_meta_wr

  val tshr_meta_wr_client = (!p_prefill_miss && (meta_wr_client_set ||
                                            meta_wr_client_clr) ||
                             repl_tshr_meta_wr) && !p_rxreq_readonce

  val tshr_meta_wr_alias = (!p_prefill_miss && meta_wr_alias ||
                            repl_tshr_meta_wr) && !p_rxreq_readonce

  val tshr_meta_wr_prefetch = repl_tshr_meta_wr ||
                              (!p_prefill_miss && tshr_meta_wr_state && (
                                p_rxreq_readshared || p_rxreq_readunique || p_rxreq_makeunique ||
                                (p_prefill_hit && (p_rxreq_stashshared || p_rxreq_stashunique))
                              ))

  val tshr_tag_wr = repl_tshr_meta_wr

  io.tshr_tag_write_en := tshr_tag_wr

  io.tshr_meta_write_en.state := tshr_meta_wr_state
  io.tshr_meta_write_meta.state := Mux(
    p_prefill_miss,
    p_prefill_miss_meta.state,
    Mux(
      p_prefill_hit && io.repl_done,
      dirResult.state,
      ParallelMux(Seq(
        (meta_wr_state_UU, MetaState.UU),
        (meta_wr_state_US, MetaState.US),
        (meta_wr_state_S , MetaState.S ),
        (meta_wr_state_I , MetaState.I )
      ))
    )
  )

  io.tshr_meta_write_en.dirty := tshr_meta_wr_dirty
  io.tshr_meta_write_meta.dirty := Mux(
    p_prefill_miss,
    p_prefill_miss_meta.dirty,
    Mux(p_prefill_hit && io.repl_done, dirResult.dirty, meta_wr_dirty_set)
  )

  io.tshr_meta_write_en.clients.zip(p_rxreq_client).foreach { case (en, client) => 
    en := client && tshr_meta_wr_client 
  }
  io.tshr_meta_write_meta.clients.zipWithIndex.foreach { case (meta, i) =>
    meta := Mux(
      p_prefill_miss,
      p_prefill_miss_meta.clients(i),
      Mux(
        p_prefill_hit && io.repl_done,
        dirResult.clients(i),
        p_rxreq_client(i) && meta_wr_client_set
      )
    )
  }

  io.tshr_meta_write_en.alias := tshr_meta_wr_alias
  io.tshr_meta_write_meta.alias := Mux(
    p_prefill_miss,
    p_prefill_miss_meta.alias,
    Mux(p_prefill_hit && io.repl_done, dirResult.alias, p_rxreq.TagAlias)
  )
  io.tshr_meta_write_en.prefetch := tshr_meta_wr_prefetch
  io.tshr_meta_write_meta.prefetch := Mux(
    p_prefill_miss || (p_prefill_hit && !io.repl_done),
    p_prefill_miss_meta.prefetch,
    Mux(p_prefill_hit && io.repl_done, dirResult.prefetch, false.B)
  )
  // ----------------------------------------------------------------

  // -- Interactions with TSHR local data and Data Storage Read
  val ds_rd_readunique_sat = rxreq_satisfied_readunique &&
                             (rxreq.ExpCompData || !rxreq_client_present)

  val ds_rd_readunique_unsat_lazy = p_rxreq_readunique &&
                                    (p_rxreq.ExpCompData || !p_rxreq_client_present) &&
                                    dn_rxrsp_comp

  val ds_rd_readunique_unsat_early = rxreq_unsatisfied_readunique &&
                                     dirResult.hit &&
                                     (rxreq.ExpCompData || !rxreq_client_present)

  val ds_rd_readunique_unsat = Mux(configReadUniqueHitSPreferReadDS,
                                 ds_rd_readunique_unsat_early,
                                 ds_rd_readunique_unsat_lazy)

  val ds_rd_readunique = ds_rd_readunique_sat || ds_rd_readunique_unsat

  val ds_rd_readshared = rxreq_readshared && dirResult.hit
  val ds_rd_readonce = rxreq_readonce && dirResult.hit

  // *NOTE: If SA received SnpRespData from upstream, the DS Read would be dropped by Data Storage Proxy.
  val ds_rd_evictback = p_rxreq_evictback &&
                        (fire_txreq_writeevictfull ||
                         fire_txreq_writeevictorevict ||
                         fire_txreq_writebackfull)

  // A satisfied conflict-hit EVB still needs the surrendered line for nested
  // snoops. Preserve it before allowing the peer refiller to commit the way.
  val prefill_hit_data_ready = io.tbuf_data0_valid && io.tbuf_data2_valid
  val rxevb_prefill_hit = rxevb_evictback && p_prefill_hit
  val rxevb_prefill_hit_need_data = rxevb_prefill_hit && !prefill_hit_data_ready

  val ds_rd = !io.tbuf_modified && (ds_rd_readunique ||
                                    ds_rd_readshared ||
                                    ds_rd_readonce ||
                                    ds_rd_evictback ||
                                    rxevb_prefill_hit_need_data)

  val ds_cancel_readunique = p_rxreq_readunique &&
                             (sa_respdata_first ||
                              dn_rxdat_compdata_first || dn_rxrsp_respsepdata)

  val ds_cancel_readshared = p_rxreq_readshared &&
                             sa_respdata_first

  val ds_cancel_readonce = p_rxreq_readonce &&
                           sa_respdata_first

  val ds_cancel_evictback = p_rxreq_evictback &&
                            dn_rxrsp_comp

  val ds_cancel = ds_cancel_readunique ||
                  ds_cancel_readshared ||
                  ds_cancel_readonce ||
                  ds_cancel_evictback

  when (ds_rd) {
    w_ds_resp := true.B
  }

  when (ds_cancel) {
    w_ds_resp := false.B
  }

  when (io.ds_rd_done) {
    w_ds_resp := false.B
  }

  io.ds_rd_en := ds_rd
  io.ds_rd_cancel := w_ds_resp && ds_cancel
  // ----------------------------------------------------------------

  // -- Interactions with upstream CCHI TXRSP channel
  val up_txrsp_comp = io.UpTXRSP.fire && io.UpTXRSP.bits.Opcode === CCHIOpcode.Comp.U

  val allow_up_rd_comp = sa_resp_decided && 
                         dn_rd_decided && 
                         !w_rd_dn_comp

  val sched_up_rd_comp_makeunique_sat = rxreq_satisfied_makeunique

  val sched_up_rd_comp_makeunique_unsat = p_rxreq_makeunique &&
                                          dn_rxrsp_comp

  val sched_up_rd_comp_makeunique = sched_up_rd_comp_makeunique_sat ||
                                    sched_up_rd_comp_makeunique_unsat

  val sched_up_rd_comp_readunique_sat = rxreq_satisfied_readunique &&
                                        rxreq_client_present && !rxreq.ExpCompData

  val sched_up_rd_comp_readunique_unsat = p_rxreq_readunique &&
                                          p_rxreq_client_present && !p_rxreq.ExpCompData &&
                                          (dn_rxrsp_comp || dn_rxdat_compdata_first || dn_rxdat_datasepresp_first)

  val sched_up_rd_comp_readunique = sched_up_rd_comp_readunique_sat ||
                                    sched_up_rd_comp_readunique_unsat

  val sched_up_rd_comp = sched_up_rd_comp_readunique ||
                         sched_up_rd_comp_makeunique

  when (sched_up_rd_comp) {
    w_s_rd_up_comp := true.B
  }

  issue_up_rd_comp := w_s_rd_up_comp && allow_up_rd_comp

  when (issue_up_rd_comp) {
    w_s_rd_up_comp := false.B
    s_rd_up_comp := true.B
  }

  when (up_txrsp_comp) {
    s_rd_up_comp := false.B
  }

  val up_txrsp_meta_state = Mux(p_prefill_any, p_prefill_miss_meta.state, dirResult.state)

  val up_txrsp_opcode = ParallelPriorityMux(Seq(
    (s_rd_up_comp, CCHIOpcode.Comp.U)
  ))

  val up_txrsp_resp = ParallelPriorityMux(Seq(
    (s_rd_up_comp, ParallelPriorityMux(Seq(
      (p_rxreq_readunique, CCHIResp.UC.U),
      (p_rxreq_readshared, Mux(up_txrsp_meta_state === MetaState.UU || meta_wr_state_UU, CCHIResp.UC.U, CCHIResp.SC.U)),
      (p_rxreq_makeunique, CCHIResp.UC.U)
    )))
  ))

  io.UpTXRSP.valid := s_rd_up_comp
  io.UpTXRSP.bits.TxnID := p_rxreq.TxnID
  io.UpTXRSP.bits.SrcID := io.consts.sliceNID
  io.UpTXRSP.bits.TgtID := p_rxreq.SrcID
  io.UpTXRSP.bits.DBID := io.consts.tshrId
  io.UpTXRSP.bits.Opcode := up_txrsp_opcode
  io.UpTXRSP.bits.RespErr := 0.U // TODO: RespErr
  io.UpTXRSP.bits.Resp := up_txrsp_resp
  io.UpTXRSP.bits.CBusy := 0.U // TODO: CBusy, may be assigned in TSHR top
  io.UpTXRSP.bits.WayValid := s_rd_up_comp
  io.UpTXRSP.bits.Way := dirResult.way
  io.UpTXRSP.bits.TraceTag := false.B // TODO: TraceTag propagation
  // ----------------------------------------------------------------

  // -- Interactions with upstream CCHI TXDAT channel
  val up_txdat_compdata0 = io.UpTXDAT.fire && io.UpTXDAT.bits.Opcode === CCHIOpcode.CompData.U && io.UpTXDAT.bits.DataID === 0.U
  val up_txdat_compdata2 = io.UpTXDAT.fire && io.UpTXDAT.bits.Opcode === CCHIOpcode.CompData.U && io.UpTXDAT.bits.DataID === 1.U

  val up_txdat_compdata_last = (up_txdat_compdata0 && !s_rd_up_compdata2) ||
                               (up_txdat_compdata2 && !s_rd_up_compdata0)

  val ds_wb_cancel_readonce_bypass = p_rxreq_readonce &&
                                     !configInclusiveReadOnce.B &&
                                     dirResult.state === MetaState.I &&
                                     up_txdat_compdata_last

  val allow_up_rd_compdata0 = (if (paramL2.hitFastPathDsRespIssue) {
    !w_ds_resp || io.ds_rd_done
  } else {
    !w_ds_resp
  }) && !w_snpresp0 && !w_rd_dn_data0 && !io.L1EVT_active && io.tbuf_data0_valid
  val allow_up_rd_compdata2 = (if (paramL2.hitFastPathDsRespIssue) {
    !w_ds_resp || io.ds_rd_done
  } else {
    !w_ds_resp
  }) && !w_snpresp2 && !w_rd_dn_data2 && !io.L1EVT_active && io.tbuf_data2_valid

  val sched_up_rd_compdata_sat_readunique = rxreq_satisfied_readunique && 
                                            (!rxreq_client_present || rxreq.ExpCompData)
                                
  val sched_up_rd_compdata_sat_readshared = rxreq_satisfied_readshared
  val sched_up_rd_compdata_sat_readonce = rxreq_satisfied_readonce

  val sched_up_rd_compdata_sat = sched_up_rd_compdata_sat_readunique ||
                                 sched_up_rd_compdata_sat_readshared ||
                                 sched_up_rd_compdata_sat_readonce

  val sched_up_rd_compdata_unsat_readunique = (rxreq_unsatisfied_readunique &&
                                               rxreq.ExpCompData) ||
                                              (p_rxreq_readunique &&
                                               (!p_rxreq_client_present && !p_rxreq.ExpCompData) &&
                                               (dn_rxrsp_comp || dn_rxdat_compdata_first || dn_rxdat_datasepresp_first))

  val sched_up_rd_compdata_unsat_readshared = rxreq_unsatisfied_readshared
  val sched_up_rd_compdata_unsat_readonce = rxreq_unsatisfied_readonce

  val sched_up_rd_compdata_unsat = sched_up_rd_compdata_unsat_readunique ||
                                   sched_up_rd_compdata_unsat_readshared ||
                                   sched_up_rd_compdata_unsat_readonce

  val sched_up_rd_compdata = sched_up_rd_compdata_sat ||
                             sched_up_rd_compdata_unsat

  when (sched_up_rd_compdata) {
    w_s_rd_up_compdata0 := true.B
    w_s_rd_up_compdata2 := true.B
  }

  val issue_up_rd_compdata0 = w_s_rd_up_compdata0 && allow_up_rd_compdata0
  val issue_up_rd_compdata2 = w_s_rd_up_compdata2 && allow_up_rd_compdata2

  issue_up_rd_compdata_first := (issue_up_rd_compdata0 || issue_up_rd_compdata2) &&
                                (w_s_rd_up_compdata0 && w_s_rd_up_compdata2)

  when (issue_up_rd_compdata0) {
    w_s_rd_up_compdata0 := false.B
    s_rd_up_compdata0 := true.B
  }

  when (issue_up_rd_compdata2) {
    w_s_rd_up_compdata2 := false.B
    s_rd_up_compdata2 := true.B
  }

  when (up_txdat_compdata0) {
    s_rd_up_compdata0 := false.B
  }

  when (up_txdat_compdata2) {
    s_rd_up_compdata2 := false.B
  }

  val up_txdat_meta_state = Mux(p_prefill_any, p_prefill_miss_meta.state, dirResult.state)

  val up_txdat_resp = ParallelPriorityMux(Seq(
    (p_rxreq_readunique, CCHIResp.UC.U),
    (p_rxreq_readshared, Mux(up_txdat_meta_state === MetaState.UU || meta_wr_state_UU, CCHIResp.UC.U, CCHIResp.SC.U)),
    (p_rxreq_readonce, CCHIResp.UC.U)
  ))

  val up_txdat_dataid = Mux(s_rd_up_compdata0, 0.U, 1.U) // TODO: cirtical word first maybe

  io.UpTXDAT.valid := s_rd_up_compdata0 || s_rd_up_compdata2
  io.UpTXDAT.bits.TxnID := p_rxreq.TxnID
  io.UpTXDAT.bits.SrcID := io.consts.sliceNID
  io.UpTXDAT.bits.TgtID := p_rxreq.SrcID
  io.UpTXDAT.bits.DBID := io.consts.tshrId
  io.UpTXDAT.bits.Opcode := CCHIOpcode.CompData.U
  io.UpTXDAT.bits.RespErr := 0.U // TODO: RespErr
  io.UpTXDAT.bits.Resp := up_txdat_resp
  io.UpTXDAT.bits.DataSource := 0.U // TODO: DataSource
  io.UpTXDAT.bits.CBusy := 0.U //  TODO: CBusy, may be assigned in TSHR top
  io.UpTXDAT.bits.WayValid := !p_rxreq_readonce
  io.UpTXDAT.bits.Way := dirResult.way
  io.UpTXDAT.bits.DataID := up_txdat_dataid
  io.UpTXDAT.bits.TraceTag := false.B // TODO: TraceTag propagation
  io.UpTXDAT.bits.Data := DontCare
  // *NOTICE: Data is assigned in TSHR top
  // ----------------------------------------------------------------

  // -- Interactions with replacer and eviction through loop-back REQ
  val txreq_evictback_peer = io.UpTXEVB.fire

  val expect_replace_readonce = if (configInclusiveReadOnce) rxreq_readonce else false.B

  val expect_miss_replace = !dirResult.hit && (
                              rxreq_readunique ||
                              rxreq_readshared ||
                              expect_replace_readonce ||
                              rxreq_makeunique ||
                              rxreq_stashshared ||
                              rxreq_stashunique
                            )

  val expect_replace = rxreq_hit_lock_conflict || expect_miss_replace

  // The peer refiller owns the conflicted old way. Conflict-hit prefill must
  // not require an old-line DirWb before starting its own ReplRd.
  val allow_replace = w_s_repl &&
                      (p_prefill_hit || !io.tshr_meta_modified || io.dir_wb_done) &&
                      ((dn_rxrsp_comp || dn_rxdat_compdata_first || dn_rxdat_datasepresp_first) ||
                      (!w_rd_dn_comp && (!w_rd_dn_data0 || !w_rd_dn_data2)))

  when (expect_replace) {
    w_s_repl := true.B
    w_s_evict_up_evict := true.B
  }

  when (allow_replace) {
    w_s_repl := false.B
    s_repl := true.B
  }

  when (rxreq_fire) {
    p_prefill_miss := expect_miss_replace
    p_prefill_hit := rxreq_hit_lock_conflict
    p_prefill_miss_meta := 0.U.asTypeOf(p_prefill_miss_meta)
    p_prefill_miss_meta.prefetch := expect_replace && (rxreq_stashshared || rxreq_stashunique)
  }

  // If the replacer picked an INVALID way whose stale tag aliases this very request (the line
  // was filled earlier, then invalidated leaving its tag behind), the looped-back EvictBack
  // would paddr-hit this originator entry itself: alloc is blocked (!paddr_hit_any), the
  // EvictBack is forced onto the reuse path of THIS busy vPipe, never enters, and the dir/ds
  // unlocks never fire -- a circular wait inside one entry. The aliasing itself proves the
  // victim way is invalid (a valid way holding this tag would have hit the directory), so no
  // eviction is needed: skip the upstream EvictBack and unlock dir/ds locally.
  // Also same for a victim that reads Invalid at ReplRd: nothing exists downstream to evict,
  // so the looped-back EvictBack is pure overhead -- and it can deadlock: the EvictBack must
  // nest into the TSHR holding the victim PA, which may itself be parked on an unbounded
  // downstream wait (s7118: the victim-PA TSHR's hit-MU starved at L3 while this refill's
  // blockRefill lock stayed armed, zeroing the set's victimMask). The Directory guarantees an
  // invalid locked entry stays invalid until this refill's own commit (invalid entries cannot
  // be hit-modified; the lock keeps other refills off the way), and an Invalid entry never
  // carries client presence (guarded by a Directory read-response assertion), so the pick-time
  // read cannot go stale in the unsafe direction.
  val evict_self_alias = io.repl_resp.paddr === io.tshr_paddr
  val evict_victim_invalid = io.repl_meta.state === MetaState.I
  val evict_not_needed = evict_self_alias || evict_victim_invalid

  when (io.repl_done) {
    p_prefill_miss := false.B
    p_prefill_hit := false.B
    s_repl := false.B
    when (w_s_evict_up_evict) {
      w_s_evict_up_evict := false.B
      s_evict_up_evict := !evict_not_needed
    }
  }

  when (txreq_evictback_peer) {
    s_evict_up_evict := false.B
  }

  val lock_on_miss = p_prefill_miss && io.repl_done
  val lock_on_hit = p_prefill_hit && !io.repl_done
  val lock_dir = lock_on_miss || lock_on_hit || rxevb_unsatisfied_evictback
  val lock_ds = lock_on_miss || lock_on_hit || rxevb_unsatisfied_evictback

  val unlock_self_evictback = RegNext(meta_wr_state_evictback_I) && !meta_wr_state_evictback_I
  val unlock_self_noevict = RegNext(io.repl_done && w_s_evict_up_evict && evict_not_needed)
  val unlock_evictback_cancel = fire_txreq_pcrdreturn && p_rxevb_unsatisfied

  val unlock_dir = io.self_unlock_dir || unlock_self_evictback || unlock_self_noevict ||
                   unlock_evictback_cancel
  val unlock_ds = io.self_unlock_ds || unlock_self_evictback || unlock_self_noevict ||
                  unlock_evictback_cancel

  when (lock_dir) {
    w_unlock_dir := true.B
  }

  when (lock_ds) {
    w_unlock_ds := true.B
  }

  when (unlock_dir) {
    w_unlock_dir := false.B
  }

  when (unlock_ds) {
    w_unlock_ds := false.B
  }

  io.repl_en := s_repl
  io.repl_reset := rxreq_fire

  io.repl_lock_req := rxreq_fire && rxreq_hit_lock_accept
  io.repl_lock_way := Mux(rxreq_fire, dirResult.way, p_hit_lock_way)

  io.repl_lock_release := s_hit_lock_release

  when (rxreq_fire) {
    when (rxreq_hit_lock_accept) {
      p_hit_lock_way := dirResult.way
    }
  }

  when (rxreq_fire) {
    w_hit_lock_release := rxreq_hit_lock_accept
    s_hit_lock_release := false.B
  }

  assert(!(p_prefill_hit && (w_hit_lock_release || s_hit_lock_release)),
    "REQ vPipe conflict-hit prefill unexpectedly owns a hit-lock")

  // Delay release until the final local tag/meta writes have registered. The
  // pending-until-accept flags must not hold the write or it could never proceed.
  val hit_lock_write_hold = w_hit_lock_release || rxreq_hit_lock_accept ||
                            (s_hit_lock_release &&
                             (io.tshr_meta_write_en.asUInt.orR ||
                              io.tshr_tag_write_en))

  io.dir_wb_locked := w_unlock_dir || hit_lock_write_hold
  io.ds_wb_locked := w_unlock_ds

  // 1. Clean local Meta (to Directory) and TSHR Buffer (to Data Storage) modified state and 
  //    cancel all non-arbitered Directory & Data Storage write back for L2 Eviction
  // 2. Cancel same way unfinished meta commiting on satisfied EvictBack since we're unlocking
  //    peer Directory commit immediately
  // 3. Cancel at snoop-response completion of an unsatisfied EvictBack.
  //    This drops the Snoop Agent's clients-clear the same cycle it rises
  //    (L2SnoopAgent.scala clear_client_en) and the pending first-beat PASSDIRTY dirty-set.
  // 4. Cancel a retry-canceled unsatisfied EvictBack's pending directory write so it cannot
  //    invalidate the replacement already committed to the same way.
  io.dir_wb_cancel := meta_wr_state_evictback_I ||
                      unlock_evictback_cancel ||
                      rxevb_satisfied_evictback && dirResult.way === rxevb.Way ||
                      p_rxreq_evictback && sa_resp_last

  // A Bypass ReadOnce miss leaves meta I and allocates no DS line. Drop its transient
  // buffer only after the final upstream beat has been accepted, so deallocation
  // cannot strand modified state or leak it into the next TSHR life.
  io.ds_wb_cancel := w_evict_dn_comp || w_evict_dn_compdbid ||
                     s_evict_dn_cbwrdata0 || s_evict_dn_cbwrdata2 || s_evict_dn_compack ||
                     ds_wb_cancel_readonce_bypass

  // 1. Activate Directory write-back immediately on replacement Directory lock released by eviction
  //    to clear the replacer reading lock in Directory.
  //    NOTE: 'unlock_self_evictback' is intentionally excluded here — its pulse is the falling edge
  //    of 'meta_wr_state_evictback_I', which is itself a 'dir_wb_cancel' source, so the EVB host's
  //    pending invalidate is cancelled and there is never committed work left for the aux to trigger;
  //    re-arming on it manufactured a phantom (empty) DirWb that collided with a nested refill's
  //    ReplRd (seed 3019). 'unlock_self_noevict' is kept: its pulse always has the refill's locked
  //    commit pending, and it is the only prompt trigger once 'expect_replace' has cleared.
  // 2. Trigger Directory write-back immediately on refill transactions to commit the un-committed
  //    I state into Directory on reuse.
  // 3. Trigger Directory write-back immediately on EvictBack to commit the un-commited meta.
  // 4. Commit modified meta before an ordinary miss replacement read can wait
  //    on it. Conflict-hit prefill must not write the surrendered old line.
  val dir_meta_stale = (io.tshr_meta_modified || io.tshr_tag_modified) && !io.dir_wb_accept
  io.dir_wb_aux := hit_lock_response ||
                   io.self_unlock_dir || unlock_self_noevict ||
                   (dir_meta_stale && expect_miss_replace) ||
                   (dir_meta_stale && rxevb_satisfied_evictback)

  assert(!(unlock_self_evictback && (io.tshr_meta_modified || io.tshr_tag_modified)),
    "EvictBack self-unlock with un-committed meta/tag: aux drop would strand a commit")

  // 1. Activate Data Storage write-back immediately on replacement Data Storage lock released by eviction
  //    since the TSHR local meta state was not updated till replacer response, and the data always return
  //    before the replace response. The stale state could never trigger a pending Data Storage write-back
  //    to the replacing way and hence, this compensational re-triggering matters.
  io.ds_wb_aux := unlock_ds
  
  io.UpTXEVB.valid := s_evict_up_evict
  io.UpTXEVB.bits.TshrId := io.consts.tshrId
  io.UpTXEVB.bits.Addr := io.repl_resp.paddr
  io.UpTXEVB.bits.Way := io.repl_resp.way
  // ----------------------------------------------------------------

  // -- Interactions with peer Refill unlock
  // For an unsatisfied EvictBack, the peer refiller's Directory commit is what makes the victim
  // slot re-selectable by unrelated refills. Release it only after this EvictBack's Data Storage
  // read has landed -- otherwise a third-party refill can re-select the slot and overwrite the
  // victim data before this EvictBack has read it. (Previously released immediately.)
  // A satisfied conflict-hit EVB follows the same ordering when its buffer is incomplete:
  // preserve the surrendered line before either peer lock can release a competing refill.
  val evictback_peer_unlock_dir_immediate = rxevb_unsatisfied_evictback && io.ds_rd_done ||
                                            (rxevb_satisfied_evictback &&
                                             !rxevb_prefill_hit_need_data &&
                                             (p_prefill_any ||
                                              dirResult.way =/= rxevb.Way ||
                                              (!io.tshr_meta_modified && !io.tshr_tag_modified)))

  val sched_evictback_peer_unlock_dir = rxevb_evictback &&
                                        !evictback_peer_unlock_dir_immediate

  val allow_evictback_peer_unlock_dir = w_s_evict_peer_unlock_dir &&
                                        Mux(p_rxevb_unsatisfied,
                                            io.ds_rd_done || ds_cancel_evictback || fire_txreq_evict ||
                                            unlock_evictback_cancel,
                                            Mux(p_prefill_hit,
                                                prefill_hit_data_ready,
                                                io.dir_wb_done))

  val evictback_peer_unlock_dir_late = allow_evictback_peer_unlock_dir

  val evictback_peer_unlock_dir = evictback_peer_unlock_dir_immediate ||
                                  evictback_peer_unlock_dir_late

  when (sched_evictback_peer_unlock_dir) {
    w_s_evict_peer_unlock_dir := true.B
  }

  when (allow_evictback_peer_unlock_dir) {
    w_s_evict_peer_unlock_dir := false.B
  }

  val evictback_peer_unlock_ds_immediate = rxevb_unsatisfied_evictback && io.ds_rd_done ||
                                           (rxevb_satisfied_evictback &&
                                            !rxevb_prefill_hit_need_data)

  val sched_evictback_peer_unlock_ds = rxevb_evictback &&
                                       !evictback_peer_unlock_ds_immediate

  val allow_evictback_peer_unlock_ds = w_s_evict_peer_unlock_ds &&
                                       Mux(p_prefill_hit,
                                           prefill_hit_data_ready,
                                           io.ds_rd_done || ds_cancel_evictback || fire_txreq_evict ||
                                           unlock_evictback_cancel)

  val evictback_peer_unlock_ds_late = allow_evictback_peer_unlock_ds

  val evictback_peer_unlock_ds = evictback_peer_unlock_ds_immediate ||
                                 evictback_peer_unlock_ds_late

  when (sched_evictback_peer_unlock_ds) {
    w_s_evict_peer_unlock_ds := true.B
  }

  when (allow_evictback_peer_unlock_ds) {
    w_s_evict_peer_unlock_ds := false.B
  }

  io.peer_unlock_dir.zipWithIndex.foreach { case (unlock_dir, i) => {
    unlock_dir := evictback_peer_unlock_dir_immediate && rxevb.TshrId === i.U ||
                  evictback_peer_unlock_dir_late && p_rxevb.TshrId === i.U
  }}

  io.peer_unlock_ds.zipWithIndex.foreach { case (unlock_ds, i) => {
    unlock_ds := evictback_peer_unlock_ds_immediate && rxevb.TshrId === i.U ||
                 evictback_peer_unlock_ds_late && p_rxevb.TshrId === i.U
  }}

  assert(!(rxevb_prefill_hit_need_data &&
           (evictback_peer_unlock_dir_immediate ||
            evictback_peer_unlock_ds_immediate)),
    "TSHR @ %m conflict-hit EVB released peer locks before preserving data")

  assert(!((evictback_peer_unlock_dir_late ||
            evictback_peer_unlock_ds_late) &&
           p_prefill_hit && !prefill_hit_data_ready),
    "TSHR @ %m conflict-hit EVB unlocked peer with incomplete preserved data")

  if (configNonAgedDirArb) {
    // -- EVB Directory commit acknowledgement (non-aged Directory arbitration)
    // Host side: keep the EVB active until the refilling peer's Directory commit lands,
    // so no same-PA request can observe the victim entry between its (possibly cancelled)
    // invalidation and the refiller's replacement commit.
    // Self-alias EVBs skip the wait: the same TSHR's proxies already serialize the commit.
    when ((evictback_peer_unlock_dir_immediate && rxevb.TshrId =/= io.consts.tshrId) ||
          (evictback_peer_unlock_dir_late && p_rxevb.TshrId =/= io.consts.tshrId)) {
      w_evict_peer_commit_dir := true.B
    }
    when (io.self_unlock_dir_ack) {
      w_evict_peer_commit_dir := false.B
    }
    assert(!(io.self_unlock_dir_ack && !w_evict_peer_commit_dir),
      "TSHR @ %m REQ vPipe received Directory commit ack while no EVB was waiting for it")

    // Refiller side: a peer EVB host unlocked our refill commit; acknowledge once the
    // tag+meta commit lands in the Directory.
    when (io.self_unlock_dir && w_unlock_dir) {
      p_unlock_source := io.self_unlock_dir_tshrId
      w_s_unlock_ack := true.B
    }
    when (io.dir_wb_done) {
      w_s_unlock_ack := false.B
    }
    io.peer_unlock_ack := w_s_unlock_ack && io.dir_wb_done
    io.peer_unlock_ack_tshrId := p_unlock_source
  } else {
    io.peer_unlock_ack := false.B
    io.peer_unlock_ack_tshrId := 0.U
  }
  // ----------------------------------------------------------------

  // -- L2 Eviction (EvictBack) active
  val evict_active = w_evict_s_dn_txreq ||
                     w_evict_dn_comp || w_evict_dn_compdbid ||
                     w_s_evict_dn_cbwrdata0 || w_s_evict_dn_cbwrdata2 ||
                     s_evict_dn_cbwrdata0 || s_evict_dn_cbwrdata2 ||
                     s_evict_dn_compack

  io.L2EVT_opcode.valid := evict_active
  io.L2EVT_opcode.bits := evictback_txreq_opcode
  // ----------------------------------------------------------------

  // -- Blocking same-PA RXSNP, on waiting of L1 CompAck
  io.blockRBE.EVT := p_prefill_miss
  io.blockRBE.SNP := w_rd_up_compack || s_repl
  io.blockRBE.EVB := (active && !p_prefill_any) ||
                     w_s_evict_peer_unlock_dir || w_s_evict_peer_unlock_ds ||
                     w_evict_peer_commit_dir ||
                     evict_active
  io.blockRBE.REQ := active

  assert(!(rxevb_unsatisfied_evictback && active), 
    "TSHR @ %m REQ vPipe passed unsatisfied EvictBack on active non-replace non-invalid transaction")
  // ----------------------------------------------------------------
}
