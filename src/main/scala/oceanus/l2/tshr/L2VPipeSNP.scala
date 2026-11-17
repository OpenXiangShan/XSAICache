package oceanus.l2.tshr

import chisel3._
import chisel3.util._
import oceanus.l2._
import oceanus.compactchi.CCHIComponent
import oceanus.chi.CHICohResps
import oceanus.chi.bundle._
import oceanus.chi.opcode._
import org.chipsalliance.cde.config.Parameters

class L2VPipeSNP(clientComponents: Seq[CCHIComponent], val sliceNum: Int)(implicit val p: Parameters)
    extends Module
    with CHIRNFOpcodesSNP
    with CHIRNFOpcodesRSP
    with CHIRNFOpcodesDAT
    with CHIRNFOpcodesREQ
    with HasL2Params {

  val io = IO(new Bundle {
    val consts = Input(new L2TSHRConsts(sliceNum))

    val DnRXSNP = Flipped(Valid(new CHIBundleSNP))

    val DnTXRSP = Decoupled(new CHIBundleRSP)
    val DnTXDAT = Decoupled(new CHIBundleDAT)

    val tshr_paddr = Input(UInt(paramL2.physicalAddrWidth.W))
    val tshr_dirResult = Input(new L2Directory.MetaReadResult)

    val tshr_meta_write_en = Output(new L2Directory.MetaWriteMask)
    val tshr_meta_write_meta = Output(new L2Directory.Meta)

    val toSA = Output(new L2SnoopAgent.PathToSnoopAgent)
    val fromSA = Input(new L2SnoopAgent.PathFromSnoopAgent)

    val ds_read_en = Output(Bool())
    val ds_read_done = Input(Bool())

    val ds_wb_cancel = Output(Bool())

    val tbuf_modified = Input(Bool()) // TSHR buffer holds data written by a non-DS source (RXDAT)
    val tbuf_half0_ready = Input(Bool()) // TSHR buffer half (DataID 0) holds valid data
    val tbuf_half2_ready = Input(Bool()) // TSHR buffer half (DataID 2) holds valid data

    val blockRBE = Output(new L2RBE.PathVPipeBlock)
    val free = Output(Bool())
  
    val EVT_active = Input(Bool())
    val REQ_evict = Flipped(Valid(UInt(paramCHI.reqOpcodeWidth.W)))
  })

  // -- FSM State ------------------------------------------
  val s_meta_write = RegInit(false.B) // Scheduling meta write
  val s_home_resp0 = RegInit(false.B) // Scheduling downstream TXDAT SnpRespData (DataID = 0)
  val s_home_resp2 = RegInit(false.B) // Scheduling downstream TXDAT SnpRespData (DataID = 2)
  val s_dct_resp0 = RegInit(false.B) // Scheduling downstream TXDAT SnpRespData (DataID = 0)
  val s_dct_resp2 = RegInit(false.B) // Scheduling downstream TXDAT SnpRespData (DataID = 2)
  val s_snpcompack = RegInit(false.B) // Scheduling SnpCompAck to Snoop Agent

  val w_snpresp0 = RegInit(false.B) // Waiting for response from Snoop Agent (DataID = 0)
  val w_snpresp2 = RegInit(false.B) // Waiting for response from Snoop Agent (DataID = 2)
  val w_ds_read = RegInit(false.B) // Waiting for Data Storage read done
  val active = s_home_resp0 || s_home_resp2 || s_dct_resp0 || s_dct_resp2 ||
               s_meta_write || w_ds_read || w_snpresp0 || w_snpresp2 || s_snpcompack

  val p_rxsnp = RegInit(0.U.asTypeOf(new CHIBundleSNP))
  val p_rxsnp_to_invalid = RegInit(false.B)
  val p_fwd_state = RegInit(0.U(CHICohResps.WIDTH.W))
  val p_txdat_dataID = RegInit(0.U(paramCHI.datDataIDWidth.W))
  val p_home_txrsp_valid = RegInit(false.B)
  val p_home_txdat_valid = RegInit(false.B)
  val p_home_txrsp_resp = RegInit(0.U(CHICohResps.WIDTH.W))
  val p_home_txdat_resp = RegInit(0.U(CHICohResps.WIDTH.W))

  val p_dct_txdat_valid = RegInit(false.B)
  val p_dct_txdat_resp = RegInit(0.U(CHICohResps.WIDTH.W))
  val p_rxsnp_need_dct_txdat = RegInit(false.B)
  val p_sa_passdirty = RegInit(false.B)
  val dirty_or_passdirty = io.tshr_dirResult.dirty || p_sa_passdirty
  // --------------------------------------------------------

  // -- RXSNP Opcode Decoder --------------------------------
  val rxsnp_opcode = Module(new SNPOpcodeDecoder(Seq(
    CHI_SnpShared,
    CHI_SnpClean,
    CHI_SnpOnce,
    CHI_SnpNotSharedDirty,
    CHI_SnpUnique,
    CHI_SnpPreferUnique,
    CHI_SnpCleanShared,
    CHI_SnpCleanInvalid,
    CHI_SnpMakeInvalid,
    CHI_SnpSharedFwd,
    CHI_SnpCleanFwd,
    CHI_SnpOnceFwd,
    CHI_SnpNotSharedDirtyFwd,
    CHI_SnpUniqueFwd,
    CHI_SnpPreferUniqueFwd,
    CHI_SnpUniqueStash,
    CHI_SnpMakeInvalidStash,
    CHI_SnpStashUnique,
    CHI_SnpStashShared,
    CHI_SnpQuery,
    // CHI_SnpDVMOp
  ), paramEnableUnsupportedCheck = true))

  rxsnp_opcode.io.valid := io.DnRXSNP.fire || active
  rxsnp_opcode.io.opcode := Mux(io.DnRXSNP.fire, io.DnRXSNP.bits.Opcode.get, p_rxsnp.Opcode.get)

  val req_evict_valid = io.REQ_evict.valid
  val req_evict_opcode = io.REQ_evict.bits
  val nested_writeBackFull = req_evict_valid && req_evict_opcode === CHI_WriteBackFull.asUInt
  val nested_writeCleanFull = req_evict_valid && req_evict_opcode === CHI_WriteCleanFull.asUInt
  val nested_writeEvictX = req_evict_valid && (req_evict_opcode === CHI_WriteEvictFull.asUInt || 
                           req_evict_opcode === CHI_WriteEvictOrEvict.asUInt)

  // nonFwd & nonStash Snoop
  val rxsnp_snpOnce = rxsnp_opcode.is(CHI_SnpOnce)
  val rxsnp_snpQuery = rxsnp_opcode.is(CHI_SnpQuery)
  val rxsnp_snpMakeInvalid = rxsnp_opcode.is(CHI_SnpMakeInvalid)
  val rxsnp_snpCleanShared = rxsnp_opcode.is(CHI_SnpCleanShared)
  // Fwd Snoop
  val rxsnp_snpOnceFwd = rxsnp_opcode.is(CHI_SnpOnceFwd)
  val rxsnp_snpCleanFwd = rxsnp_opcode.is(CHI_SnpCleanFwd)
  val rxsnp_snpUniqueFwd = rxsnp_opcode.is(CHI_SnpUniqueFwd)
  val rxsnp_snpSharedFwd = rxsnp_opcode.is(CHI_SnpSharedFwd)
  val rxsnp_snpPreferUniqueFwd = rxsnp_opcode.is(CHI_SnpPreferUniqueFwd)
  val rxsnp_snpNotSharedDirtyFwd = rxsnp_opcode.is(CHI_SnpNotSharedDirtyFwd)
  val rxsnp_fwd = rxsnp_snpOnceFwd || rxsnp_snpCleanFwd || rxsnp_snpUniqueFwd || rxsnp_snpSharedFwd || 
                  rxsnp_snpPreferUniqueFwd || rxsnp_snpNotSharedDirtyFwd
  // Stash Snoop
  val rxsnp_snpUniqueStash = rxsnp_opcode.is(CHI_SnpUniqueStash)
  val rxsnp_snpMakeInvalidStash = rxsnp_opcode.is(CHI_SnpMakeInvalidStash)
  val rxsnp_snpStashUnique = rxsnp_opcode.is(CHI_SnpStashUnique)
  val rxsnp_snpStashShared = rxsnp_opcode.is(CHI_SnpStashShared)
  val rxsnp_stash = rxsnp_snpUniqueStash || rxsnp_snpMakeInvalidStash || rxsnp_snpStashUnique || rxsnp_snpStashShared

  val rxsnp_to_shared = rxsnp_opcode.is(
    CHI_SnpClean,
    CHI_SnpNotSharedDirty,
    CHI_SnpShared,
    CHI_SnpSharedFwd,
    CHI_SnpCleanFwd,
    CHI_SnpNotSharedDirtyFwd
  )
  val rxsnp_to_invalid = rxsnp_opcode.is(
    CHI_SnpUnique,
    CHI_SnpPreferUnique,
    CHI_SnpCleanInvalid,
    CHI_SnpMakeInvalid,
    CHI_SnpUniqueStash,
    CHI_SnpMakeInvalidStash,
    CHI_SnpUniqueFwd,
    CHI_SnpPreferUniqueFwd
  )
  val nested_evt_no_meta_write = rxsnp_opcode.is(
    CHI_SnpStashUnique,
    CHI_SnpStashShared,
    CHI_SnpQuery
  )

  val meta_write_invalid = MuxCase(false.B, Seq(
    !req_evict_valid -> rxsnp_to_invalid,
    nested_writeCleanFull -> rxsnp_to_invalid,
    nested_writeBackFull -> !nested_evt_no_meta_write,
    nested_writeEvictX -> !nested_evt_no_meta_write
  ))
  
  val meta_write_shared = MuxCase(false.B, Seq(
    !req_evict_valid -> rxsnp_to_shared,
    nested_writeCleanFull -> (rxsnp_to_shared || rxsnp_snpOnceFwd || rxsnp_snpOnce),
    nested_writeBackFull -> false.B,
    nested_writeEvictX -> false.B
  ))

  val rxsnp_do_not_read_ds = rxsnp_snpQuery || rxsnp_snpMakeInvalid || rxsnp_snpMakeInvalidStash || 
                             rxsnp_snpStashUnique || rxsnp_snpStashShared
  val rxsnp_do_not_send_home_data = rxsnp_snpQuery || rxsnp_snpMakeInvalid || 
                                    rxsnp_snpPreferUniqueFwd || rxsnp_snpUniqueFwd || 
                                    (rxsnp_snpOnceFwd && !nested_writeBackFull && !nested_writeCleanFull) || 
                                    rxsnp_snpStashUnique || rxsnp_snpStashShared || rxsnp_snpMakeInvalidStash

  def metaToResp(state: UInt, dirty: Bool, passDirty: Bool): UInt = {
    val resp = Wire(UInt(CHICohResps.WIDTH.W))
    resp := Mux(passDirty, CHICohResps.I_PD, CHICohResps.I)
    when (state === L2Directory.MetaState.S) {
      resp := Mux(passDirty, CHICohResps.SC_PD, Mux(dirty, CHICohResps.SD, CHICohResps.SC))
    }.elsewhen (state === L2Directory.MetaState.US || state === L2Directory.MetaState.UU) {
      resp := Mux(passDirty, CHICohResps.UC_PD, Mux(dirty, CHICohResps.UD, CHICohResps.UC))
    }
    resp
  }

  val rxsnp_meta_write_en = WireDefault(0.U.asTypeOf(new L2Directory.MetaWriteMask))
  val rxsnp_meta_write_meta = WireDefault(0.U.asTypeOf(new L2Directory.Meta))

  rxsnp_meta_write_en.state := meta_write_invalid || meta_write_shared
  rxsnp_meta_write_en.dirty := meta_write_invalid || meta_write_shared || rxsnp_snpCleanShared
  rxsnp_meta_write_en.clients.foreach(_ := meta_write_invalid)

  rxsnp_meta_write_meta.state := MuxCase(io.tshr_dirResult.state, Seq(
    meta_write_invalid -> L2Directory.MetaState.I,
    meta_write_shared -> Mux(io.tshr_dirResult.state === L2Directory.MetaState.I, 
                             L2Directory.MetaState.I, L2Directory.MetaState.S)
  ))
  rxsnp_meta_write_meta.dirty := MuxCase(dirty_or_passdirty, Seq(
    meta_write_invalid -> false.B,
    meta_write_shared -> false.B,
    rxsnp_snpCleanShared -> false.B
  ))
  // rxsnp_meta_write_meta.clients only when to invalid
  rxsnp_meta_write_meta.clients.foreach(_ := false.B)
  rxsnp_meta_write_meta.alias := io.tshr_dirResult.alias

  val current_resp = metaToResp(io.tshr_dirResult.state, dirty_or_passdirty, false.B)
  val make_clean_resp = metaToResp(rxsnp_meta_write_meta.state, false.B, false.B)
  val pass_dirty_resp = metaToResp(rxsnp_meta_write_meta.state, false.B, true.B)

  val fwd_resp_state = MuxCase(CHICohResps.I, Seq(
    rxsnp_snpOnceFwd -> CHICohResps.I,
    rxsnp_snpCleanFwd -> CHICohResps.SC,
    rxsnp_snpSharedFwd -> CHICohResps.SC,
    rxsnp_snpNotSharedDirtyFwd -> CHICohResps.SC,
    rxsnp_snpUniqueFwd -> Mux(dirty_or_passdirty, CHICohResps.UD_PD, CHICohResps.UC),
    rxsnp_snpPreferUniqueFwd -> Mux(dirty_or_passdirty, CHICohResps.UD_PD, CHICohResps.UC)
  ))
  val home_resp_state = MuxCase(current_resp, Seq(
    rxsnp_snpQuery -> current_resp,
    rxsnp_snpStashUnique -> CHICohResps.I,
    rxsnp_snpStashShared -> CHICohResps.I,
    meta_write_invalid -> CHICohResps.I,
    meta_write_shared -> make_clean_resp
  ))
  val home_data_resp_state = MuxCase(current_resp, Seq(
    rxsnp_snpCleanShared -> Mux(dirty_or_passdirty, pass_dirty_resp, current_resp),
    meta_write_invalid -> Mux(dirty_or_passdirty, CHICohResps.I_PD, CHICohResps.I),
    meta_write_shared -> Mux(dirty_or_passdirty, pass_dirty_resp, make_clean_resp)
  ))
  // ---------------------------------------------------------

  // -- State Transition ----------------------------------------
  // Control signals
  val enter_send_sa_uop = io.toSA.SnpMakeInvalid || io.toSA.SnpToInvalid || io.toSA.SnpToShared || io.toSA.SnpToClean

  // we can decide rxsnp_need_local_ds_read at enter because:
  // io.tshr_dirResult.dirty 0->1: dirty data come from upstream and stay in tshr buffer
  // io.tshr_dirResult.dirty 1->0: impossible
  // io.tbuf_modified: TSHR buffer was modified by a non-DS source (RXDAT); the DS slot is
  // stale until the pending DSBufWb commits, so no DS read may be issued in this window
  val enter_need_local_ds_read = io.DnRXSNP.fire && !io.ds_read_done && io.tshr_dirResult.state =/= L2Directory.MetaState.I &&
                                 (rxsnp_fwd || (!rxsnp_fwd && (io.DnRXSNP.bits.RetToSrc.get.asBool || io.tshr_dirResult.dirty))) && 
                                 !rxsnp_do_not_read_ds && !io.tbuf_modified
  val enter_noop_meta_write = io.tshr_dirResult.state === L2Directory.MetaState.I
  val enter_need_meta_write = io.DnRXSNP.fire && (meta_write_shared || meta_write_invalid || rxsnp_snpCleanShared) &&
                              !enter_noop_meta_write
  val need_home_txdat = io.tshr_dirResult.state =/= L2Directory.MetaState.I && !rxsnp_do_not_send_home_data && 
                        (p_rxsnp.RetToSrc.get.asBool || dirty_or_passdirty)
  val enter_need_dct_txdat = io.DnRXSNP.fire && rxsnp_fwd && io.tshr_dirResult.state =/= L2Directory.MetaState.I
  val need_dct_txdat = p_rxsnp_need_dct_txdat
  val no_tx_inflight = !p_home_txrsp_valid && !p_home_txdat_valid && !p_dct_txdat_valid

  val fire_home_data0 = io.DnTXDAT.fire && p_txdat_dataID === 0.U && p_home_txdat_valid
  val fire_home_data2 = io.DnTXDAT.fire && p_txdat_dataID === 2.U && p_home_txdat_valid
  val fire_dct_data0 = io.DnTXDAT.fire && p_txdat_dataID === 0.U && p_dct_txdat_valid
  val fire_dct_data2 = io.DnTXDAT.fire && p_txdat_dataID === 2.U && p_dct_txdat_valid
  
  val do_home_snpresp = s_home_resp0 && s_home_resp2 && !w_snpresp0 && !w_snpresp2 && !w_ds_read && 
                        !io.EVT_active && !need_home_txdat && no_tx_inflight 
  val do_home_snprespdata0 = s_home_resp0 && !w_snpresp0 && !w_snpresp2 && !w_ds_read &&
                             !io.EVT_active && need_home_txdat && no_tx_inflight
  val do_dct_snprespdata0 = s_dct_resp0 && !s_home_resp0 && !s_home_resp2 &&
                            !w_snpresp0 && !w_snpresp2 && !w_ds_read && need_dct_txdat && no_tx_inflight

  val do_meta_write = s_meta_write && (io.DnTXRSP.fire || fire_home_data0)
  val do_sa_snpcompack = s_snpcompack && !(w_snpresp0 || w_snpresp2 || w_ds_read || s_home_resp0 || s_home_resp2 ||
                         s_dct_resp0 || s_dct_resp2)


  val block_vpipe_evt = do_home_snpresp || do_home_snprespdata0 || p_home_txdat_valid || p_home_txrsp_valid || 
                        do_dct_snprespdata0 || p_dct_txdat_valid

  // State update
  when (io.DnRXSNP.fire) {
    p_rxsnp := io.DnRXSNP.bits
    p_rxsnp_to_invalid := meta_write_invalid
    p_rxsnp_need_dct_txdat := enter_need_dct_txdat
    p_sa_passdirty := false.B

    s_meta_write := enter_need_meta_write
    s_home_resp0 := true.B
    s_home_resp2 := true.B
    s_dct_resp0 := enter_need_dct_txdat
    s_dct_resp2 := enter_need_dct_txdat
    s_snpcompack := io.toSA.SnpToClean

    w_ds_read := enter_need_local_ds_read && !io.ds_read_done
    w_snpresp0 := enter_send_sa_uop && !io.fromSA.SnpResp
    w_snpresp2 := enter_send_sa_uop && !io.fromSA.SnpResp
  }

  when (io.ds_read_done) {
    w_ds_read := false.B
  }

  when (io.fromSA.SnpRespData0) {
    w_snpresp0 := false.B
  }

  when (io.fromSA.SnpRespData2) {
    w_snpresp2 := false.B
    p_sa_passdirty := io.fromSA.PASSDIRTY
  }

  when (io.fromSA.SnpResp && !io.DnRXSNP.fire) {
    w_snpresp0 := false.B
    w_snpresp2 := false.B
    p_sa_passdirty := io.fromSA.PASSDIRTY
  }

  when (io.DnTXRSP.fire) {
    s_home_resp0 := false.B
    s_home_resp2 := false.B
    p_home_txrsp_valid := false.B
  }

  when (fire_home_data0) {
    s_home_resp0 := false.B
    p_txdat_dataID := 2.U
  }

  when (fire_home_data2) {
    s_home_resp2 := false.B
    p_home_txdat_valid := false.B
  }

  when (fire_dct_data0) {
    s_dct_resp0 := false.B
    p_txdat_dataID := 2.U
  }

  when (fire_dct_data2) {
    s_dct_resp2 := false.B
    p_dct_txdat_valid := false.B
  }

  when (do_meta_write) {
    s_meta_write := false.B
  }

  when (io.toSA.SnpCompAck) {
    s_snpcompack := false.B
  }

  when (do_home_snpresp) {
    p_home_txrsp_valid := true.B
    p_home_txrsp_resp := home_resp_state
    p_fwd_state := Mux(need_dct_txdat, fwd_resp_state, 0.U)
  }

  when (do_home_snprespdata0) {
    p_dct_txdat_valid := false.B
    p_home_txdat_valid := true.B
    p_home_txdat_resp := home_data_resp_state
    p_txdat_dataID := 0.U
    p_fwd_state := Mux(need_dct_txdat, fwd_resp_state, 0.U)
  }

  when (do_dct_snprespdata0) {
    p_dct_txdat_valid := true.B
    p_dct_txdat_resp := p_fwd_state
    p_txdat_dataID := 0.U
    p_fwd_state := 0.U
  }
  // ---------------------------------------------------------

  // -- Interact with Snoop Agent signals --------------------
  io.toSA.SnpMakeInvalid := io.DnRXSNP.fire && rxsnp_opcode.is(
    CHI_SnpMakeInvalid,
    CHI_SnpMakeInvalidStash
  )

  io.toSA.SnpToInvalid := io.DnRXSNP.fire && (rxsnp_opcode.is(
    CHI_SnpUnique,
    CHI_SnpPreferUnique,
    CHI_SnpCleanInvalid,
    CHI_SnpUniqueFwd,
    CHI_SnpPreferUniqueFwd,
    CHI_SnpUniqueStash
  ) || rxsnp_opcode.is(
    CHI_SnpClean,
    CHI_SnpNotSharedDirty,
    CHI_SnpShared,
    CHI_SnpOnce,
    CHI_SnpSharedFwd,
    CHI_SnpCleanFwd,
    CHI_SnpOnceFwd,
    CHI_SnpNotSharedDirtyFwd,
    CHI_SnpCleanShared
  ) && (nested_writeBackFull || nested_writeEvictX))

  io.toSA.SnpToShared := io.DnRXSNP.fire && rxsnp_opcode.is(
    CHI_SnpClean,
    CHI_SnpNotSharedDirty,
    CHI_SnpShared,
    CHI_SnpOnce,
    CHI_SnpSharedFwd,
    CHI_SnpCleanFwd,
    CHI_SnpOnceFwd,
    CHI_SnpNotSharedDirtyFwd
  ) && !nested_writeBackFull && !nested_writeEvictX

  io.toSA.SnpToClean := io.DnRXSNP.fire && rxsnp_opcode.is(
    CHI_SnpCleanShared
  ) && !nested_writeBackFull && !nested_writeEvictX

  io.toSA.SnpCompAck := do_sa_snpcompack
  io.toSA.CLIENTS := 0.U.asTypeOf(io.toSA.CLIENTS)
  io.toSA.ALIAS := io.tshr_dirResult.alias
  io.toSA.IS_L2_EVICT := false.B // always generated by REQ vPipe
  io.toSA.IS_SRC_EXCLUDED := false.B
  // ---------------------------------------------------------

  // -- Send DnTXRSP & DnTXDAT -------------------------------
  io.DnTXRSP.valid := p_home_txrsp_valid
  io.DnTXRSP.bits := DontCare
  io.DnTXRSP.bits.QoS.get := p_rxsnp.QoS.get
  io.DnTXRSP.bits.TgtID.get := p_rxsnp.SrcID.get
  io.DnTXRSP.bits.SrcID.get := io.consts.nodeId
  io.DnTXRSP.bits.TxnID.get := p_rxsnp.TxnID.get
  io.DnTXRSP.bits.Opcode.get := Mux(need_dct_txdat, CHI_SnpRespFwded.U, CHI_SnpResp.U)
  io.DnTXRSP.bits.RespErr.get := 0.U
  io.DnTXRSP.bits.Resp.get := p_home_txrsp_resp
  io.DnTXRSP.bits.FwdState(p_fwd_state)
  // io.DnTXRSP.bits.DataPull(0.U)
  io.DnTXRSP.bits.CBusy.foreach(_ := 0.U)
  io.DnTXRSP.bits.DBID_PGroupID_StashGroupID_TagGroupID.foreach(_ := 0.U)
  io.DnTXRSP.bits.PCrdType.get := 0.U
  io.DnTXRSP.bits.TagOp.foreach(_ := 0.U)
  io.DnTXRSP.bits.TraceTag.get := p_rxsnp.TraceTag.get

  // When the TSHR buffer is the data source (buffer modified by RXDAT, DS read
  // suppressed), each SNP data beat is blocked until its buffer half is ready.
  val txdat_beat_ready = Mux(p_txdat_dataID === 0.U, io.tbuf_half0_ready, io.tbuf_half2_ready)
  io.DnTXDAT.valid := (p_home_txdat_valid || p_dct_txdat_valid) && (!io.tbuf_modified || txdat_beat_ready)
  io.DnTXDAT.bits := DontCare
  io.DnTXDAT.bits.QoS.get := p_rxsnp.QoS.get
  io.DnTXDAT.bits.TgtID.get := Mux(p_dct_txdat_valid, p_rxsnp.FwdNID.get, p_rxsnp.SrcID.get)
  io.DnTXDAT.bits.SrcID.get := io.consts.nodeId
  io.DnTXDAT.bits.TxnID.get := Mux(p_dct_txdat_valid, p_rxsnp.FwdTxnID.get, p_rxsnp.TxnID.get)
  io.DnTXDAT.bits.HomeNID.get := p_rxsnp.SrcID.get
  io.DnTXDAT.bits.Opcode.get := Mux(p_dct_txdat_valid, CHI_CompData.U, Mux(need_dct_txdat, CHI_SnpRespDataFwded.U, CHI_SnpRespData.U))
  io.DnTXDAT.bits.RespErr.get := 0.U
  io.DnTXDAT.bits.Resp.get := Mux(p_dct_txdat_valid, p_dct_txdat_resp, p_home_txdat_resp)
  io.DnTXDAT.bits.FwdState(p_fwd_state)
  // io.DnTXDAT.bits.DataPull(0.U)
  // io.DnTXDAT.bits.DataSource(0.U)
  io.DnTXDAT.bits.CBusy.foreach(_ := 0.U)
  io.DnTXDAT.bits.DBID.get := p_rxsnp.TxnID.get
  io.DnTXDAT.bits.CCID.get := 0.U
  io.DnTXDAT.bits.DataID.get := p_txdat_dataID
  io.DnTXDAT.bits.TagOp.foreach(_ := 0.U)
  io.DnTXDAT.bits.Tag.foreach(_ := 0.U)
  io.DnTXDAT.bits.TU.foreach(_ := 0.U)
  io.DnTXDAT.bits.TraceTag.get := p_rxsnp.TraceTag.get
  io.DnTXDAT.bits.RSVDC.foreach(_ := 0.U)
  io.DnTXDAT.bits.BE.get := Fill(paramCHI.datBEWidth, true.B)
  // TODO: fill from TSHR buffer in the outer TXDAT mux.
  io.DnTXDAT.bits.Data.get := 0.U(paramCHI.datDataWidth.W) 
  io.DnTXDAT.bits.DataCheck.foreach(_ := 0.U)
  io.DnTXDAT.bits.Poison.foreach(_ := 0.U)
  // ---------------------------------------------------------

  // -- Other IO signals -------------------------------------
  io.tshr_meta_write_en := Mux(do_meta_write, rxsnp_meta_write_en, 0.U.asTypeOf(new L2Directory.MetaWriteMask))
  io.tshr_meta_write_meta := rxsnp_meta_write_meta

  io.ds_read_en := enter_need_local_ds_read && !io.ds_read_done

  io.ds_wb_cancel := active && p_rxsnp_to_invalid

  io.blockRBE.EVT := block_vpipe_evt
  io.blockRBE.SNP := active 
  io.blockRBE.EVB := active
  io.blockRBE.REQ := active
  io.free := !active
  // ---------------------------------------------------------

  // -- Assertion --------------------------------------------
  val rxsnp_ret_to_src_must_be_zero = rxsnp_opcode.is(
    CHI_SnpQuery,
    CHI_SnpCleanShared,
    CHI_SnpCleanInvalid,
    CHI_SnpMakeInvalid,
    CHI_SnpOnceFwd,
    CHI_SnpUniqueFwd,
    CHI_SnpUniqueStash,
    CHI_SnpMakeInvalidStash,
    CHI_SnpStashUnique,
    CHI_SnpStashShared
  )
  val rxsnp_do_not_go_to_sd_must_be_one = rxsnp_opcode.is(
    CHI_SnpUnique,
    CHI_SnpCleanShared,
    CHI_SnpCleanInvalid,
    CHI_SnpMakeInvalid,
    CHI_SnpUniqueFwd,
    CHI_SnpUniqueStash,
    CHI_SnpMakeInvalidStash,
    CHI_SnpStashUnique,
    CHI_SnpStashShared
  )
  // A nested eviction cancelled during a P-credit retry carries PCrdReturn
  // (L2VPipeREQ.evictback_txreq_opcode): it performs no write-back and has no
  // meta effect (the tables' defaults already produce that) — accept it as
  // supported.
  val req_evict_pcrdreturn = req_evict_valid && req_evict_opcode === CHI_PCrdReturn.asUInt
  val req_evict_supported = !req_evict_valid || nested_writeBackFull ||
                            nested_writeCleanFull || nested_writeEvictX || req_evict_pcrdreturn
  val dir_state_unique = io.tshr_dirResult.state === L2Directory.MetaState.US ||
                         io.tshr_dirResult.state === L2Directory.MetaState.UU
  val rxsnp_no_sa_uop = rxsnp_snpQuery || rxsnp_snpStashUnique || rxsnp_snpStashShared
  val sa_uops = Seq(
    io.toSA.SnpMakeInvalid,
    io.toSA.SnpToInvalid,
    io.toSA.SnpToShared,
    io.toSA.SnpToClean
  )
  val meta_write_any = io.tshr_meta_write_en.asUInt.orR

  val txrsp_stalled_q = RegNext(io.DnTXRSP.valid && !io.DnTXRSP.ready, false.B)
  val txrsp_bits_q = RegNext(io.DnTXRSP.bits.asUInt)
  val txdat_stalled_q = RegNext(io.DnTXDAT.valid && !io.DnTXDAT.ready, false.B)
  val txdat_bits_q = RegNext(io.DnTXDAT.bits.asUInt)
  val active_q = RegNext(active, false.B)
  val p_rxsnp_q = RegNext(p_rxsnp.asUInt)
  val req_evict_valid_q = RegNext(req_evict_valid, false.B)
  val req_evict_opcode_q = RegNext(req_evict_opcode, 0.U)
  val expect_meta_write = RegInit(false.B)
  val expect_dct = RegInit(false.B)
  val expect_compack = RegInit(false.B)
  val meta_write_seen = RegInit(false.B)
  val home_complete_seen = RegInit(false.B)
  val dct_complete_seen = RegInit(false.B)
  val compack_seen = RegInit(false.B)

  when (io.DnRXSNP.fire) {
    expect_meta_write := enter_need_meta_write
    expect_dct := enter_need_dct_txdat
    expect_compack := io.toSA.SnpToClean
    meta_write_seen := false.B
    home_complete_seen := false.B
    dct_complete_seen := false.B
    compack_seen := false.B
  }
  when (meta_write_any) {
    assert(!meta_write_seen,
      "TSHR @ %m SNP vPipe committed meta more than once for one transaction")
    meta_write_seen := true.B
  }
  when (io.DnTXRSP.fire || fire_home_data2) {
    assert(!home_complete_seen,
      "TSHR @ %m SNP vPipe completed more than one Home response")
    home_complete_seen := true.B
  }
  when (fire_dct_data2) {
    assert(!dct_complete_seen,
      "TSHR @ %m SNP vPipe completed DCT more than once")
    dct_complete_seen := true.B
  }
  when (io.toSA.SnpCompAck) {
    assert(!compack_seen,
      "TSHR @ %m SNP vPipe emitted SnpCompAck more than once")
    compack_seen := true.B
  }
  when (active_q && !active) {
    assert(home_complete_seen,
      "TSHR @ %m SNP vPipe completed without a Home response")
    assert(meta_write_seen === expect_meta_write,
      "TSHR @ %m SNP vPipe meta-write completion count disagrees with entry decode")
    assert(dct_complete_seen === expect_dct,
      "TSHR @ %m SNP vPipe DCT completion count disagrees with entry decode")
    assert(compack_seen === expect_compack,
      "TSHR @ %m SNP vPipe SnpCompAck completion count disagrees with entry decode")
  }

  // RXSNP acceptance and CHI request-field legality.
  assert(!(io.DnRXSNP.fire && active),
    "TSHR @ %m SNP vPipe accepted a new snoop while active")
  assert(req_evict_supported,
    "TSHR @ %m SNP vPipe observed an unsupported REQ_evict opcode")
  when (io.DnRXSNP.fire || active) {
    assert(io.tshr_dirResult.state =/= L2Directory.MetaState.I ||
           (!io.tshr_dirResult.dirty && !io.tshr_dirResult.clients.asUInt.orR),
      "TSHR @ %m SNP vPipe observed dirty data or clients while local meta was invalid")
  }
  when (io.DnRXSNP.fire) {
    assert(PopCount(sa_uops) <= 1.U,
      "TSHR @ %m SNP vPipe issued multiple SnoopAgent uops")
    assert(rxsnp_no_sa_uop || PopCount(sa_uops) === 1.U,
      "TSHR @ %m SNP vPipe did not issue exactly one required SnoopAgent uop")
    assert(!rxsnp_no_sa_uop || PopCount(sa_uops) === 0.U,
      "TSHR @ %m SNP vPipe issued a SnoopAgent uop on a no-SA path")
    assert(!rxsnp_ret_to_src_must_be_zero || !io.DnRXSNP.bits.RetToSrc.get.asBool,
      "TSHR @ %m SNP vPipe received RetToSrc=1 on an opcode that requires zero")
    assert(!rxsnp_do_not_go_to_sd_must_be_one || io.DnRXSNP.bits.DoNotGoToSD.get.asBool,
      "TSHR @ %m SNP vPipe received DoNotGoToSD=0 on an opcode that requires one")
    assert(!rxsnp_snpQuery || !io.DnRXSNP.bits.DoNotGoToSD.get.asBool,
      "TSHR @ %m SNP vPipe received DoNotGoToSD=1 on SnpQuery")
    assert(rxsnp_fwd || io.DnRXSNP.bits.FwdNID.get === 0.U,
      "TSHR @ %m SNP vPipe received FwdNID on a non-Fwd snoop")
    assert(rxsnp_fwd || rxsnp_stash || io.DnRXSNP.bits.FwdTxnID.get === 0.U,
      "TSHR @ %m SNP vPipe received FwdTxnID on a non-Fwd non-stash snoop")
  }

  // External completion events must match an outstanding wait state.
  assert(!io.fromSA.SnpRespData0 || w_snpresp0,
    "TSHR @ %m SNP vPipe received unexpected or duplicate SA DataID 0")
  assert(!io.fromSA.SnpRespData2 || w_snpresp2,
    "TSHR @ %m SNP vPipe received unexpected or duplicate SA DataID 2")
  assert(!io.fromSA.SnpResp || w_snpresp0 || w_snpresp2 || enter_send_sa_uop,
    "TSHR @ %m SNP vPipe received an unexpected SA SnpResp")
  assert(!io.fromSA.SnpResp || enter_send_sa_uop || (w_snpresp0 && w_snpresp2),
    "TSHR @ %m SNP vPipe mixed SA data beats with a no-data SnpResp")
  assert(!(io.fromSA.SnpResp && (io.fromSA.SnpRespData0 || io.fromSA.SnpRespData2)),
    "TSHR @ %m SNP vPipe received SA SnpResp and SnpRespData together")
  when (io.fromSA.SnpRespData0 || io.fromSA.SnpRespData2) {
    assert(!rxsnp_snpMakeInvalid && !rxsnp_snpMakeInvalidStash,
      "TSHR @ %m SNP vPipe received data for a MakeInvalid SnoopAgent request")
  }

  // State bits and transaction ownership.
  assert(!(s_home_resp0 && !s_home_resp2),
    "TSHR @ %m SNP vPipe has Home DataID 0 pending after DataID 2 completed")
  assert(!(s_dct_resp0 && !s_dct_resp2),
    "TSHR @ %m SNP vPipe has DCT DataID 0 pending after DataID 2 completed")
  assert(PopCount(Seq(p_home_txrsp_valid, p_home_txdat_valid, p_dct_txdat_valid)) <= 1.U,
    "TSHR @ %m SNP vPipe has multiple downstream transmit owners")
  assert(PopCount(sa_uops :+ io.toSA.SnpCompAck) <= 1.U,
    "TSHR @ %m SNP vPipe issued multiple SnoopAgent operations")
  assert(!sa_uops.reduce(_ || _) || io.DnRXSNP.fire,
    "TSHR @ %m SNP vPipe issued a SnoopAgent request outside the snoop entry cycle")
  assert(PopCount(Seq(do_home_snpresp, do_home_snprespdata0, do_dct_snprespdata0)) <= 1.U,
    "TSHR @ %m SNP vPipe scheduled multiple downstream responses")
  assert(!(s_dct_resp0 || s_dct_resp2 || p_dct_txdat_valid) || need_dct_txdat,
    "TSHR @ %m SNP vPipe scheduled DCT without a latched DCT decision")
  assert(!p_dct_txdat_valid || (!s_home_resp0 && !s_home_resp2 &&
                                !p_home_txrsp_valid && !p_home_txdat_valid),
    "TSHR @ %m SNP vPipe started DCT before the Home response completed")
  assert(!p_home_txrsp_valid || (s_home_resp0 && s_home_resp2),
    "TSHR @ %m SNP vPipe TXRSP owner does not match Home scheduling state")
  assert(!(p_home_txrsp_valid || p_home_txdat_valid || p_dct_txdat_valid) ||
         !(w_snpresp0 || w_snpresp2 || w_ds_read),
    "TSHR @ %m SNP vPipe established a downstream valid before all data sources completed")
  when (active) {
    assert(!need_dct_txdat || rxsnp_fwd,
      "TSHR @ %m SNP vPipe latched DCT work for a non-Fwd snoop")
    assert(!s_meta_write || meta_write_invalid || meta_write_shared || rxsnp_snpCleanShared,
      "TSHR @ %m SNP vPipe retained a meta write for an opcode that does not modify meta")
    assert(!s_snpcompack || (rxsnp_snpCleanShared &&
           !nested_writeBackFull && !nested_writeEvictX),
      "TSHR @ %m SNP vPipe retained SnpCompAck on a non-ToClean path")
  }
  when (p_home_txdat_valid) {
    assert((p_txdat_dataID === 0.U && s_home_resp0 && s_home_resp2) ||
           (p_txdat_dataID === 2.U && !s_home_resp0 && s_home_resp2),
      "TSHR @ %m SNP vPipe Home TXDAT DataID disagrees with scheduling state")
  }
  when (p_dct_txdat_valid) {
    assert((p_txdat_dataID === 0.U && s_dct_resp0 && s_dct_resp2) ||
           (p_txdat_dataID === 2.U && !s_dct_resp0 && s_dct_resp2),
      "TSHR @ %m SNP vPipe DCT TXDAT DataID disagrees with scheduling state")
  }
  assert(!active_q || !active || p_rxsnp.asUInt === p_rxsnp_q,
    "TSHR @ %m SNP vPipe request payload changed during an active transaction")

  // Output-channel exclusivity, ownership, ordering, and stability.
  assert(!(io.DnTXRSP.valid && io.DnTXDAT.valid),
    "TSHR @ %m SNP vPipe asserted TXRSP and TXDAT valid together")
  assert(!(io.DnTXDAT.valid && p_txdat_dataID =/= 0.U && p_txdat_dataID =/= 2.U),
    "TSHR @ %m SNP vPipe emitted an unsupported TXDAT DataID")
  assert(!io.DnTXRSP.valid || active,
    "TSHR @ %m SNP vPipe asserted TXRSP valid while inactive")
  assert(!io.DnTXDAT.valid || active,
    "TSHR @ %m SNP vPipe asserted TXDAT valid while inactive")
  assert(!(io.DnTXRSP.fire && (w_snpresp0 || w_snpresp2 || w_ds_read)),
    "TSHR @ %m SNP vPipe sent TXRSP before all data sources completed")
  assert(!(io.DnTXDAT.fire && (w_snpresp0 || w_snpresp2 || w_ds_read)),
    "TSHR @ %m SNP vPipe sent TXDAT before all data sources completed")
  assert(!io.DnTXRSP.valid || io.DnTXRSP.bits.Resp.get < CHICohResps.I_PD,
    "TSHR @ %m SNP vPipe used PassDirty on a response without data")
  when (io.DnTXRSP.valid) {
    assert(io.DnTXRSP.bits.Opcode.get === Mux(need_dct_txdat, CHI_SnpRespFwded.U, CHI_SnpResp.U),
      "TSHR @ %m SNP vPipe emitted an incorrect Home TXRSP opcode")
    assert(io.DnTXRSP.bits.QoS.get === p_rxsnp.QoS.get &&
           io.DnTXRSP.bits.TgtID.get === p_rxsnp.SrcID.get &&
           io.DnTXRSP.bits.SrcID.get === io.consts.nodeId &&
           io.DnTXRSP.bits.TxnID.get === p_rxsnp.TxnID.get &&
           io.DnTXRSP.bits.RespErr.get === 0.U &&
           io.DnTXRSP.bits.TraceTag.get === p_rxsnp.TraceTag.get,
      "TSHR @ %m SNP vPipe emitted incorrect Home TXRSP routing or control fields")
    assert(need_dct_txdat || io.DnTXRSP.bits.FwdState_DataPull.get === 0.U,
      "TSHR @ %m SNP vPipe drove FwdState/DataPull on a non-Fwd Home TXRSP")
  }
  when (io.DnTXDAT.valid) {
    assert(io.DnTXDAT.bits.QoS.get === p_rxsnp.QoS.get &&
           io.DnTXDAT.bits.SrcID.get === io.consts.nodeId &&
           io.DnTXDAT.bits.HomeNID.get === p_rxsnp.SrcID.get &&
           io.DnTXDAT.bits.DBID.get === p_rxsnp.TxnID.get &&
           io.DnTXDAT.bits.RespErr.get === 0.U &&
           io.DnTXDAT.bits.TraceTag.get === p_rxsnp.TraceTag.get,
      "TSHR @ %m SNP vPipe emitted incorrect TXDAT common routing or control fields")
  }
  when (p_dct_txdat_valid) {
    assert(io.DnTXDAT.bits.Opcode.get === CHI_CompData.U,
      "TSHR @ %m SNP vPipe DCT data does not use CompData")
    assert(io.DnTXDAT.bits.FwdState_DataPull_DataSource.get === 0.U,
      "TSHR @ %m SNP vPipe drove FwdState/DataPull/DataSource on CompData")
    assert(p_dct_txdat_resp =/= CHICohResps.SD && p_dct_txdat_resp =/= CHICohResps.SD_PD,
      "TSHR @ %m SNP vPipe emitted an unsupported SD DCT response")
    assert(io.DnTXDAT.bits.TgtID.get === p_rxsnp.FwdNID.get &&
           io.DnTXDAT.bits.TxnID.get === p_rxsnp.FwdTxnID.get &&
           io.DnTXDAT.bits.HomeNID.get === p_rxsnp.SrcID.get &&
           io.DnTXDAT.bits.DBID.get === p_rxsnp.TxnID.get,
      "TSHR @ %m SNP vPipe emitted incorrect DCT routing IDs")
  }
  when (p_home_txdat_valid) {
    assert(io.DnTXDAT.bits.Opcode.get === Mux(need_dct_txdat, CHI_SnpRespDataFwded.U, CHI_SnpRespData.U),
      "TSHR @ %m SNP vPipe emitted an incorrect Home TXDAT opcode")
    assert(io.DnTXDAT.bits.TgtID.get === p_rxsnp.SrcID.get &&
           io.DnTXDAT.bits.TxnID.get === p_rxsnp.TxnID.get &&
           (need_dct_txdat || io.DnTXDAT.bits.FwdState_DataPull_DataSource.get === 0.U),
      "TSHR @ %m SNP vPipe emitted incorrect Home TXDAT routing IDs")
  }
  when (io.DnTXDAT.valid) {
    assert(io.DnTXDAT.bits.BE.get.andR,
      "TSHR @ %m SNP vPipe emitted a full-line response without all byte enables")
  }
  when (txrsp_stalled_q) {
    assert(io.DnTXRSP.valid && io.DnTXRSP.bits.asUInt === txrsp_bits_q,
      "TSHR @ %m SNP vPipe changed TXRSP payload while stalled")
  }
  when (txdat_stalled_q) {
    assert(io.DnTXDAT.valid && io.DnTXDAT.bits.asUInt === txdat_bits_q,
      "TSHR @ %m SNP vPipe changed TXDAT payload while stalled")
  }

  // Meta updates and buffer ownership.
  assert(meta_write_any === do_meta_write,
    "TSHR @ %m SNP vPipe meta-write output disagrees with its commit event")
  when (meta_write_any) {
    assert(active && s_meta_write && io.tshr_meta_write_en.dirty,
      "TSHR @ %m SNP vPipe emitted an inactive meta write or failed to write dirty")
    assert(io.tshr_meta_write_en.asUInt === rxsnp_meta_write_en.asUInt,
      "TSHR @ %m SNP vPipe emitted a meta-write mask inconsistent with snoop decode")
    assert(!rxsnp_snpQuery,
      "TSHR @ %m SNP vPipe modified meta for SnpQuery")
    assert(!io.tshr_meta_write_meta.dirty &&
           io.tshr_meta_write_meta.alias === io.tshr_dirResult.alias,
      "TSHR @ %m SNP vPipe meta write did not commit clean data or preserve alias")
    assert(io.tshr_meta_write_en.state === (meta_write_invalid || meta_write_shared),
      "TSHR @ %m SNP vPipe state-write mask disagrees with snoop transition")
    assert(io.tshr_meta_write_en.clients.asUInt.orR === meta_write_invalid,
      "TSHR @ %m SNP vPipe client-write mask disagrees with invalidation")
    when (io.tshr_meta_write_en.state) {
      assert(io.tshr_meta_write_meta.state === L2Directory.MetaState.I ||
             io.tshr_meta_write_meta.state === L2Directory.MetaState.S,
        "TSHR @ %m SNP vPipe state write did not target I or SC")
    }.otherwise {
      assert(rxsnp_snpCleanShared && !io.tshr_meta_write_en.clients.asUInt.orR,
        "TSHR @ %m SNP vPipe emitted a dirty-only write outside SnpCleanShared")
    }
    when (io.tshr_meta_write_en.clients.asUInt.orR) {
      assert(io.tshr_meta_write_meta.state === L2Directory.MetaState.I &&
             !io.tshr_meta_write_meta.clients.asUInt.orR,
        "TSHR @ %m SNP vPipe client invalidation did not write local I/no-clients")
    }
  }
  assert(!io.ds_read_en || io.DnRXSNP.fire,
    "TSHR @ %m SNP vPipe asserted ds_read_en outside the request-entry cycle")
  when (io.ds_read_en) {
    assert(!io.ds_read_done && io.tshr_dirResult.state =/= L2Directory.MetaState.I &&
           !rxsnp_do_not_read_ds,
      "TSHR @ %m SNP vPipe issued an unnecessary or already-completed DS read")
  }
  assert(!(io.ds_read_en && io.tbuf_modified),
    "TSHR @ %m SNP vPipe issued DS read while TSHR buffer holds non-DS-source (RXDAT) data")
  assert(!(io.DnTXRSP.valid || io.DnTXDAT.valid) || io.blockRBE.EVT,
    "TSHR @ %m SNP vPipe did not block EVT while owning a downstream response")
  assert(!(io.EVT_active && io.blockRBE.EVT),
    "TSHR @ %m SNP vPipe observed EVT active while the SNP data path blocked EVT")
  assert(io.free === !active && io.blockRBE.SNP === active && io.blockRBE.REQ === active,
    "TSHR @ %m SNP vPipe free/RBE ownership outputs disagree with active state")
  assert(io.blockRBE.EVT === block_vpipe_evt,
    "TSHR @ %m SNP vPipe EVT block output disagrees with buffer ownership")
  when (io.toSA.SnpCompAck) {
    assert(active && rxsnp_opcode.is(CHI_SnpCleanShared) &&
           !w_snpresp0 && !w_snpresp2 && !w_ds_read &&
           !s_home_resp0 && !s_home_resp2 &&
           !s_dct_resp0 && !s_dct_resp2 &&
           !p_home_txrsp_valid && !p_home_txdat_valid && !p_dct_txdat_valid,
      "TSHR @ %m SNP vPipe emitted SnpCompAck before SnpCleanShared completed")
  }

  // Opcode-specific negative behavior.
  when (active && rxsnp_snpQuery) {
    assert(!w_snpresp0 && !w_snpresp2 && !w_ds_read && !s_meta_write &&
           !s_dct_resp0 && !s_dct_resp2 && !io.DnTXDAT.valid,
      "TSHR @ %m SNP vPipe performed data, SA, DCT, or meta work for SnpQuery")
  }
  when (active && rxsnp_snpMakeInvalid) {
    assert(!w_ds_read && !need_dct_txdat && !io.DnTXDAT.valid,
      "TSHR @ %m SNP vPipe read or returned data for SnpMakeInvalid")
  }
  when (active && rxsnp_snpMakeInvalidStash) {
    assert(!w_ds_read && !need_dct_txdat && !io.DnTXDAT.valid,
      "TSHR @ %m SNP vPipe read or returned data for SnpMakeInvalidStash")
  }
  when (active && (rxsnp_snpStashUnique || rxsnp_snpStashShared)) {
    assert(!w_snpresp0 && !w_snpresp2 && !w_ds_read && !s_meta_write &&
           !s_dct_resp0 && !s_dct_resp2 && !io.DnTXDAT.valid,
      "TSHR @ %m SNP vPipe performed SA, DS, meta, data, or DCT work on an ignored Stash snoop")
  }
  when (io.DnTXRSP.valid && rxsnp_stash) {
    assert(io.DnTXRSP.bits.FwdState_DataPull.get === 0.U,
      "TSHR @ %m SNP vPipe requested DataPull on a no-DataPull Stash path")
    when (rxsnp_snpStashUnique || rxsnp_snpStashShared) {
      assert(io.DnTXRSP.bits.Opcode.get === CHI_SnpResp.U &&
             io.DnTXRSP.bits.Resp.get === CHICohResps.I,
        "TSHR @ %m SNP vPipe emitted a non-I response on an ignored Stash snoop")
    }
  }
  when (io.DnTXDAT.valid && rxsnp_stash) {
    assert(rxsnp_snpUniqueStash && io.DnTXDAT.bits.FwdState_DataPull_DataSource.get === 0.U,
      "TSHR @ %m SNP vPipe emitted illegal Stash data or requested DataPull")
  }
  when (active && !rxsnp_fwd) {
    assert(!need_dct_txdat && !s_dct_resp0 && !s_dct_resp2 && !p_dct_txdat_valid,
      "TSHR @ %m SNP vPipe scheduled DCT for a non-Fwd snoop")
  }
  // ---------------------------------------------------------
}
