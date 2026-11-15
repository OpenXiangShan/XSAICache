package oceanus.l2.tshr

import chisel3._
import chisel3.util._
import utility._
import oceanus.chi.bundle._
import oceanus.compactchi._
import oceanus.compactchi.CCHIOpcode._
import oceanus.l2._
import oceanus.l2.L2Directory._
import oceanus.l2.L2DataStorage._
import oceanus.l2.tshr._
import org.chipsalliance.cde.config.Parameters


class L2TSHRDirectoryProxy(val id: Int)(implicit val p: Parameters) extends Module with HasL2Params {

  class DirReadFSM extends Bundle {
    val PreArb = Bool()     // sending Read Request to Directory, waiting for Arbiter completion
    val PostArb = Bool()    // Read Request has been accepted by Directory
    val Done = Bool()       // Read Response has been received from Directory
    val ReplPreArb = Bool() // sending Replacement Read Request to Directory, waiting for Arbiter completion
    val ReplPostArb = Bool()// Replacement Read Request has been accepted by Directory
    val ReplDone = Bool()   // Replacement Read Response has been received from Directory
    val ReplRetry = Bool()  // Replacement Read Retry Ack has been received from Directory
    def NotYet =            // haven't sent Read Request to Directory
      !PreArb && !PostArb && !Done && !ReplPreArb && !ReplPostArb && !ReplDone && !ReplRetry
    def Repl =
      ReplPreArb || ReplPostArb || ReplDone || ReplRetry
  }

  object DirReadFSM {
    def init = {
      val initState = Wire(new DirReadFSM)
      initState.elements.foreach(_._2 := false.B)
      initState
    }
  }

  class DirWriteFSM extends Bundle {
    val PreArb = Bool()     // sending Write Request to Directory, waiting for Arbiter completion
    val Done = Bool()       // Write Request has been accepted by Directory and observable to later requests
    def NotYet =            // haven't sent Write Request to Directory
      !PreArb && !Done
  }

  object DirWriteFSM {
    def init = {
      val initState = Wire(new DirWriteFSM)
      initState.elements.foreach(_._2 := false.B)
      initState
    }
  }

  class DirRelFSM extends Bundle {
    val PreArb = Bool()     // sending Release-train Request to Directory, waiting for DirRel Arbiter completion
    val Done = Bool()       // Release-train Request has been accepted by Directory
    def NotYet =            // haven't sent Release-train Request to Directory
      !PreArb && !Done
  }

  object DirRelFSM {
    def init = {
      val initState = Wire(new DirRelFSM)
      initState.elements.foreach(_._2 := false.B)
      initState
    }
  }

  val io = IO(new Bundle {

    val tshrId = Input(UInt(mshrIndexWidth.W))

    val toDir = Output(new L2Directory.PathToDirectory)
    val fromDir = Input(new L2Directory.PathFromDirectory)

    val tshr_valid = Input(Bool())
    val tshr_paddr = Input(UInt(paramL2.physicalAddrWidth.W))

    val tshr_alloc = Input(Bool())
    val tshr_reuse = Input(Bool())
    val tshr_inactive = Input(Bool())
    val tshr_inactivate = Input(Bool())
    val tshr_dealloc = Input(Bool())

    val read_arbed = Input(Bool())
    val read_en = Input(Bool())
    val repl_en = Input(Bool())
    val repl_reset = Input(Bool())

    // Acceptance-time release pulse and exact way from the EVT vPipe. WayValid
    // uses the flit way; other releases use their DirRd-derived way.
    val repl_release = if (enableRRIPReleaseUpdate) Some(Input(Bool())) else None
    val repl_way = if (enableRRIPReleaseUpdate) Some(Input(UInt(wayBits.W))) else None

    val meta = Input(new L2Directory.Meta)
    val meta_way = Input(UInt(wayBits.W))

    val meta_modify = Input(Bool())
    val meta_modified = Input(new L2Directory.MetaWriteMask)
    val tag_modify = Input(Bool())
    val tag_modified = Input(Bool())
    val rrip_update = Input(Bool())
    val rrip_prefetch = Input(Bool())
    val rrip_refill_prefetch = Input(Bool())

    val rd_idle = Output(Bool())
    val rd_accept = Output(Bool())
    val rd_done = Output(Bool())
    val rd_done_flow = Output(Bool())

    val repl_idle = Output(Bool())
    val repl_ready = Output(Bool())
    val repl_accept = Output(Bool())
    val repl_done = Output(Bool())
    val repl_retry = Output(Bool())

    val repl_lock_req = Input(Bool())
    val repl_lock_way = Input(UInt(wayBits.W))
    val repl_lock_conflict = Output(Bool())
    val repl_lock_release = Input(Bool())

    val wb_locked = Input(Bool())
    val snoop_active = Input(Bool())
    val wb_cancel = Input(Bool())
    val wb_aux = Input(Bool())
    val wb_accept = Output(Bool())
    val wb_done = Output(Bool())
    val rel_idle = Output(Bool())
  })

  // Whether the meta was aggressively immediately trying to be written back on every modification.
  val configAggressiveWrite = false

  // Whether ReplRd only happens after a done DirRd.
  //  - Under most situations, this should always be set to 'true', except debug or feature evaluation,
  //    since we always need hit/miss info (from any done Directory Read) before any replacement action.
  //  - When set to 'false', the "rd_done" and "repl_ready" signals might not behave as expected.
  val configReplReadAfterReadOnly = true

  val fromDir_en = io.fromDir.TSHRID === io.tshrId

  val fromDir_DirRdArbComp = fromDir_en && io.fromDir.DirRdArbComp
  val fromDir_DirRdResp = fromDir_en && io.fromDir.DirRdResp
  val fromDir_DirWbArbComp = fromDir_en && io.fromDir.DirWbArbComp
  val fromDir_ReplRdArbComp = fromDir_en && io.fromDir.ReplRdArbComp
  val fromDir_ReplRdResp = fromDir_en && io.fromDir.ReplRdResp
  val fromDir_ReplRdRetryAck = fromDir_en && io.fromDir.ReplRdRetryAck

  //
  val tshr_inactive = io.tshr_inactivate || io.tshr_inactive

  // Directory read states
  val state_dirRead = RegInit(new DirReadFSM, DirReadFSM.init)
  val state_dirRead_next = WireInit(state_dirRead)

  state_dirRead := state_dirRead_next

  when (state_dirRead.NotYet) {

    /*
    1. 
    */
    whenOpt ()(io.read_en) {
      when (io.read_arbed) {
        // 1. [] -> DirRead_PostArb
        state_dirRead_next.PostArb := true.B
      }.otherwise {
        // 2. [] -> DirRead_PreArb
        state_dirRead_next.PreArb := true.B
      }
    }.elsewhenOpt (!configReplReadAfterReadOnly)(io.repl_en) {
      // 3. [] -> DirRead_ReplPreArb
      state_dirRead_next.ReplPreArb := true.B
    }
  }

  //when (state_dirRead.PreArb) {

    /*
    1. 

    *NOTICE: The outer "when" context of DirRead_PreArb state could be omitted, because 
             DirRdArbComp was always expected to be received only under DirRead_PreArb state.
             Ensure this always stands when adding state transitions under DirRead_PreArb.
    */
    when (fromDir_DirRdArbComp) {
      // 1. DirRead_PreArb -> DirRead_PostArb
      state_dirRead_next.PreArb := false.B
      state_dirRead_next.PostArb := true.B
    }
  //}

//when (state_dirRead.PostArb) {

    /*
    1.

    *NOTICE: The outer "when" context of DirRead_PostArb state could be omitted, becasue
             DirRdResp was always expected to be received only under DirRead_PostArb state.
             Ensure this always stands when adding state transitions under DirRead_PostArb.
    */
    when (fromDir_DirRdResp) {
      // 1. DirRead_PostArb -> DirRead_Done
      state_dirRead_next.PostArb := false.B
      state_dirRead_next.Done := true.B
    }
//}

  when (state_dirRead.Done) {
    
    /*
    1.
    */
    when (io.repl_en) {
      // 1. DirRead_Done -> DirRead_ReplPreArb
      state_dirRead_next.Done := false.B
      state_dirRead_next.ReplPreArb := true.B
    }
  }

//when (state_dirRead.ReplPreArb) {

    /* 
    1.

    *NOTICE: The outer "when" context of DirRead_ReplPreArb state could be omitted, because
             ReplRdArbComp was always expected to be received only under DirRead_ReplPreArb state.
             Ensure this always stands when adding state transitions under DirRead_ReplPreArb.
    */
    when (fromDir_ReplRdArbComp) {
      // 1. DirRead_ReplPreArb -> DirRead_ReplPostArb
      state_dirRead_next.ReplPreArb := false.B
      state_dirRead_next.ReplPostArb := true.B
    }
//}

//when (state_dirRead.ReplPostArb) {

    /*
    1.

    *NOTICE: The outer "when" context of DirRead_ReplPostArb state could be omitted, because
             ReplRdResp and ReplRdRetryAck were always expected to be received only under DirRead_ReplPostArb state.
             Ensure this always stands when adding state transitions under DirRead_ReplPostArb.
    */
    when (fromDir_ReplRdResp) {
      // 1. DirRead_ReplPostArb -> DirRead_ReplDone
      state_dirRead_next.ReplPostArb := false.B
      state_dirRead_next.ReplDone := true.B
    }
    when (fromDir_ReplRdRetryAck) {
      // 2. DirRead_ReplPostArb -> DirRead_ReplRetry
      state_dirRead_next.ReplPostArb := false.B
      state_dirRead_next.ReplRetry := true.B
    }
//}

  when (state_dirRead.ReplRetry) {

    /*
    1. 
    */
    when (io.repl_en) {
      // 1. DirRead_ReplRetry -> DirRead_ReplPreArb
      state_dirRead_next.ReplRetry := false.B
      state_dirRead_next.ReplPreArb := true.B
    }
  }

  when (state_dirRead.ReplDone) {

    /*
    1.
    */
    when (io.repl_reset) {
      // 1. DirRead_ReplDone -> DirRead_Done
      state_dirRead_next.ReplDone := false.B
      state_dirRead_next.Done := true.B
    }
  }

  when (io.tshr_dealloc) {
    state_dirRead_next := DirReadFSM.init
  }

  io.rd_idle := state_dirRead.NotYet
  io.rd_accept := io.toDir.DirRd && fromDir_DirRdArbComp
  io.rd_done := state_dirRead.Done || state_dirRead.Repl
  io.rd_done_flow := state_dirRead.Done || state_dirRead.Repl || fromDir_DirRdResp

  io.repl_idle := !state_dirRead.ReplPreArb && !state_dirRead.ReplPostArb
  io.repl_ready := state_dirRead.Done || state_dirRead.ReplRetry
  io.repl_accept := io.toDir.ReplRd && fromDir_ReplRdArbComp
  io.repl_done := state_dirRead.ReplDone && !io.repl_reset
  io.repl_retry := state_dirRead.ReplRetry

  assert(!(fromDir_DirRdArbComp && !state_dirRead.PreArb), "receiving DirRdArbComp on unexpected state (expecting PreArb)")
  assert(!(fromDir_DirRdResp && !state_dirRead.PostArb), "receiving DirRdResp on unexpected state (expecting PostArb)")
  assert(!(fromDir_ReplRdArbComp && !state_dirRead.ReplPreArb), "receiving ReplRdArbComp on unexpected state (expected ReplPreArb)")
  assert(!(fromDir_ReplRdResp && !state_dirRead.ReplPostArb), "receiving ReplRdResp on unexpected state (expected ReplPostArb)")
  assert(!(fromDir_ReplRdRetryAck && !state_dirRead.ReplPostArb), "receiving ReplRdRetryAck on unexpected state (expected ReplPostArb)")

  assert(!(fromDir_ReplRdResp && fromDir_ReplRdRetryAck), "ReplRdResp and ReplRdRetryAck must be exclusive to each other")

  assert(PopCount(state_dirRead.asUInt) <= 1.U, "multiple active states in DirReadFSM")

  val repl_reset_at_dir_resp = paramL2.hitFastPathDirRespFlow.B && fromDir_DirRdResp
  assert(!(io.repl_reset && !state_dirRead.Done && !state_dirRead.ReplDone && !repl_reset_at_dir_resp),
    "replace read state reset on inter-states")

  FSMPerfHistogram(s"L2TSHR_${id}_DirReadFSM_PreArb", state_dirRead.PreArb, state_dirRead_next.PreArb)
  FSMPerfHistogram(s"L2TSHR_${id}_DirReadFSM_PostArb", state_dirRead.PostArb, state_dirRead_next.PostArb)
  FSMPerfHistogram(s"L2TSHR_${id}_DirReadFSM_Done", state_dirRead.Done, state_dirRead_next.Done)
  FSMPerfHistogram(s"L2TSHR_${id}_DirReadFSM_ReplPreArb", state_dirRead.ReplPreArb, state_dirRead_next.ReplPreArb)
  FSMPerfHistogram(s"L2TSHR_${id}_DirReadFSM_ReplPostArb", state_dirRead.ReplPostArb, state_dirRead_next.ReplPostArb)
  FSMPerfHistogram(s"L2TSHR_${id}_DirReadFSM_ReplDone", state_dirRead.ReplDone, state_dirRead_next.ReplDone)
  FSMPerfHistogram(s"L2TSHR_${id}_DirReadFSM_ReplRetry", state_dirRead.ReplRetry, state_dirRead_next.ReplRetry)
  FSMPerfHistogram(s"L2TSHR_${id}_DirReadFSM_NotYet", state_dirRead.NotYet, state_dirRead_next.NotYet)

  FSMTransPerfHistogram(s"L2TSHR_${id}_DirReadFSM_Done_to_ReplPreArb", state_dirRead.Done, state_dirRead_next.ReplPreArb)
  FSMTransPerfHistogram(s"L2TSHR_${id}_DirReadFSM_Done_to_NotYet", state_dirRead.Done, state_dirRead_next.NotYet)

  // Directory write states
  val state_dirWrite = RegInit(new DirWriteFSM, DirWriteFSM.init)
  val state_dirWrite_next = WireInit(state_dirWrite)

  state_dirWrite := state_dirWrite_next

  val wb_trigger = (io.meta_modify || io.meta_modified.any || io.tag_modify || io.tag_modified || io.wb_aux) && !io.wb_cancel
  val wb_trigger_now = (io.meta_modify || io.tag_modify || io.wb_aux) && !io.wb_cancel

  if (configAggressiveWrite) {

    when (state_dirWrite.NotYet) {

      /*
      1.
      */
      when (io.wb_cancel) {
        // 1. [] -> DirWrite_Done
        state_dirWrite_next.Done := true.B
      }.elsewhen (wb_trigger) {
        // 2. [] -> DirWrite_PreArb
        state_dirWrite_next.PreArb := true.B
      }.elsewhen (io.tshr_valid && tshr_inactive) {
        // 3. [] -> DirWrite_Done
        state_dirWrite_next.Done := true.B
      }
    }

    when (state_dirWrite.PreArb) {

      /*
      1. 
      */
      when (fromDir_DirWbArbComp) {
        when (wb_trigger_now) {
          // 1. DirWrite_PreArb -> DirWrite_PreArb
        }.otherwise {
          // 2. DirWrite_PreArb -> DirWrite_Done
          state_dirWrite_next.PreArb := false.B
          state_dirWrite_next.Done := true.B
        }
      }.elsewhen (io.wb_cancel) {
        // 3. DirWrite_PreArb -> DirWrite_Done
        state_dirWrite_next.PreArb := false.B
        state_dirWrite_next.Done := true.B
      }
    }

    when (state_dirWrite.Done) {

      /*
      1. 
      */
      when (wb_trigger) {
        // 1. DirWrite_Done -> DirWrite_PreArb
        state_dirWrite_next.Done := false.B
        state_dirWrite_next.PreArb := true.B
      }
    }
  } else {

    when (state_dirWrite.NotYet) {

      /* 
      1.  
      */
      when (io.wb_cancel) {
        // 1. [] -> DirWrite_Done
        state_dirWrite_next.Done := true.B
      }.elsewhen (io.tshr_valid && (tshr_inactive || io.wb_aux)) {
        when (wb_trigger) {
        // 2. [] -> DirWrite_PreArb
        state_dirWrite_next.PreArb := true.B
        }.otherwise {
          // 3. [] -> DirWrite_Done
          state_dirWrite_next.Done := true.B
        }
      }
    }

    when (state_dirWrite.PreArb) {

      /*
      1.
      */
      when (fromDir_DirWbArbComp) {
        when (wb_trigger_now) {
          // 1. DirWrite_PreArb -> DirWrite_PreArb
        }.otherwise {
          // 2. DirWrite_PreArb -> DirWrite_Done
          state_dirWrite_next.PreArb := false.B
          state_dirWrite_next.Done := true.B
        }
      }.elsewhen (io.wb_cancel) {
        // 3. DirWrite_PreArb -> DirWrite_Done
        state_dirWrite_next.PreArb := false.B
        state_dirWrite_next.Done := true.B
      }
    }

    when (state_dirWrite.Done) {

      /*
      1.
      */
      when (wb_trigger) {
        when (io.wb_aux) {
          // 1. DirWrite_Done -> DirWrite_PreArb
          state_dirWrite_next.Done := false.B
          state_dirWrite_next.PreArb := true.B
        }.otherwise {
          // 2. DirWrite_Done -> []
          state_dirWrite_next.Done := false.B
        }
      }
    }
  }

  when (io.tshr_dealloc) {
    state_dirWrite := DirWriteFSM.init
  }

  io.wb_accept := io.toDir.DirWb && fromDir_DirWbArbComp
  io.wb_done := state_dirWrite.Done

  assert(!(fromDir_DirWbArbComp && !state_dirWrite.PreArb), "receiving DirWbArbComp on unexpected state (expecting PreArb)")
  assert(PopCount(state_dirWrite.asUInt) <= 1.U, "multiple active states in DirWriteFSM")

  val dirRelActive = WireDefault(false.B)
  val dirRelWay = WireDefault(0.U(wayBits.W))
  val dirRelPrefetch = WireDefault(false.B)
  val dirRelIdle = WireDefault(true.B)
  io.rel_idle := dirRelIdle

  // One pending release holds the exact way and acceptance-time provenance
  // until Directory completes its update. Back-to-back same-PA EVT releases are
  // excluded by the microarchitecture, so a second pulse is an assertion error.
  if (enableRRIPReleaseUpdate) {
    val fromDir_DirRelArbComp = fromDir_en && io.fromDir.DirRelArbComp
    val state_dirRel = RegInit(new DirRelFSM, DirRelFSM.init)
    val state_dirRel_next = WireInit(state_dirRel)
    val pendingWay = Reg(UInt(wayBits.W))
    val pendingLocalValid = Reg(Bool())
    val pendingPrefetch = Reg(Bool())

    state_dirRel := state_dirRel_next

    when (io.repl_release.get) {
      assert(state_dirRel.NotYet || state_dirRel.Done,
        "back-to-back same-PA EVT release while DirRel is pending")
      state_dirRel_next.PreArb := true.B
      state_dirRel_next.Done := false.B
      pendingWay := io.repl_way.get
      pendingLocalValid := io.rd_done || io.meta_modified.prefetch
      pendingPrefetch := io.meta.prefetch
    }

    when (state_dirRel.PreArb && fromDir_DirRelArbComp) {
      state_dirRel_next.PreArb := false.B
      state_dirRel_next.Done := true.B
    }

    when (io.tshr_dealloc) {
      state_dirRel_next := DirRelFSM.init
    }

    dirRelIdle := state_dirRel.NotYet || state_dirRel.Done
    dirRelActive := state_dirRel.PreArb
    dirRelWay := pendingWay
    dirRelPrefetch := pendingPrefetch
    io.toDir.DirRel := state_dirRel.PreArb &&
      !(state_dirRead.PreArb || state_dirRead.ReplPreArb || state_dirWrite.PreArb)
    io.toDir.RRIP_LOCAL_VALID := state_dirRel.PreArb && pendingLocalValid

    assert(!(fromDir_DirRelArbComp && !state_dirRel.PreArb), "receiving DirRelArbComp on unexpected state (expecting PreArb)")
    assert(PopCount(state_dirRel.asUInt) <= 1.U, "multiple active states in DirRelFSM")
  } else {
    io.toDir.DirRel := false.B
    io.toDir.RRIP_LOCAL_VALID := false.B
  }

  // interactions with Directory
  io.toDir.TSHRID := io.tshrId
  io.toDir.PADDR := io.tshr_paddr
  io.toDir.WAY := Mux(dirRelActive, dirRelWay, io.meta_way)
  io.toDir.META := io.meta
  io.toDir.META.prefetch := Mux(dirRelActive, dirRelPrefetch, io.meta.prefetch)
  io.toDir.META_WEN := io.meta_modified
  io.toDir.META_WEN.prefetch := io.meta_modified.prefetch && !dirRelActive
  io.toDir.TAG_WEN := io.tag_modified
  io.toDir.RRIP_UPDATE := io.rrip_update
  io.toDir.RRIP_PREFETCH := io.rrip_prefetch
  io.toDir.RRIP_REFILL_PREFETCH := io.rrip_refill_prefetch
  io.toDir.ReplLock := io.repl_lock_req
  io.toDir.REPL_LOCK_WAY := io.repl_lock_way
  io.toDir.REPL_LOCK_RELEASE := io.repl_lock_release
  io.repl_lock_conflict := fromDir_en && io.fromDir.ReplLockConflict
  io.toDir.DirRd := state_dirRead.PreArb
  val releaseWriteSettling = io.repl_lock_release &&
    (io.meta_modify || io.tag_modify || io.snoop_active)
  io.toDir.DirWb := state_dirWrite.PreArb && !io.wb_locked && !releaseWriteSettling
  io.toDir.ReplRd := state_dirRead.ReplPreArb

  when (io.repl_lock_release && io.toDir.DirWb) {
    assert(!(io.meta_modify || io.tag_modify),
      "ReplLock release before local tag/meta settled")
    assert(io.toDir.WAY === io.toDir.REPL_LOCK_WAY, "ReplLock release way mismatch")
  }

  assert(!(io.repl_lock_release && io.wb_cancel),
    "ReplLock release cannot be canceled")

  assert(PopCount(Seq(io.toDir.DirRd, io.toDir.DirWb, io.toDir.ReplRd, io.toDir.DirRel)) <= 1.U,
    "DirRd, DirWb, ReplRd and DirRel operations to Directory overlapped")

  FSMPerfHistogram(s"L2TSHR_${id}_DirWriteFSM_PreArb", state_dirWrite.PreArb, state_dirWrite_next.PreArb)
  FSMPerfHistogram(s"L2TSHR_${id}_DirWriteFSM_Done", state_dirWrite.Done, state_dirWrite_next.Done)
  FSMPerfHistogram(s"L2TSHR_${id}_DirWriteFSM_NotYet", state_dirWrite.NotYet, state_dirWrite_next.NotYet)
}
