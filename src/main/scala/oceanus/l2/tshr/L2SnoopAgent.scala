package oceanus.l2.tshr

import chisel3._
import chisel3.util._
import oceanus.compactchi._
import oceanus.l2._
import org.chipsalliance.cde.config.Parameters

object SnoopOpcodeDerive {
  object Intent extends ChiselEnum {
    val SnpMakeInvalid, SnpToInvalid, SnpToShared, SnpToClean = Value
  }

  final case class Result(needSnoop: Boolean, opcode: Int)

  private def isInvalidate(intent: Intent.Type): Bool =
    intent === Intent.SnpMakeInvalid || intent === Intent.SnpToInvalid

  private def isInvalidate(intent: Int): Boolean =
    intent == Intent.SnpMakeInvalid.litValue.toInt || intent == Intent.SnpToInvalid.litValue.toInt

  // The uop encodes the caller's intent; the emitted SNP opcode is derived here
  // from the directory snapshot. Directory dirty is intentionally excluded:
  // L2 dirty belongs to DS ownership, while the directory cannot know L1 dirty.
  def apply(intent: Intent.Type, state: UInt, clients: UInt, hit: Bool, aliasMismatch: Bool): (Bool, UInt) = {
    val hasClient = clients.orR
    val invalidate = isInvalidate(intent) || aliasMismatch
    val needSnoop = hit && hasClient && (invalidate || state === L2Directory.MetaState.UU)
    // A true SnpMakeInvalid intent (L3 SnpMakeInvalid / CMO MakeInvalid) must keep
    // the CCHI SnpMakeInvalid opcode even on a UU line: the dirty data is
    // contractually dead (full-line overwrite flow) and the SNP vPipe forbids any
    // data return for MakeInvalid. Only the other invalidate paths use the UU ->
    // SnpToInvalid substitution to pull dirty data back from L1.
    val opcode = Mux(
      invalidate,
      Mux(
        intent === Intent.SnpMakeInvalid,
        CCHIOpcode.SnpMakeInvalid.U,
        Mux(state === L2Directory.MetaState.UU, CCHIOpcode.SnpToInvalid.U, CCHIOpcode.SnpMakeInvalid.U)
      ),
      CCHIOpcode.SnpToShared.U
    )

    (needSnoop, opcode)
  }

  def apply(intent: Int, state: Int, clients: Int, hit: Boolean, aliasMismatch: Boolean): Result = {
    val hasClient = clients != 0
    val invalidate = isInvalidate(intent) || aliasMismatch
    val needSnoop = hit && hasClient && (invalidate || state == L2Directory.MetaState.UU.litValue.toInt)
    val opcode =
      if (invalidate) {
        if (intent == Intent.SnpMakeInvalid.litValue.toInt) {
          // MakeInvalid contract: data is dead, never substitute SnpToInvalid
          CCHIOpcode.SnpMakeInvalid.opcode
        } else if (state == L2Directory.MetaState.UU.litValue.toInt) {
          CCHIOpcode.SnpToInvalid.opcode
        } else {
          CCHIOpcode.SnpMakeInvalid.opcode
        }
      } else {
        CCHIOpcode.SnpToShared.opcode
      }

    Result(needSnoop, opcode)
  }
}

object L2SnoopAgent {

  class PathToSnoopAgentUOPs extends Bundle {
    val SnpMakeInvalid = Bool()
    val SnpToInvalid = Bool()
    val SnpToShared = Bool()
    val SnpToClean = Bool()

    val SnpCompAck = Bool()
  }

  class PathFromSnoopAgentUOPs extends Bundle {
    val SnpResp = Bool()
    val SnpRespData0 = Bool()
    val SnpRespData2 = Bool()
  }

  class PathToSnoopAgent extends PathToSnoopAgentUOPs {
    val CLIENTS = Vec(1, Bool())  
    val ALIAS = UInt(2.W)
    val isL2Evict = Bool()
  }

  class PathFromSnoopAgent extends PathFromSnoopAgentUOPs {
    val PASSDIRTY = Bool()
    val RESP = UInt(3.W)
  }
}

class L2SnoopAgent(val sliceNum: Int)(implicit val p: Parameters) extends Module with HasL2Params {

  val io = IO(new Bundle {
    val consts = Input(new L2TSHRConsts(sliceNum))

    val uopFromSNP = Flipped(Valid(new L2SnoopAgent.PathToSnoopAgent))
    val uopFromREQ = Flipped(Valid(new L2SnoopAgent.PathToSnoopAgent))

    val tshr_paddr = Input(UInt(paramL2.physicalAddrWidth.W))
    val tshr_dirResult = Input(new L2Directory.MetaReadResult)

    val txSnp = Decoupled(new FlitSNP)

    val UpRXRSP = Flipped(Valid(new FlitUpRSP))
    val UpRXDAT = Flipped(Valid(new FlitUpDAT))

    val snoop_active = Output(Bool())
    val tshr_meta_write_en = Output(new L2Directory.MetaWriteMask)
    val tshr_meta_write_meta = Output(new L2Directory.Meta)

    val fromSA = Output(new L2SnoopAgent.PathFromSnoopAgent)
    val fromSAForSNP = Output(new L2SnoopAgent.PathFromSnoopAgent)
    val fromSAForREQ = Output(new L2SnoopAgent.PathFromSnoopAgent)
    val dbg = Output(new Bundle {
      val slotSNPValid = Bool()
      val slotREQValid = Bool()
      val slotSNPEvict = Bool()
      val slotREQEvict = Bool()
      val slotSNPOp = UInt(4.W)
      val slotREQOp = UInt(4.W)
      val acceptSNP = Bool()
      val acceptREQ = Bool()
      val startService = Bool()
      val startFromSNP = Bool()
      val startFromREQ = Bool()
      val serviceFromSNP = Bool()
      val serviceFromREQ = Bool()
      val state = UInt(2.W)
    })
  })

  val sIdle :: sSnpReq :: sWaitCore :: Nil = Enum(3)
  val state = RegInit(sIdle)

  val busy = RegInit(false.B)
  val armedSNP = RegInit(true.B)
  val armedREQ = RegInit(true.B)
  val holdCountSNP = RegInit(0.U(3.W))
  val holdCountREQ = RegInit(0.U(3.W))

  val slotSNPValid = RegInit(false.B)
  val slotREQUop = RegInit(0.U.asTypeOf(new L2SnoopAgent.PathToSnoopAgent))
  val slotSNPUop = RegInit(0.U.asTypeOf(new L2SnoopAgent.PathToSnoopAgent))
  val slotREQAlias = RegInit(0.U(2.W))
  val slotSNPAlias = RegInit(0.U(2.W))
  val slotREQValid = RegInit(false.B)
  val serviceAlias = RegInit(0.U(2.W))
  val servicePaddr = Reg(UInt(paramL2.physicalAddrWidth.W))
  val serviceOpcode = Reg(UInt(2.W))
  val serviceFromSNP = RegInit(false.B)
  val serviceFromREQ = RegInit(false.B)
  val servicePreserveClient = RegInit(false.B)

  val seenData0 = RegInit(false.B)
  val passDirtyReg = RegInit(false.B)

  val uopValids = Seq(io.uopFromSNP.valid, io.uopFromREQ.valid)
  val uopAnyValid = uopValids.reduce(_ || _)

  def intentOf(uop: L2SnoopAgent.PathToSnoopAgent): SnoopOpcodeDerive.Intent.Type = {
    val intent = Wire(SnoopOpcodeDerive.Intent())
    intent := SnoopOpcodeDerive.Intent.SnpToShared
    when (uop.SnpMakeInvalid) {
      intent := SnoopOpcodeDerive.Intent.SnpMakeInvalid
    }.elsewhen (uop.SnpToInvalid) {
      intent := SnoopOpcodeDerive.Intent.SnpToInvalid
    }.elsewhen (uop.SnpToClean) {
      intent := SnoopOpcodeDerive.Intent.SnpToClean
    }
    intent
  }

  def uopBits(uop: L2SnoopAgent.PathToSnoopAgent): Seq[Bool] = Seq(
    uop.SnpMakeInvalid,
    uop.SnpToInvalid,
    uop.SnpToShared,
    uop.SnpToClean
  )

  val acceptSNP = io.uopFromSNP.valid && armedSNP
  val acceptREQ = io.uopFromREQ.valid && armedREQ

  val chooseSlotREQEvict = slotREQValid && slotREQUop.isL2Evict
  val chooseSlotSNP = slotSNPValid && !chooseSlotREQEvict
  val chooseSlotREQ = slotREQValid && !chooseSlotSNP
  val startService = state === sIdle && !busy && (chooseSlotREQEvict || chooseSlotSNP || chooseSlotREQ)
  io.snoop_active := io.uopFromREQ.valid || io.uopFromSNP.valid ||
    slotREQValid || slotSNPValid || startService || busy || (state =/= sIdle)
  val startUop = Wire(new L2SnoopAgent.PathToSnoopAgent)
  startUop := Mux(chooseSlotSNP, slotSNPUop, slotREQUop)
  val startFromSNP = startService && chooseSlotSNP
  val startFromREQ = startService && chooseSlotREQ
  val startAlias = Mux(chooseSlotSNP, slotSNPAlias, slotREQAlias)
  val aliasMismatch = io.tshr_dirResult.hit &&
    io.tshr_dirResult.clients.asUInt.orR &&
    startUop.CLIENTS.asUInt.orR &&
    (io.tshr_dirResult.alias =/= startUop.ALIAS)
  when (startService) {
    servicePreserveClient := startFromREQ && aliasMismatch
  }
  val (startNeedSnoop, startOpcode) =
    SnoopOpcodeDerive(intentOf(startUop), io.tshr_dirResult.state, io.tshr_dirResult.clients.asUInt, io.tshr_dirResult.hit, aliasMismatch)

  val rspMatch =
    state === sWaitCore &&
      io.UpRXRSP.valid &&
      io.UpRXRSP.bits.TxnID === io.consts.tshrId &&
      io.UpRXRSP.bits.Opcode === CCHIOpcode.SnpResp.U

  val datMatch =
    state === sWaitCore &&
      io.UpRXDAT.valid &&
      io.UpRXDAT.bits.TxnID === io.consts.tshrId &&
      io.UpRXDAT.bits.Opcode === CCHIOpcode.SnpRespData.U

  val datBeat0 = datMatch && io.UpRXDAT.bits.DataID === 0.U
  // Upstream RXDAT DataID carries a packed beat index ({0,1} for the two 256-bit halves), not the CHI chunk index {0,2}
  val datBeat2 = datMatch && io.UpRXDAT.bits.DataID === 1.U

  val done = rspMatch || (datBeat2 && seenData0)
  val invalidatingService = serviceOpcode === CCHIOpcode.SnpMakeInvalid.U || serviceOpcode === CCHIOpcode.SnpToInvalid.U
  // Alias back-invalidation preserves ownership through the subsequent grant; other invalidating completions clear it.
  val clear_client_en = done && invalidatingService && !servicePreserveClient
  io.tshr_meta_write_en := 0.U.asTypeOf(new L2Directory.MetaWriteMask)
  io.tshr_meta_write_en.clients(0) := clear_client_en
  io.tshr_meta_write_meta := 0.U.asTypeOf(new L2Directory.Meta)
  io.tshr_meta_write_meta.clients(0) := false.B
  val directDone = startService && !startNeedSnoop
  val doneForSNP = serviceFromSNP
  val doneForREQ = serviceFromREQ
  val directForSNP = directDone && startFromSNP
  val directForREQ = directDone && startFromREQ
  val dataPassDirty = io.UpRXDAT.bits.Resp(2)
  val pdValid = rspMatch || datBeat0 || datBeat2

  val passDirty = Mux(
    rspMatch,
    io.UpRXRSP.bits.Resp(2),
    Mux(datBeat0 || datBeat2, dataPassDirty, false.B)
  )

  val sa_resp = Mux(rspMatch, io.UpRXRSP.bits.Resp,
                Mux(datBeat0 || datBeat2, io.UpRXDAT.bits.Resp, CCHIResp.I.U))

  io.fromSAForSNP.SnpResp := (rspMatch && doneForSNP) || directForSNP
  io.fromSAForSNP.SnpRespData0 := datBeat0 && serviceFromSNP
  io.fromSAForSNP.SnpRespData2 := datBeat2 && serviceFromSNP
  io.fromSAForSNP.PASSDIRTY := doneForSNP && pdValid && passDirty
  io.fromSAForSNP.RESP := sa_resp

  io.fromSAForREQ.SnpResp := (rspMatch && doneForREQ) || directForREQ
  io.fromSAForREQ.SnpRespData0 := datBeat0 && serviceFromREQ
  io.fromSAForREQ.SnpRespData2 := datBeat2 && serviceFromREQ
  io.fromSAForREQ.PASSDIRTY := doneForREQ && pdValid && passDirty
  io.fromSAForREQ.RESP := sa_resp

  io.fromSA.SnpResp := io.fromSAForSNP.SnpResp || io.fromSAForREQ.SnpResp
  io.fromSA.SnpRespData0 := io.fromSAForSNP.SnpRespData0 || io.fromSAForREQ.SnpRespData0
  io.fromSA.SnpRespData2 := io.fromSAForSNP.SnpRespData2 || io.fromSAForREQ.SnpRespData2
  io.fromSA.PASSDIRTY := io.fromSAForSNP.PASSDIRTY || io.fromSAForREQ.PASSDIRTY
  io.fromSA.RESP := sa_resp

  io.dbg.slotSNPValid := slotSNPValid
  io.dbg.slotREQValid := slotREQValid
  io.dbg.slotSNPEvict := slotSNPUop.isL2Evict
  io.dbg.slotREQEvict := slotREQUop.isL2Evict
  io.dbg.slotSNPOp := Cat(slotSNPUop.SnpMakeInvalid, slotSNPUop.SnpToInvalid, slotSNPUop.SnpToShared, slotSNPUop.SnpToClean)
  io.dbg.slotREQOp := Cat(slotREQUop.SnpMakeInvalid, slotREQUop.SnpToInvalid, slotREQUop.SnpToShared, slotREQUop.SnpToClean)
  io.dbg.acceptSNP := acceptSNP
  io.dbg.acceptREQ := acceptREQ
  io.dbg.startService := startService
  io.dbg.startFromSNP := startFromSNP
  io.dbg.startFromREQ := startFromREQ
  io.dbg.serviceFromSNP := serviceFromSNP
  io.dbg.serviceFromREQ := serviceFromREQ
  io.dbg.state := state

  io.txSnp.valid := state === sSnpReq
  io.txSnp.bits := 0.U.asTypeOf(new FlitSNP)
  io.txSnp.bits.SrcID := io.consts.sliceNID // RN echoes SrcID into SnpResp.TgtID; must be our slice NID for L2Top demux
  io.txSnp.bits.TgtID := io.consts.clientNID // target the (single) coherent upstream client; L2Top demuxes DnSNP on TgtID
  io.txSnp.bits.TxnID := io.consts.tshrId
  io.txSnp.bits.Opcode := serviceOpcode
  io.txSnp.bits.Addr := servicePaddr >> 3
  io.txSnp.bits.alias := serviceAlias

  when (!io.uopFromSNP.valid) {
    armedSNP := true.B
    holdCountSNP := 0.U
  }.elsewhen (!armedSNP && holdCountSNP =/= 7.U) {
    holdCountSNP := holdCountSNP + 1.U
  }
  when (!io.uopFromREQ.valid) {
    armedREQ := true.B
    holdCountREQ := 0.U
  }.elsewhen (!armedREQ && holdCountREQ =/= 7.U) {
    holdCountREQ := holdCountREQ + 1.U
  }

  when (acceptSNP) {
    slotSNPUop := io.uopFromSNP.bits
    slotSNPAlias := io.tshr_dirResult.alias
    slotSNPValid := true.B
    armedSNP := false.B
  }
  when (acceptREQ) {
    slotREQUop := io.uopFromREQ.bits
    slotREQAlias := io.tshr_dirResult.alias
    slotREQValid := true.B
    armedREQ := false.B
  }

  when (datBeat0) {
    seenData0 := true.B
    passDirtyReg := io.UpRXDAT.bits.Resp(2)
  }

  when (done) {
    busy := false.B
    seenData0 := false.B
    passDirtyReg := false.B
    serviceFromSNP := false.B
    serviceFromREQ := false.B
    state := sIdle
  }.elsewhen (directDone) {
    when (startFromSNP) { slotSNPValid := false.B }
    when (startFromREQ) { slotREQValid := false.B }
  }.elsewhen (startService && startNeedSnoop) {
    busy := true.B
    serviceFromSNP := startFromSNP
    serviceFromREQ := startFromREQ
    serviceAlias := startAlias
    servicePaddr := io.tshr_paddr
    serviceOpcode := startOpcode
    when (startFromSNP) { slotSNPValid := false.B }
    when (startFromREQ) { slotREQValid := false.B }
    state := sSnpReq
  }

  switch (state) {
    is (sSnpReq) {
      when (io.txSnp.fire) {
        state := sWaitCore
      }
    }
  }

  when (acceptSNP) {
    assert(!slotSNPValid,
      s"L2SnoopAgent @ %m: SNP slot accepted a new uop while occupied")
    assert(PopCount(uopBits(io.uopFromSNP.bits)) === 1.U,
      s"L2SnoopAgent @ %m: accepted invalid or multi-hot SNP uop")
  }
  when (acceptREQ) {
    assert(!slotREQValid,
      s"L2SnoopAgent @ %m: REQ slot accepted a new uop while occupied")
    assert(PopCount(uopBits(io.uopFromREQ.bits)) === 1.U,
      s"L2SnoopAgent @ %m: accepted invalid or multi-hot REQ uop")
  }
  when (startService) {
    assert(io.tshr_dirResult.state === L2Directory.MetaState.I ||
           io.tshr_dirResult.state === L2Directory.MetaState.S ||
           io.tshr_dirResult.state === L2Directory.MetaState.US ||
           io.tshr_dirResult.state === L2Directory.MetaState.UU,
      s"L2SnoopAgent @ %m: started service with illegal directory state")
    assert(io.tshr_dirResult.hit || !startNeedSnoop,
      s"L2SnoopAgent @ %m: missed directory line requested an upstream snoop")
  }

  assert(!(rspMatch && datMatch),
    s"L2SnoopAgent @ %m: matched dataless and data response in the same cycle")
  assert(PopCount(Seq(serviceFromSNP, serviceFromREQ)) <= 1.U,
    s"L2SnoopAgent @ %m: service latched multiple sources")
  assert(!busy || PopCount(Seq(serviceFromSNP, serviceFromREQ)) === 1.U,
    s"L2SnoopAgent @ %m: service did not retain its source")
  assert(!(datBeat2 && !seenData0),
    s"L2SnoopAgent @ %m: SnpRespData DataID 2 arrived before DataID 0")
  assert(!(datBeat2 && passDirtyReg =/= io.UpRXDAT.bits.Resp(2)),
    "L2SnoopAgent: SnpRespData PassDirty changed between data beats")
  assert(!(io.uopFromSNP.valid && !armedSNP && holdCountSNP >= 4.U),
    s"L2SnoopAgent @ %m: SNP uop input valid stayed high after acceptance; deassert before reusing the request")
  assert(!(io.uopFromREQ.valid && !armedREQ && holdCountREQ >= 4.U),
    s"L2SnoopAgent @ %m: REQ uop input valid stayed high after acceptance; deassert before reusing the request")

}
