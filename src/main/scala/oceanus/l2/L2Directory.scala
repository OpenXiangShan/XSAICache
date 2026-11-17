package oceanus.l2

import chisel3._
import chisel3.util._
import utility._
import utility.sram.SRAMTemplate
import xscache.coupledL2.utils.SplittedSRAM
import oceanus.l2._
import oceanus.compactchi._
import org.chipsalliance.cde.config.{Field, Parameters}
import xscache.oceanus.compactchi.HasCCHIParameters

case object L2DirectoryBlockRefillFaultModeKey extends Field[Int](0)

object L2DirectoryBlockRefillFaultMode {
  val NeverUnlock = 2
}

object L2Directory {

  // Refill lock table entry: one slot per TSHR (indexed by TSHR ID), holding the
  // victim (set,way) picked by that TSHR's in-flight refill until its fill
  // commit lands. A TSHR holds at most one outstanding lock by construction
  // (repl_reset = rxreq_fire: the replace FSM resets only when a new request
  // enters the TSHR, after the previous refill committed).
  // hitHold marks a reservation transferred from that refill, or created by an
  // unsatisfied initial-hit ReadUnique, MakeUnique, or StashUnique, until its
  // final allocating DirWb commits.
  class RefillLockEntry(implicit val p: Parameters) extends Bundle with HasL2Params {
    val valid = Bool()
    val set = UInt(setBits.W)
    val way = UInt(wayBits.W)
    val hitHold = Bool()
  }

  object MetaState {

    def apply() = UInt(2.W)

    def UU: UInt = 3.U(2.W)
    def US: UInt = 2.U(2.W)
    def S: UInt = 1.U(2.W)
    def I: UInt = 0.U(2.W)

    def atLeastS(state: UInt): Bool = state >= MetaState.S
    def atLeastU(state: UInt): Bool = state >= MetaState.US

    def isClientU(state: UInt, client: Bool) = {
      Mux(state === UU, client, false.B)
    }

    def isClientS(state: UInt, client: Bool) = {
      Mux(state === US || state === S, client, false.B)
    }

    def toLocal(state: UInt): L2CacheLocalState = {
      val value = Wire(new L2CacheLocalState)
      value.bits := ParallelLookUp(state, Seq(
        (UU, L2CacheStates.U),
        (US, L2CacheStates.U),
        (S , L2CacheStates.S),
        (I , L2CacheStates.I)
      ))
      value
    }
  }

  // Meta: coherence state (NO tag — tag is internal to Directory)
  class Meta(implicit val p: Parameters) extends Bundle with HasL2Params with HasCCHIParameters {
    val state = MetaState()
    val dirty = Bool()
    val clients = Vec(1, Bool()) // TODO: parameterize with coherent l2 client count
    val alias = UInt(paramCCHI.TagAlias_Width.W)
    val prefetch = Bool()
  }

  class MetaReadResult(implicit override val p: Parameters) extends Meta with HasL2Params {
    val way = UInt(wayBits.W)
    val hit = Bool() // DirRd tag hit
  }

  class MetaWriteMask(implicit val p: Parameters) extends Bundle with HasL2Params {
    val state = Bool()
    val dirty = Bool()
    val clients = Vec(1, Bool()) // TODO: parameterize with coherent l2 client count
    val alias = Bool()
    val prefetch = Bool()

    def any = this.asUInt.orR

    def maskAndWrite(dst: Meta, src: Meta): Unit = {
      when (state) { dst.state := src.state }
      when (dirty) { dst.dirty := src.dirty }
      when (alias) { dst.alias := src.alias }
      when (prefetch) { dst.prefetch := src.prefetch }
      clients.zip(dst.clients.zip(src.clients)).foreach { case (mask, (dst, src)) => when (mask) { dst := src } }
    }

    def maskAndWrite(dst: Meta, dstFlag: MetaWriteMask, src: Meta): Unit = {
      maskAndWrite(dst, src)
      when (state) { dstFlag.state := true.B }
      when (dirty) { dstFlag.dirty := true.B }
      when (alias) { dstFlag.alias := true.B }
      when (prefetch) { dstFlag.prefetch := true.B }
      clients.zip(dstFlag.clients).foreach { case (mask, dst) => when (mask) { dst := true.B } }
    }

    def unmaskAndWrite(dst: Meta, src: Meta): Unit = {
      when (!state) { dst.state := src.state }
      when (!dirty) { dst.dirty := src.dirty }
      when (!alias) { dst.alias := src.alias }
      when (!prefetch) { dst.prefetch := src.prefetch }
      clients.zip(dst.clients.zip(src.clients)).foreach { case (mask, (dst, src)) => when (!mask) { dst := src } }
    }
  }

  object MetaWriteMask {
    def empty(implicit p: Parameters): MetaWriteMask = 0.U.asTypeOf(new MetaWriteMask)
  }

  class ReplReadResult(implicit override val p: Parameters) extends Bundle with HasL2Params {
    val paddr = UInt(paramL2.physicalAddrWidth.W) // victim PA; valid when meta.state =/= I
    val way = UInt(wayBits.W)
    val meta = new L2Directory.Meta
  }

  // A valid entry represents one active set and caches its complete replacement
  // row. RRIP entries hold two RRPV bits per way; full-SRAM mode also holds one
  // origin bit per way. ownerCount tracks outstanding refill locks only: DirRd
  // hits and DirRel updates share an entry without becoming owners.
  class ReplCAMEntry(val replBits: Int, val includeOrigin: Boolean)(implicit val p: Parameters)
      extends Bundle with HasL2Params {
    val valid = Bool()
    val set = UInt(setBits.W)
    val repl = UInt(replBits.W)
    val origin = if (includeOrigin) Some(UInt(paramL2.ways.W)) else None
    val ownerCount = UInt(log2Ceil(paramL2.ways + 1).W)
  }

  class PathToDirectoryUOPs extends Bundle {
    val DirRd = Bool()
    val DirWb = Bool()
    val ReplRd = Bool()
    val DirRel = Bool() // DRRIP release-type update, issued at EVT vPipe acceptance
    val ReplLock = Bool() // Fire-and-forget hit-way reservation, outside normal arbitration
    val ReplLockCandidate = Bool()
  }

  class PathToDirectory(implicit val p: Parameters) extends PathToDirectoryUOPs with HasL2Params {
    val TSHRID = UInt(mshrIndexWidth.W)
    val PADDR = UInt(paramL2.physicalAddrWidth.W)
    val WAY = UInt(wayBits.W)
    val META = new L2Directory.Meta // only applicable for DirWb
    val META_WEN = new L2Directory.MetaWriteMask // only applicable for DirWb
    val TAG_WEN = Bool()
    val RRIP_UPDATE = Bool()
    val RRIP_PREFETCH = Bool()
    val RRIP_REFILL_PREFETCH = Bool()
    val RRIP_LOCAL_VALID = Bool() // DirRel: use META.prefetch from the latest TSHR-local copy
    val REPL_LOCK_WAY = UInt(wayBits.W)
    val REPL_LOCK_RELEASE = Bool()
  }

  class PathFromDirectoryUOPs extends Bundle {
    val DirRdArbComp = Bool()
    val DirRdResp = Bool()
    val DirWbArbComp = Bool()
    val ReplRdArbComp = Bool()
    val ReplRdResp = Bool()
    val ReplRdRetryAck = Bool()
    val DirRelArbComp = Bool()
    val ReplLockConflict = Bool()
  }

  class PathFromDirectory(implicit val p: Parameters) extends PathFromDirectoryUOPs with HasL2Params {
    val TSHRID = UInt(mshrIndexWidth.W)
    val META = new L2Directory.MetaReadResult // only applicable for DirRd
    val REPL = new L2Directory.ReplReadResult // only applicable for ReplRd
  }

  // Directory entry assembled from the split tag and meta arrays
  class DirEntry(implicit val p: Parameters) extends Bundle with HasL2Params {
    val tag = UInt(tagWidth.W)
    val meta = new L2Directory.Meta
  }
}

// ══════════════════════════════════════════════════════════════════════
// Tree-PLRU for 4-way (3 bits/set)
// ══════════════════════════════════════════════════════════════════════

object PLRU4 {
  val nBits = 3
  def getReplaceWay(state: UInt): UInt = {
    val way = WireDefault(0.U(2.W))
    when(state(0) === 0.U) { way := Mux(state(1) === 0.U, 0.U, 1.U) }
      .otherwise           { way := Mux(state(2) === 0.U, 2.U, 3.U) }
    way
  }
  def getNextState(state: UInt, touchWay: UInt): UInt = {
    val next = Wire(Vec(nBits, Bool()))
    next := state.asTypeOf(Vec(nBits, Bool()))
    val inLeft = touchWay(1) === 0.U
    val first  = touchWay(0) === 0.U
    next(0) := Mux(inLeft, true.B, false.B)
    when(inLeft) { next(1) := Mux(first, true.B, false.B) }
      .otherwise { next(2) := Mux(first, true.B, false.B) }
    next.asUInt
  }
}

// ══════════════════════════════════════════════════════════════════════
// Directory (with integrated arbiter)
// ══════════════════════════════════════════════════════════════════════
//
// Each TSHR connects one-to-one: io.toDir(i) / io.fromDir(i).
// The normal group retains ReplRd > DirRd > DirWb with starvation protection
// and round-robin within a group. DirRel class priority is configurable:
// dirrel-high, round-robin, or dirrel-low; high/RR hold the R/W turn.
// Normal winners enter a 4-stage pipeline: S1 decode and request registration,
// S2 SRAM request, S3 response arrival/capture, and S4 hit/way/result/update using
// the registered response. RRIP SRAM mode caches active sets in a TSHR-sized CAM;
// zero-owner entries write back in exclusive Directory slots.
//
// ArbComp: DirRd/ReplRd/DirWb completion is asserted at the grant cycle.
// Responses are driven at S4 and routed one-to-one to fromDir(s4_tshr).
// DirRel instead completes at its W cycle, after replacement and metadata writes.
//
// replStall protects the FF S4-write/S1-read window for both ReplRd and DirRd.
// SRAM mode uses CAM-entry initialization, direct-backing, and CAM-commit
// interlocks instead where possible.

class Directory(implicit val p: Parameters) extends Module with HasL2Params {
  val MAX_AGE = 15.U(4.W)
  val idxW    = log2Ceil(nMSHR)
  val useRRIP = Seq("drrip", "srrip", "brrip").contains(paramL2.replacement)
  val useDRRIP = paramL2.replacement == "drrip"
  val useRRIPRelease = enableRRIPReleaseUpdate
  val rrip = if (useRRIP) Some(oceanus.l2.ReplacementPolicy.fromString(paramL2.replacement, paramL2.ways)) else None
  // RRIP uses 2 bits/way; PLRU uses the low 3 of the existing register width.
  val replBits = if (useRRIP) rrip.get.nBits else 2 * paramL2.ways
  private val enableDirStateProbe = p(L2SliceDirStateProbeEnableKey)
  private val enableDirPlruProbe = p(L2SliceDirPlruProbeEnableKey)
  private val blockRefillFaultMode = p(L2DirectoryBlockRefillFaultModeKey).U(2.W)

  val io = IO(new Bundle {
    val toDir   = Input(Vec(nMSHR, new L2Directory.PathToDirectory))
    val fromDir = Output(Vec(nMSHR, new L2Directory.PathFromDirectory))
    val tshrIdle = Input(Vec(nMSHR, Bool()))
    val debugStateWrite = if (enableDirStateProbe) {
      Some(Output(new Bundle {
        val valid = Bool()
        val set = UInt(setBits.W)
        val way = UInt(wayBits.W)
        val state = L2Directory.MetaState()
        val dirty = Bool()
        val client0 = Bool()
      }))
    } else {
      None
    }
    val debugRrip = if (enableDirPlruProbe) {
      Some(Output(new Bundle {
        val valid = Bool()
        val set = UInt(setBits.W)
        val state = UInt(replBits.W)
        val s4Valid = Bool()
        val s4Tshr = UInt(mshrIndexWidth.W)
        val s4IsDirRd = Bool()
        val s4IsRepl = Bool()
        val wayS4 = UInt(wayBits.W)
        val s4Rrip = UInt(replBits.W)
        val currentSetRrip = UInt(replBits.W)
      }))
    } else {
      None
    }
    val debugReplCAM = if (enableDirPlruProbe) {
      Some(Output(Vec(nMSHR, new L2Directory.ReplCAMEntry(replBits, true))))
    } else {
      None
    }
    val debugRetry = if (enableDirPlruProbe) {
      Some(Output(new Bundle {
        val replStall = Bool()
        val s2Valid = Bool()
        val s2Tshr = UInt(log2Ceil(paramL2.mshrSize).W)
        val s2IsDirRd = Bool()
        val s2IsRepl = Bool()
        val s2Set = UInt(setBits.W)
        val s4Valid = Bool()
        val s4Tshr = UInt(log2Ceil(paramL2.mshrSize).W)
        val s4IsDirRd = Bool()
        val s4IsRepl = Bool()
        val s4Set = UInt(setBits.W)
        val hasInvalid = Bool()
        val invalidWay = UInt(wayBits.W)
        val hitWay = UInt(wayBits.W)
        val replWay = UInt(wayBits.W)
        val chosenWay = UInt(wayBits.W)
        val victimWay = UInt(wayBits.W)
        val blockRefillMaskS2 = UInt(paramL2.ways.W)
        val freeWayMaskS4 = UInt(paramL2.ways.W)
        val dirWbDone = Bool()
        val dirWbDoneSet = UInt(setBits.W)
        val dirWbDoneWay = UInt(wayBits.W)
        val blockRefillLocked = Vec(paramL2.sets, UInt(paramL2.ways.W))
        val ageWb = UInt(4.W)
        val ageRd = UInt(4.W)
        val rrReplRd = UInt(log2Ceil(paramL2.mshrSize).W)
        val rrDirWb = UInt(log2Ceil(paramL2.mshrSize).W)
        val rrDirRd = UInt(log2Ceil(paramL2.mshrSize).W)
        val replRegs = Vec(paramL2.sets, UInt(replBits.W))
        val originRegs = Vec(paramL2.sets, UInt(paramL2.ways.W))
        val psel = UInt(10.W)
        val dirRelScheduling = new Bundle {
          val grant = Bool()
          val active = Bool()
          val commit = Bool()
          val wait_cycles = Bool()
          val retry = Bool()
          val block_normal = Bool()
          val block_pipeline = Bool()
          val block_conflict = Bool()
          val block_cam_commit = Bool()
          val block_backing = Bool()
          val normal_block = Bool()
          val dir_rd_block = Bool()
          val dir_wb_block = Bool()
          val repl_rd_block = Bool()
          val normal_block_winning = Bool()
          val dir_rd_block_winning = Bool()
          val dir_wb_block_winning = Bool()
          val repl_rd_block_winning = Bool()
          val normal_block_inflight = Bool()
          val dir_rd_block_inflight = Bool()
          val dir_wb_block_inflight = Bool()
          val repl_rd_block_inflight = Bool()
        }
      }))
    } else {
      None
    }
  })

  val sets = paramL2.sets
  val ways = paramL2.ways
  require(paramL2.replacement == "plru" || useRRIP,
    s"Directory: unknown replacement policy '${paramL2.replacement}'")
  require(isPow2(ways) && ways >= 2,
    s"Directory: ways must be a power of two >= 2, got $ways")
  require(useRRIP || ways == 4,
    "Directory: PLRU policy is hardwired for 4 ways (use an RRIP policy for other way counts)")
  require(!useDRRIP || setBits >= 2,
    "Directory: DRRIP set dueling requires at least 4 sets")

  // ── Split tag/meta SRAMs + replacement state ────────────────────────
  val tagArray = Module(new SplittedSRAM(
    gen = UInt(tagWidth.W), set = sets, way = ways, waySplit = 2,
    singlePort = true, readMCP2 = false,
    hasMbist = paramL2.hasMbist, hasSramCtl = paramL2.hasSramCtl,
    suffix = Some("l2_tag")
  ))
  val metaArray = Module(new SRAMTemplate(
    gen = new L2Directory.Meta, set = sets, way = ways,
    singlePort = true, useBitmask = true,
    hasMbist = paramL2.hasMbist, hasSramCtl = paramL2.hasSramCtl,
    suffix = Some("l2_meta")
  ))
  val useRRIPSram = useRRIP && paramL2.drripUseSRAM
  val useOriginSram = useRRIPSram && paramL2.originUseSRAM
  val replFF = if (!useRRIPSram) {
    Some(RegInit(VecInit(Seq.fill(sets)(0.U(replBits.W)))))
  } else {
    None
  }
  val originFF = if (!useOriginSram) {
    Some(RegInit(VecInit(Seq.fill(sets)(0.U(ways.W)))))
  } else {
    None
  }
  val replSRAM = if (useRRIPSram) {
    Some(Module(new SRAMTemplate(
      gen = UInt(2.W), set = sets, way = ways, singlePort = true,
      shouldReset = true,
      hasMbist = paramL2.hasMbist, hasSramCtl = paramL2.hasSramCtl,
      suffix = Some("l2_repl")
    )))
  } else {
    None
  }
  val originSRAM = if (useOriginSram) {
    Some(Module(new SRAMTemplate(
      gen = Bool(), set = sets, way = ways, singlePort = true,
      shouldReset = true,
      hasMbist = paramL2.hasMbist, hasSramCtl = paramL2.hasSramCtl,
      suffix = Some("l2_orgb")
    )))
  } else {
    None
  }

  // CoupledL2 clears both replacement arrays after reset. These set-counted
  // sweeps run in lockstep with initActive below, which uses the same depth and
  // masks every request class until tag/meta and replacement initialization end.

  // RRIP storage has three backends:
  // - !useRRIPSram: replFF and originFF hold all rows; no CAM is instantiated.
  // - useRRIPSram && !useOriginSram: replSRAM holds RRPV rows and the CAM caches
  //   active RRPV rows, while originFF is accessed directly. CAM entries have no
  //   origin field in this mixed backend. originUseSRAM is ignored without RRIP
  //   SRAM, so the full-FF backend is independent of that parameter.
  // - useRRIPSram && useOriginSram: replSRAM and originSRAM back CAM entries
  //   containing both complete rows.
  // PLRU uses replFF and bypasses the CAM, replacement SRAM, and origin updates.
  //
  // CAM entry lifetime (the control sites below give the hazard details):
  // 1. A ReplRd CAM miss allocates a valid set entry in S1 and reads the backing
  //    rows. The entry cannot write back before its allocating S4 has resolved.
  // 2. Same-set hits reuse the entry. An older S4 update forwards directly into
  //    S3 and S2; a same-set ReplRd instead waits while the allocation's refill
  //    lock is not yet visible. There is no extra stall for forwarding.
  // 3. Successful S4 operations modify the CAM row when resident, or use masked
  //    backing writes otherwise. An allocating ReplRd retry invalidates its entry.
  // 4. A successful refill-lock acquisition increments ownerCount. DirRd hit
  //    updates and DirRel never acquire ownership and never increment the count.
  // 5. A matching fill-commit DirWb decrements ownerCount at grant and schedules
  //    the existing one-cycle-later lock release. A non-final owner leaves the
  //    entry valid and shared.
  // 6. A final owner is grantable only when its complete commit is safe. That
  //    grant writes RRPV plus origin when present, decrements to zero, and
  //    invalidates the entry at one edge, with no post-grant drain. The separate
  //    pendClear path releases the refill lock on the following edge.
  // 7. An unrelated zero-owner entry may also write back and invalidate in an
  //    eligible DirWb window. CAM-full is an assertion on new-set allocation,
  //    not a backpressure mechanism; capacity must be bounded by the TSHRs.
  val replCAM = if (useRRIPSram) {
    Some(RegInit(VecInit(Seq.fill(nMSHR)(0.U.asTypeOf(
      new L2Directory.ReplCAMEntry(replBits, useOriginSram))))))
  } else {
    None
  }
  val replCAMValidBits = if (useRRIPSram) VecInit(replCAM.get.map(_.valid)).asUInt else 0.U(nMSHR.W)
  def replCAMMatch(set: UInt): UInt = {
    if (useRRIPSram) {
      VecInit((0 until nMSHR).map(i => replCAM.get(i).valid && replCAM.get(i).set === set)).asUInt
    } else {
      0.U(nMSHR.W)
    }
  }
  val replCAMFull = if (useRRIPSram) replCAMValidBits.andR else false.B
  val replCAMFreeIndex = if (useRRIPSram) {
    PriorityEncoder((~replCAMValidBits)(nMSHR - 1, 0))
  } else {
    0.U(idxW.W)
  }
  val replCAMCommitFire = WireDefault(false.B)
  val directBackingWen = WireDefault(false.B)
  val directBackingCapture = WireDefault(false.B)
  val readWriteHazard = WireDefault(false.B)
  val writePending_s4 = RegInit(false.B)
  val writePendingSet_s4 = RegInit(0.U(setBits.W))
  val writePendingData_s4 = RegInit(0.U(replBits.W))
  val writePendingOrigin_s4 = RegInit(0.U(ways.W))
  val writePendingMask_s4 = RegInit(0.U(ways.W))
  val readLaunchValid_s1 = RegInit(false.B)
  val readLaunchIsRepl_s1 = RegInit(false.B)
  val readLaunchIsDirRd_s1 = RegInit(false.B)
  val readLaunchSet_s1 = RegInit(0.U(setBits.W))
  val readLaunchReplCAMUse_s1 = RegInit(false.B)
  val readLaunchReplCAMAlloc_s1 = RegInit(false.B)
  val readLaunchReplCAMIndex_s1 = RegInit(0.U(idxW.W))
  val originReadValidLaunch_s1 = RegInit(false.B)
  val originReadResponseValid_s1 = RegInit(false.B)
  val writeLaunchPending = WireDefault(false.B)
  val s2_valid = WireDefault(false.B)
  val s2_replCAMAlloc = WireDefault(false.B)
  val s2_set = Wire(UInt(setBits.W))
  val s3_valid = WireDefault(false.B)
  val s3_replCAMAlloc = WireDefault(false.B)
  val s3_set = Wire(UInt(setBits.W))
  val s4_valid = WireDefault(false.B)
  val s4_replCAMAlloc = WireDefault(false.B)
  val s4_set = Wire(UInt(setBits.W))

  val relActive = RegInit(false.B)
  val relSet = Reg(UInt(setBits.W))
  val relWay = Reg(UInt(wayBits.W))
  val relTshr = Reg(UInt(mshrIndexWidth.W))
  val relLocalValid = Reg(Bool())
  val relPrefetch = Reg(Bool())
  val relReplCAMHit = Reg(Bool())
  // Refill lock table: one slot per TSHR, indexed directly by TSHR ID. CAM
  // ownership retires at the DirWb grant; pendClear only delays the lock clear.
  // The pending refill bypass makes the registered commit visible before the
  // physical table update, without overriding a newer same-slot hit lock.
  val lockTable = RegInit(VecInit(Seq.fill(nMSHR)(0.U.asTypeOf(new L2Directory.RefillLockEntry))))
  val refillPending_s4 = RegInit(false.B)
  val refillPendingTshr_s4 = RegInit(0.U(idxW.W))
  val refillPendingSet_s4 = RegInit(0.U(setBits.W))
  val refillPendingWay_s4 = RegInit(0.U(wayBits.W))
  val refillThisEntryVec = VecInit((0 until nMSHR).map { i =>
    refillPending_s4 && refillPendingTshr_s4 === i.U
  })
  val effectiveLockTable = Wire(Vec(nMSHR, new L2Directory.RefillLockEntry))
  for (i <- 0 until nMSHR) {
    effectiveLockTable(i).valid := refillThisEntryVec(i) || lockTable(i).valid
    effectiveLockTable(i).hitHold := Mux(refillThisEntryVec(i), false.B, lockTable(i).hitHold)
    effectiveLockTable(i).set := Mux(refillThisEntryVec(i), refillPendingSet_s4, lockTable(i).set)
    effectiveLockTable(i).way := Mux(refillThisEntryVec(i), refillPendingWay_s4, lockTable(i).way)
  }
  val pendClearValid = RegInit(false.B)
  val pendClearIdx = RegInit(0.U(idxW.W))
  val dirWbCommitBlocked = WireDefault(false.B)
  val pendClearApply = WireDefault(false.B)
  val initActive = RegInit(true.B)
  val initSet = RegInit(0.U(setBits.W))
  val initMeta = Wire(new L2Directory.Meta)
  initMeta.state := L2Directory.MetaState.I
  initMeta.dirty := false.B
  initMeta.alias := 0.U
  initMeta.clients.foreach(_ := false.B)
  initMeta.prefetch := false.B
  val initEntry = Wire(new L2Directory.DirEntry)
  initEntry.tag := 0.U
  initEntry.meta := initMeta
  val initMetaMask = Wire(new L2Directory.Meta)
  initMetaMask.state := Fill(2, true.B)
  initMetaMask.dirty := true.B
  initMetaMask.alias := Fill(2, true.B)
  initMetaMask.clients.foreach(_ := true.B)
  initMetaMask.prefetch := true.B
  val initWayMask = Fill(ways, true.B)
  when(initActive) {
    when(initSet === (sets - 1).U) {
      initActive := false.B
    }.otherwise {
      initSet := initSet + 1.U
    }
  }
  if (useRRIPSram) {
    when (!initActive) {
      assert(replSRAM.get.io.resetDone,
        "Directory: RRIP backing clear did not finish before request admission")
      if (useOriginSram) {
        assert(originSRAM.get.io.resetDone,
          "Directory: origin backing clear did not finish before request admission")
      }
    }
  }

  // ══════════════════════════════════════════════════════════════════
  // Internal arbiter
  // ══════════════════════════════════════════════════════════════════
  def requestSet(req: L2Directory.PathToDirectory): UInt =
    req.PADDR(offsetBits + setBits - 1, offsetBits)
  def replCAMReadBlocked(req: L2Directory.PathToDirectory): Bool = {
    val set = requestSet(req)
    (readLaunchValid_s1 && readLaunchReplCAMAlloc_s1 && readLaunchSet_s1 === set) ||
    (s2_valid && s2_replCAMAlloc && s2_set === set) ||
    (s3_valid && s3_replCAMAlloc && s3_set === set) ||
    (s4_valid && s4_replCAMAlloc && s4_set === set)
  }
  // ReplRd must see the older refill lock before choosing a victim. A CAM-resident
  // DirRd instead uses S4-to-S3/S2 forwarding and needs no allocation-specific stall.
  val replRdVec = VecInit(io.toDir.map(req => req.ReplRd && !replCAMReadBlocked(req)))
  val dirWbVec  = VecInit(io.toDir.map(_.DirWb))
  val dirWbUnlockVec = VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
    val lock = effectiveLockTable(i)
    req.DirWb && req.TAG_WEN && lock.valid &&
      lock.set === requestSet(req) && lock.way === req.WAY(wayBits - 1, 0)
  })
  val requestReplCAMMatchVec = VecInit(io.toDir.map(req => replCAMMatch(requestSet(req))))
  val dirWbReplCAMIndexVec = VecInit(requestReplCAMMatchVec.map(matches => PriorityEncoder(matches)))
  // Candidate decode precedes the winner so final-commit readiness can gate the
  // DirWb grant without a combinational path from the grant back to itself.
  val dirWbFinalVec = if (useRRIPSram) {
    VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
      dirWbUnlockVec(i) && requestReplCAMMatchVec(i).orR &&
        replCAM.get(dirWbReplCAMIndexVec(i)).ownerCount === 1.U
    })
  } else {
    VecInit(Seq.fill(nMSHR)(false.B))
  }
  val dirRdVec  = VecInit(io.toDir.map(_.DirRd))

  val rawAnyReplRd = replRdVec.asUInt.orR
  val rawAnyDirWb  = dirWbVec.asUInt.orR
  val rawAnyDirRd  = dirRdVec.asUInt.orR
  val anyReplRd = rawAnyReplRd && !initActive
  val anyDirWb  = rawAnyDirWb && !initActive
  val anyDirRd  = rawAnyDirRd && !initActive
  val anyLevel2 = anyDirWb || anyDirRd
  val anyNormal = anyReplRd || anyLevel2
  val relVec = VecInit(io.toDir.map(_.DirRel))
  val anyRel = relVec.asUInt.orR && !initActive && useRRIPRelease.B
  val lastGrantWasRel = if (paramL2.dirRelArbitration == "rr") {
    Some(RegInit(false.B))
  } else {
    None
  }
  val dirRelClassWins = paramL2.dirRelArbitration match {
    case "dirrel-high" => anyRel
    case "rr"          => anyRel && (!anyNormal || !lastGrantWasRel.get)
    case _             => false.B
  }
  val dirRelClassBlock = if (paramL2.dirRelArbitration == "dirrel-low") {
    false.B
  } else {
    relActive || dirRelClassWins
  }
  val dirRelWinningBlock = dirRelClassBlock && !relActive && dirRelClassWins
  val dirRelInflightBlock = dirRelClassBlock && relActive

  // Replacement-read and pipeline hazards are driven after S2/S3/S4 exist.
  val replStall = Wire(Bool())
  val pipelineBusy = WireDefault(false.B)

  // starvation counters for Level-2 groups
  val ageWb = RegInit(0.U(4.W))
  val ageRd = RegInit(0.U(4.W))
  val wbStarved  = anyDirWb && (ageWb === MAX_AGE)
  val rdStarved  = anyDirRd && (ageRd === MAX_AGE)
  val anyStarved = wbStarved || rdStarved

  val wbWinsAge = anyDirWb && (!anyDirRd || (ageWb >= ageRd))
  val level2WinIsWb = anyLevel2 && Mux(anyStarved,
    Mux(wbStarved && rdStarved, ageWb >= ageRd, wbStarved),
    wbWinsAge)

  // ReplRd preempts Level-2 unless starved or blocked by a replacement/CAM hazard.
  // CAM commit eligibility never stalls normal Directory operations.
  val replRdWins = anyReplRd && !anyStarved && !replStall && !readWriteHazard

  // round-robin pointers
  val rrReplRd = RegInit(0.U(idxW.W))
  val rrDirWb  = RegInit(0.U(idxW.W))
  val rrDirRd  = RegInit(0.U(idxW.W))

  def roundRobinNext(last: UInt, mask: UInt, n: Int): UInt = {
    val w        = log2Ceil(n)
    val doubled  = Cat(mask, mask)
    val startPos = Mux(last === (n - 1).U, 0.U, last + 1.U)
    val rotated  = (doubled >> startPos)(n - 1, 0)
    val relIdx   = PriorityEncoder(rotated)
    val absRaw   = startPos +& relIdx
    val abs      = Mux(absRaw >= n.U, absRaw - n.U, absRaw)
    abs(w - 1, 0)
  }

  def roundRobinOneHot(last: UInt, mask: UInt, n: Int): UInt = {
    val w = log2Ceil(n)
    val start = Mux(last === (n - 1).U, 0.U(w.W), last + 1.U)
    val startOneHot = UIntToOH(start, n)
    val beforeStart = ((1.U((n + 1).W) << start) - 1.U)(n - 1, 0)
    val upper = Mux(last === (n - 1).U, 0.U(n.W), mask & ~beforeStart)
    val firstUpper = upper & (~upper + 1.U)
    val firstAny = mask & (~mask + 1.U)
    Mux(upper.orR, firstUpper, Mux(mask.orR, firstAny, startOneHot))
  }

  def oneHotToIndex(oneHot: UInt, n: Int): UInt = {
    val w = log2Ceil(n)
    VecInit((0 until w).map { bit =>
      balancedOr((0 until n).filter(i => (i & (1 << bit)) != 0).map(i => oneHot(i)))
    }).asUInt
  }

  def balancedOneHotMux(select: UInt, inputs: Seq[UInt]): UInt = {
    require(inputs.nonEmpty)
    require(inputs.map(_.getWidth).distinct.length == 1)
    val width = inputs.head.getWidth
    VecInit((0 until width).map { bit =>
      balancedOr(inputs.indices.map(i => select(i) && inputs(i)(bit)))
    }).asUInt
  }

  val winReplRdOH = roundRobinOneHot(rrReplRd, replRdVec.asUInt, nMSHR)
  val winDirWbOH = roundRobinOneHot(rrDirWb, dirWbVec.asUInt, nMSHR)
  val winDirRdOH = roundRobinOneHot(rrDirRd, dirRdVec.asUInt, nMSHR)
  val winReplRd = oneHotToIndex(winReplRdOH, nMSHR)
  val winDirWb = oneHotToIndex(winDirWbOH, nMSHR)
  val winDirRd = oneHotToIndex(winDirRdOH, nMSHR)
  val dirWbSelectedFinal = balancedOr(io.toDir.indices.map { i =>
    winDirWbOH(i) && dirWbFinalVec(i)
  })

  // Grantable describes the normal-group result without the DirRel class gate.
  // It includes final-commit and replacement/backing hazards so reverse-stall
  // counters identify only real grants suppressed by winning/in-flight DirRel.
  // A request admitted during a registered write cycle reaches the array in the
  // following cycle. Normal traffic can be pipelined without a blanket
  // writeLaunchPending admission bubble; current-cycle backing hazards still apply.
  val normalReplRdGrantable = replRdWins && anyReplRd
  val normalDirWbGrantable = !dirWbCommitBlocked && !replRdWins && anyLevel2 && level2WinIsWb && anyDirWb
  // Full-FF DirRd must also observe every older S4 full-row update. SRAM
  // backends retain their existing CAM/direct interlocks instead of this stall.
  val normalDirRdGrantable = !replRdWins && anyLevel2 && !level2WinIsWb && anyDirRd &&
    (useRRIPSram.B || !replStall) && !readWriteHazard
  val grantReplRd = !dirRelClassBlock && normalReplRdGrantable
  val grantDirWb  = !dirRelClassBlock && normalDirWbGrantable
  val grantDirRd  = !dirRelClassBlock && normalDirRdGrantable

  val dirRdBlockDirRelWinning = dirRelWinningBlock && normalDirRdGrantable
  val dirWbBlockDirRelWinning = dirRelWinningBlock && normalDirWbGrantable
  val replRdBlockDirRelWinning = dirRelWinningBlock && normalReplRdGrantable
  val dirRdBlockDirRelInflight = dirRelInflightBlock && normalDirRdGrantable
  val dirWbBlockDirRelInflight = dirRelInflightBlock && normalDirWbGrantable
  val replRdBlockDirRelInflight = dirRelInflightBlock && normalReplRdGrantable
  val normalBlockDirRelWinning = dirRdBlockDirRelWinning || dirWbBlockDirRelWinning || replRdBlockDirRelWinning
  val normalBlockDirRelInflight = dirRdBlockDirRelInflight || dirWbBlockDirRelInflight || replRdBlockDirRelInflight
  val dirRdBlockDirRel = dirRdBlockDirRelWinning || dirRdBlockDirRelInflight
  val dirWbBlockDirRel = dirWbBlockDirRelWinning || dirWbBlockDirRelInflight
  val replRdBlockDirRel = replRdBlockDirRelWinning || replRdBlockDirRelInflight
  val normalBlockDirRel = dirRdBlockDirRel || dirWbBlockDirRel || replRdBlockDirRel

  val winIdx   = Mux(grantReplRd, winReplRd, Mux(grantDirWb, winDirWb, winDirRd))
  val winOH = Mux(grantReplRd, winReplRdOH, Mux(grantDirWb, winDirWbOH, winDirRdOH))
  val anyGrant = grantReplRd || grantDirWb || grantDirRd
  val winReq   = io.toDir(winIdx)

  when(grantReplRd) { rrReplRd := winReplRd }
  when(grantDirWb)  { rrDirWb  := winDirWb  }
  when(grantDirRd)  { rrDirRd  := winDirRd  }

  def incrCapped(x: UInt, cond: Bool): UInt = Mux(cond && (x < MAX_AGE), x + 1.U, x)
  ageWb := Mux(grantDirWb, 0.U, incrCapped(ageWb, anyDirWb))
  ageRd := Mux(grantDirRd, 0.U, incrCapped(ageRd, anyDirRd))

  // enableRRIPReleaseUpdate gates the DirRel request path for DRRIP/SRRIP when
  // the raw feature switch is enabled; BRRIP/PLRU omit the path in every module.
  // dirRelArbitration applies between that class and the normal operation class:
  // dirrel-low keeps normal-first arbitration, dirrel-high gives DirRel strict
  // priority, and rr alternates the class turn when both classes are pending.
  // The normal group's internal ReplRd > DirRd/DirWb policy is unchanged.
  //
  // Every DirRel grant requires an idle normal pipeline. In dirrel-low, a
  // final-owner DirWb remains ungranted during relActive so its commit cannot
  // intersect the accepted R/W turn. With dirRelSingleCycle=false, R captures
  // the accepted request and relActive
  // reserves the following W turn. R reads origin only when a full-SRAM set is
  // not CAM-resident; mixed mode reads originFF directly. W commits RRPV, origin,
  // and the metadata prefetch clear after same-set/CAM/port hazards have cleared,
  // then acknowledges the request. No Tag/Meta read is allowed in either phase.
  //
  // Single-cycle mode is available only with RRIP SRAM and origin FF. It commits
  // and acknowledges from the current request at dirRelGrant without relActive,
  // but retains the idle-pipeline, RRIP/meta-port, and write-hazard guards. It
  // removes the separate R/W occupancy, not arbitration protection.
  // DirRel never allocates a CAM entry or increments ownerCount in either mode.
  val dirRelGrant = WireDefault(false.B)
  val dirRelPath = if (useRRIPRelease) {
    val rrRel  = RegInit(0.U(idxW.W))
    val winRel = roundRobinNext(rrRel, relVec.asUInt, nMSHR)
    when(dirRelGrant) { rrRel := winRel }
    Some((anyRel, io.toDir(winRel)))
  } else {
    None
  }
  dirRelGrant := dirRelPath.map(_._1).getOrElse(false.B) && useRRIPRelease.B &&
    !relActive && !anyGrant && !pipelineBusy && !writeLaunchPending
  if (paramL2.dirRelArbitration == "rr") {
    when(dirRelGrant) { lastGrantWasRel.get := true.B }
    when(anyGrant) { lastGrantWasRel.get := false.B }
  }
  val dirRelReq = dirRelPath.map(_._2).getOrElse(0.U.asTypeOf(new L2Directory.PathToDirectory))

  // ══════════════════════════════════════════════════════════════════
  // S1: decode the arbitration winner
  // ══════════════════════════════════════════════════════════════════
  val isReplRd = grantReplRd
  val isDirRd  = grantDirRd
  val isDirWb  = grantDirWb
  val isRead   = isReplRd || isDirRd

  val reqSet = balancedOneHotMux(
    winOH,
    io.toDir.toSeq.map(req => requestSet(req))
  )
  val reqTag = winReq.PADDR(paramL2.physicalAddrWidth - 1, offsetBits + setBits)

  // Only a ReplRd CAM miss allocates an entry. S1 establishes valid/set and a
  // zero ownerCount; the allocating request still takes the backing read at S2
  // because its registered s2_replCAMHit is false. Younger same-set DirRds may
  // encounter the valid entry before that allocation reaches S4, so the older
  // allocating S4 forwards its completed row into younger S3 and S2 stages. Same-set
  // ReplRd requests instead remain blocked until the allocation's lock outcome
  // is visible. CAM-full and duplicate-set allocation are assertions, not stalls.
  val requestReplCAMHitVec = VecInit(io.toDir.map { req =>
    replCAMMatch(requestSet(req)).orR
  })
  // Reuse the per-requester CAM index and prepare each complete row before the
  // winner is known. After arbitration, balanced one-hot selection replaces a
  // selected-set CAM comparison, priority encoder, and dynamic row read.
  val requestReplCAMReplVec = VecInit(io.toDir.indices.map { i =>
    if (useRRIPSram) replCAM.get(dirWbReplCAMIndexVec(i)).repl else 0.U(replBits.W)
  })
  val requestReplCAMOriginVec = VecInit(io.toDir.indices.map { i =>
    if (useRRIPSram && useOriginSram) replCAM.get(dirWbReplCAMIndexVec(i)).origin.get else 0.U(ways.W)
  })
  val s1ReplCAMHit = balancedOr(io.toDir.indices.map { i =>
    winOH(i) && requestReplCAMHitVec(i)
  })
  val s1ReplCAMIndex = balancedOneHotMux(winOH, dirWbReplCAMIndexVec.toSeq)
  val s1ReplCAMAlloc = isReplRd && useRRIPSram.B && !s1ReplCAMHit
  val s1ReplCAMIndexOrAlloc = Mux(s1ReplCAMHit, s1ReplCAMIndex, replCAMFreeIndex)
  if (useRRIPSram) {
    when (s1ReplCAMAlloc) {
      assert(!replCAMFull, "RRIP CAM full on a new ReplRd set allocation")
      assert(!replCAMMatch(reqSet).orR, "duplicate RRIP CAM set allocation")
      replCAM.get(replCAMFreeIndex).valid := true.B
      replCAM.get(replCAMFreeIndex).set := reqSet
      replCAM.get(replCAMFreeIndex).ownerCount := 0.U
    }
  }

  val dirRelSet = dirRelReq.PADDR(offsetBits + setBits - 1, offsetBits)
  val dirRelReplCAMMatch = replCAMMatch(dirRelSet)
  val dirRelReplCAMHit = dirRelReplCAMMatch.orR
  val relNeedsOrigin = dirRelGrant && !dirRelReplCAMHit && useOriginSram.B

  readLaunchValid_s1 := isRead
  readLaunchIsRepl_s1 := isReplRd
  readLaunchIsDirRd_s1 := isDirRd
  readLaunchSet_s1 := reqSet
  readLaunchReplCAMUse_s1 := s1ReplCAMHit || s1ReplCAMAlloc
  readLaunchReplCAMAlloc_s1 := s1ReplCAMAlloc
  readLaunchReplCAMIndex_s1 := s1ReplCAMIndexOrAlloc

  val tagReadData_s2 = tagArray.io.r(readLaunchValid_s1, readLaunchSet_s1).resp.data
  val metaReadData_s2 = metaArray.io.r(readLaunchValid_s1, readLaunchSet_s1).resp.data
  val dirReadData_s2 = Wire(Vec(ways, new L2Directory.DirEntry))
  for (w <- 0 until ways) {
    dirReadData_s2(w).tag := tagReadData_s2(w)
    dirReadData_s2(w).meta := metaReadData_s2(w)
  }

  val s1ReplCAMRepl = balancedOneHotMux(winOH, requestReplCAMReplVec.toSeq)
  val s1ReplCAMOrigin = balancedOneHotMux(winOH, requestReplCAMOriginVec.toSeq)
  val replState_s1 = Mux(s1ReplCAMHit, s1ReplCAMRepl,
    replFF.map(_(reqSet)).getOrElse(0.U(replBits.W)))
  val originState_s1 = Mux(s1ReplCAMHit && useOriginSram.B, s1ReplCAMOrigin,
    originFF.map(_(reqSet)).getOrElse(0.U(ways.W)))
  val replReadValidLaunch_s1 = RegInit(false.B)
  replReadValidLaunch_s1 := isRead && !s1ReplCAMHit
  val replSramRead_s2 = replSRAM.map(_.io.r(replReadValidLaunch_s1, readLaunchSet_s1).resp.data.asUInt)
  val originReadSet = Mux(relNeedsOrigin, dirRelSet, reqSet)
  val originReadSetLaunch_s1 = RegInit(0.U(setBits.W))
  originReadSetLaunch_s1 := originReadSet
  originReadValidLaunch_s1 := (isRead && !s1ReplCAMHit) || relNeedsOrigin
  originReadResponseValid_s1 := originReadValidLaunch_s1
  val originSramRead_s2 = originSRAM.map(_.io.r(
    originReadValidLaunch_s1, originReadSetLaunch_s1).resp.data.asUInt)

  // ── S1 → S2 ───────────────────────────────────────────────────────
  // The registered launch issues the SRAM request in S2. CAM snapshots retain
  // their registered fallback until the corresponding SRAM response reaches S3.
  s2_valid := RegNext(isRead, false.B)
  val s2_isRepl = RegNext(isReplRd, false.B)
  val s2_isDirRd = RegNext(isDirRd, false.B)
  val s2_tshr = RegNext(winReq.TSHRID, 0.U)
  s2_set := RegNext(reqSet, 0.U)
  val s2_tag = RegNext(reqTag, 0.U)
  val s2_repl_reg = RegNext(replState_s1, 0.U)
  val s2_origin_reg = RegNext(originState_s1, 0.U)
  val s2_replCAMHit = RegNext(s1ReplCAMHit, false.B)
  val s2_replCAMUse = RegNext(s1ReplCAMHit || s1ReplCAMAlloc, false.B)
  s2_replCAMAlloc := RegNext(s1ReplCAMAlloc, false.B)
  val s2_replCAMIndex = RegNext(s1ReplCAMIndexOrAlloc, 0.U)
  val s2_forwardRepl = Wire(UInt(replBits.W))
  val s2_forwardOrigin = Wire(UInt(ways.W))
  val s2_repl = Mux(s2_replCAMHit, s2_forwardRepl, s2_repl_reg)
  val s2_origin = Mux(s2_replCAMHit, s2_forwardOrigin, s2_origin_reg)
  val s2_replCAMStale = Wire(Bool())
  val s2_rrip_update = RegNext(winReq.RRIP_UPDATE, false.B)
  val s2_rrip_prefetch = RegNext(winReq.RRIP_PREFETCH, false.B)
  val s2_rrip_refill_prefetch = RegNext(winReq.RRIP_REFILL_PREFETCH, false.B)

  // ── S2 → S3: the raw SRAM response arrives ─────────────────────────
  s3_valid := RegNext(s2_valid, false.B)
  val s3_isRepl = RegNext(s2_isRepl, false.B)
  val s3_isDirRd = RegNext(s2_isDirRd, false.B)
  val s3_tshr = RegNext(s2_tshr, 0.U)
  s3_set := RegNext(s2_set, 0.U)
  val s3_tag = RegNext(s2_tag, 0.U)
  val s3_replCAMHit = RegNext(s2_replCAMHit, false.B)
  val s3_repl_reg = RegNext(s2_repl, 0.U)
  val s3_origin_reg = RegNext(s2_origin, 0.U)
  val s3_forwardRepl = Wire(UInt(replBits.W))
  val s3_forwardOrigin = Wire(UInt(ways.W))
  val s3_repl = Mux(s3_replCAMHit, s3_forwardRepl,
    if (useRRIPSram) replSramRead_s2.get else s3_repl_reg)
  val s3_origin = Mux(s3_replCAMHit, s3_forwardOrigin,
    if (useOriginSram) originSramRead_s2.get else s3_origin_reg)
  val s3_replCAMStale = RegNext(s2_replCAMStale, false.B)
  val s3_replCAMUse = RegNext(s2_replCAMUse, false.B) && !s3_replCAMStale
  s3_replCAMAlloc := RegNext(s2_replCAMAlloc, false.B)
  val s3_replCAMIndex = RegNext(s2_replCAMIndex, 0.U)
  val s3_rrip_update = RegNext(s2_rrip_update, false.B)
  val s3_rrip_prefetch = RegNext(s2_rrip_prefetch, false.B)
  val s3_rrip_refill_prefetch = RegNext(s2_rrip_refill_prefetch, false.B)
  val s3_replCAMStaleNow = Wire(Bool())

  // ── S3 → S4: register response data before hit/result logic ─────────
  s4_valid := RegNext(s3_valid, false.B)
  val s4_isRepl = RegNext(s3_isRepl, false.B)
  val s4_isDirRd = RegNext(s3_isDirRd, false.B)
  val s4_tshr = RegNext(s3_tshr, 0.U)
  s4_set := RegNext(s3_set, 0.U)
  val s4_tag = RegNext(s3_tag, 0.U)
  val s4_replCAMHit = RegNext(s3_replCAMHit, false.B)
  val s4_repl = RegNext(s3_repl, 0.U)
  val s4_origin = RegNext(s3_origin, 0.U)
  val s4_replCAMStale = RegNext(s3_replCAMStale || s3_replCAMStaleNow, false.B)
  val s4_replCAMUse = RegNext(s3_replCAMUse, false.B) && !s4_replCAMStale
  s4_replCAMAlloc := RegNext(s3_replCAMAlloc, false.B)
  val s4_replCAMIndex = RegNext(s3_replCAMIndex, 0.U)
  val s4_rrip_update = RegNext(s3_rrip_update, false.B)
  val s4_rrip_prefetch = RegNext(s3_rrip_prefetch, false.B)
  val s4_rrip_refill_prefetch = RegNext(s3_rrip_refill_prefetch, false.B)
  directBackingWen := writePending_s4
  // Only an actual backing write blocks a younger read. The pending cycle then
  // protects the registered write launch and its read-after-write ordering.
  readWriteHazard := writePending_s4 || directBackingCapture
  pipelineBusy := readLaunchValid_s1 || originReadValidLaunch_s1 || originReadResponseValid_s1 ||
    writePending_s4 || writeLaunchPending || s2_valid || s3_valid || s4_valid

  when (dirRelGrant) {
    if (!paramL2.dirRelSingleCycle) {
      relActive := true.B
    }
    relSet := dirRelSet
    relWay := dirRelReq.WAY(wayBits - 1, 0)
    relTshr := dirRelReq.TSHRID
    relLocalValid := dirRelReq.RRIP_LOCAL_VALID
    relPrefetch := dirRelReq.META.prefetch
    relReplCAMHit := dirRelReplCAMHit
  }
  val relReplCAMMatchNow = replCAMMatch(relSet)
  val relReplCAMHitNow = relReplCAMMatchNow.orR
  val relReplCAMIndexNow = PriorityEncoder(relReplCAMMatchNow)
  val relSetConflict = (anyGrant && reqSet === relSet) ||
    (readLaunchValid_s1 && readLaunchSet_s1 === relSet) ||
    (originReadValidLaunch_s1 && originReadSetLaunch_s1 === relSet) ||
    (s2_valid && s2_set === relSet) || (s3_valid && s3_set === relSet) ||
    (s4_valid && s4_set === relSet)
  val relReplCAMCommitConflict = relReplCAMHit && !relReplCAMHitNow
  val relConflict = relSetConflict || relReplCAMCommitConflict

  // Low-priority mode lets normal traffic win and retries stale R attempts.
  // High/RR holds normal grants through W; this guard still prevents stale
  // same-set or CAM-lifecycle writes.
  when (relActive && relConflict) {
    relActive := false.B
  }
  val relWFire = relActive && !relConflict && !anyGrant && !pipelineBusy &&
    !replCAMCommitFire && !directBackingWen
  // Two-cycle mode commits at relWFire from the payload captured at R, including
  // the acceptance-time local-valid and prefetch bits. Single-cycle mode instead
  // consumes the current dirRelReq/dirRelSet wires at the grant edge. The shared
  // relCommit* selection keeps the actual RRPV/origin/meta update identical.
  val dirRelCommitFire = if (paramL2.dirRelSingleCycle) dirRelGrant else relWFire
  val relCommitSet = if (paramL2.dirRelSingleCycle) dirRelSet else relSet
  val relCommitWay = if (paramL2.dirRelSingleCycle) dirRelReq.WAY(wayBits - 1, 0) else relWay
  val relCommitTshr = if (paramL2.dirRelSingleCycle) dirRelReq.TSHRID else relTshr
  val relCommitLocalValid = if (paramL2.dirRelSingleCycle) dirRelReq.RRIP_LOCAL_VALID else relLocalValid
  val relCommitPrefetch = if (paramL2.dirRelSingleCycle) dirRelReq.META.prefetch else relPrefetch

  // A full-SRAM R miss captures its origin row only when the registered read
  // response is valid. A delayed W uses this per-request copy so an intervening
  // dirrel-low read cannot replace the row before the RMW commit.
  val relOriginHold = if (useOriginSram) Some(Reg(UInt(ways.W))) else None
  val relOriginHoldValid = if (useOriginSram) Some(RegInit(false.B)) else None
  if (useOriginSram) {
    when (relActive && !relReplCAMHit && !relOriginHoldValid.get && originReadResponseValid_s1) {
      relOriginHold.get := originSramRead_s2.get
      relOriginHoldValid.get := true.B
    }
    when (dirRelGrant || (relActive && relConflict) || dirRelCommitFire) {
      relOriginHoldValid.get := false.B
    }
  }

  // Class-level scheduling events, not per-request latency measurements. The
  // normal-block event requires an actual normal grant to be the only remaining
  // R/W-slot blocker; other wait reasons are non-exclusive hazard observations.
  val dirRelRWait = anyRel && !relActive && !dirRelGrant
  val dirRelWWait = if (paramL2.dirRelSingleCycle) false.B else relActive && !relWFire
  val dirRelWait = dirRelRWait || dirRelWWait
  val dirRelRetry = if (paramL2.dirRelSingleCycle) false.B else relActive && relConflict
  val dirRelRNormalBlock = dirRelRWait && anyGrant && !pipelineBusy
  val dirRelWNormalBlock = dirRelWWait && anyGrant && !relConflict && !pipelineBusy &&
    !replCAMCommitFire && !directBackingWen
  val dirRelBlockNormal = dirRelRNormalBlock || dirRelWNormalBlock
  val dirRelBlockPipeline = dirRelWait && pipelineBusy
  val dirRelBlockConflict = dirRelWWait && relConflict
  val dirRelBlockCAMCommit = dirRelWWait && replCAMCommitFire
  val dirRelBlockBacking = dirRelWWait && directBackingWen

  // ══════════════════════════════════════════════════════════════════
  // DirWb: tag and bitmask partial meta writes at S1
  // ══════════════════════════════════════════════════════════════════
  val wbEntry = Wire(new L2Directory.DirEntry)
  wbEntry.tag  := reqTag
  wbEntry.meta := winReq.META

  val wbMetaMask = Wire(new L2Directory.Meta)
  wbMetaMask.state := Fill(2, winReq.META_WEN.state)
  wbMetaMask.dirty := winReq.META_WEN.dirty
  wbMetaMask.alias := Fill(2, winReq.META_WEN.alias)
  wbMetaMask.clients.zip(winReq.META_WEN.clients).foreach { case (m, w) => m := w }
  wbMetaMask.prefetch := winReq.META_WEN.prefetch

  val relMetaClear = Wire(new L2Directory.DirEntry)
  relMetaClear := 0.U.asTypeOf(new L2Directory.DirEntry)
  val relMetaMask = Wire(new L2Directory.Meta)
  relMetaMask := 0.U.asTypeOf(new L2Directory.Meta)
  relMetaMask.prefetch := true.B

  val tagWriteValid = initActive || (isDirWb && winReq.TAG_WEN)
  val metaWriteValid = initActive || isDirWb || dirRelCommitFire
  val dirWriteEntry = Mux(initActive, initEntry, Mux(dirRelCommitFire, relMetaClear, wbEntry))
  val dirWriteSet = Mux(initActive, initSet, Mux(dirRelCommitFire, relCommitSet, reqSet))
  val dirWriteWayMask = Mux(initActive, initWayMask,
    Mux(dirRelCommitFire, UIntToOH(relCommitWay, ways), UIntToOH(winReq.WAY(wayBits - 1, 0), ways)))
  val dirWriteMetaMask = Mux(initActive, initMetaMask.asUInt,
    Mux(dirRelCommitFire, relMetaMask.asUInt, wbMetaMask.asUInt))

  val tagWriteValidLaunch_s1 = RegNext(tagWriteValid, false.B)
  val metaWriteValidLaunch_s1 = RegNext(metaWriteValid, false.B)
  val tagWriteDataLaunch_s1 = RegInit(0.U(tagWidth.W))
  tagWriteDataLaunch_s1 := dirWriteEntry.tag
  val metaWriteDataLaunch_s1 = RegInit(0.U.asTypeOf(new L2Directory.Meta))
  metaWriteDataLaunch_s1 := dirWriteEntry.meta
  val dirWriteSetLaunch_s1 = RegInit(0.U(setBits.W))
  dirWriteSetLaunch_s1 := dirWriteSet
  val dirWriteWayMaskLaunch_s1 = RegInit(0.U(ways.W))
  dirWriteWayMaskLaunch_s1 := dirWriteWayMask
  val dirWriteMetaMaskLaunch_s1 = RegInit(0.U(new L2Directory.Meta().getWidth.W))
  dirWriteMetaMaskLaunch_s1 := dirWriteMetaMask

  tagArray.io.w(
    valid   = tagWriteValidLaunch_s1,
    data    = tagWriteDataLaunch_s1,
    setIdx  = dirWriteSetLaunch_s1,
    waymask = dirWriteWayMaskLaunch_s1
  )
  metaArray.io.w(
    valid   = metaWriteValidLaunch_s1,
    data    = metaWriteDataLaunch_s1,
    setIdx  = dirWriteSetLaunch_s1,
    waymask = dirWriteWayMaskLaunch_s1,
    bitmask = dirWriteMetaMaskLaunch_s1
  )

  if (enableDirStateProbe) {
    io.debugStateWrite.get.valid := isDirWb
    io.debugStateWrite.get.set := reqSet
    io.debugStateWrite.get.way := winReq.WAY(wayBits - 1, 0)
    io.debugStateWrite.get.state := winReq.META.state
    io.debugStateWrite.get.dirty := winReq.META.dirty
    io.debugStateWrite.get.client0 := winReq.META.clients(0)
  }

  // DirWb completion remains at the grant cycle. Its complete write bundle is
  // registered for the following edge. Normal admissions pipeline behind that
  // write, while writeLaunchPending retains the CAM/DirRel visibility hazards.

  // ══════════════════════════════════════════════════════════════════
  // S4: hit detect, way select, PLRU update from the registered response
  // ══════════════════════════════════════════════════════════════════
  // The registered SRAM request is issued in S2 and its raw response arrives in
  // S3. Capture the complete tag/meta row before any hit or result consumer.
  val dirData_s4 = RegEnable(dirReadData_s2, 0.U.asTypeOf(dirReadData_s2), s3_valid)

  val tagMatch = VecInit(dirData_s4.map(_.tag === s4_tag))
  val validVec = VecInit(dirData_s4.map(_.meta.state =/= L2Directory.MetaState.I))
  val hitVec   = VecInit(tagMatch.zip(validVec).map { case (t, v) => t && v })
  val hit_s4   = hitVec.asUInt.orR
  val hitWay   = PriorityEncoder(hitVec)
  val blockRefillMask_s2 = Wire(UInt(ways.W))
  blockRefillMask_s2 := VecInit((0 until ways).map { w =>
    VecInit(effectiveLockTable.map(e => e.valid && e.set === s2_set && e.way === w.U)).asUInt.orR
  }).asUInt
  val freeWayMask_s2 = Wire(UInt(ways.W))
  freeWayMask_s2 := (~blockRefillMask_s2)(ways - 1, 0)
  val freeWayMask_s3 = RegInit(((BigInt(1) << ways) - 1).U(ways.W))
  when(s2_isRepl) {
    freeWayMask_s3 := freeWayMask_s2
  }
  val freeWayMask_s4 = RegInit(((BigInt(1) << ways) - 1).U(ways.W))
  when(s3_isRepl) {
    freeWayMask_s4 := freeWayMask_s3
  }

  // FF retains the original global RAW guard. The SRAM backend needs it only
  // for direct backing operations; CAM-resident updates can proceed by set.
  if (useRRIPSram) {
    replStall := (readLaunchValid_s1 && !readLaunchReplCAMUse_s1) ||
      (s2_valid && !s2_replCAMUse) || (s3_valid && !s3_replCAMUse) ||
      (s4_valid && !s4_replCAMUse)
  } else {
    replStall :=
      (readLaunchValid_s1 && readLaunchIsRepl_s1) ||
      (readLaunchValid_s1 && readLaunchIsDirRd_s1) ||
      (s2_valid && s2_isRepl) ||
      (s3_valid && s3_isRepl) ||
      (s4_valid && s4_isRepl) ||
      (s2_valid && s2_isDirRd) ||
      (s4_valid && s4_isDirRd && hit_s4)
  }

  val invalidVec = VecInit(dirData_s4.map(_.meta.state === L2Directory.MetaState.I))
  val hasInvalid = invalidVec.asUInt.orR
  val invalidWay = PriorityEncoder(invalidVec)
  val replWay    = if (useRRIP) rrip.get.get_replace_way(s4_repl)
                   else         PLRU4.getReplaceWay(s4_repl(PLRU4.nBits - 1, 0))
  val chosenWay  = Mux(hasInvalid, invalidWay, replWay)

  val currentSetMatchS4 = VecInit(lockTable.indices.map { i =>
    lockTable(i).valid && !refillThisEntryVec(i) && lockTable(i).set === s4_set
  })
  val currentLockMask_s4 = VecInit((0 until ways).map { w =>
    val tableMatch = balancedOr(lockTable.indices.map { i =>
      currentSetMatchS4(i) && lockTable(i).way === w.U
    })
    val pendingMatch = refillPending_s4 &&
      refillPendingSet_s4 === s4_set &&
      refillPendingWay_s4 === w.U
    tableMatch || pendingMatch
  }).asUInt

  def balancedOr(bits: Seq[Bool]): Bool = {
    if (bits.isEmpty) {
      false.B
    } else if (bits.length == 1) {
      bits.head
    } else {
      val mid = bits.length / 2
      balancedOr(bits.take(mid)) || balancedOr(bits.drop(mid))
    }
  }

  // Bypass the pending refill only for its physical table slot. Other entries
  // compare directly, avoiding an effective-entry mux before every comparison.
  val storedConflictVec = VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
    val lockSet = requestSet(req)
    val lockWay = req.REPL_LOCK_WAY
    val tableConflict = balancedOr(
      lockTable.zipWithIndex.filter { case (_, j) => j != i }.map {
        case (entry, j) =>
          entry.valid && !refillThisEntryVec(j) &&
          entry.set === lockSet && entry.way === lockWay
      })
    val pendingConflict = refillPending_s4 &&
      refillPendingTshr_s4 =/= i.U &&
      refillPendingSet_s4 === lockSet &&
      refillPendingWay_s4 === lockWay
    tableConflict || pendingConflict
  })

  val eligibleVec = VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
    req.ReplLockCandidate && !storedConflictVec(i)
  })

  val sameCycleConflictVec = VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
    val lockSet = requestSet(req)
    val lockWay = req.REPL_LOCK_WAY
    if (i == 0) {
      false.B
    } else {
      balancedOr((0 until i).map { j =>
        val earlier = io.toDir(j)
        eligibleVec(j) && requestSet(earlier) === lockSet &&
          earlier.REPL_LOCK_WAY === lockWay
      })
    }
  })

  val replLockConflictVec = VecInit(storedConflictVec.zip(sameCycleConflictVec).map {
    case (stored, sameCycle) => stored || sameCycle
  })

  // A simultaneous ReplLock always wins its way against ReplRd. Other existing
  // owners keep their earlier reservation and divert a later ReplLock to prefill.
  val rawLockSetMatchS4 = VecInit(io.toDir.map { req =>
    req.ReplLockCandidate && requestSet(req) === s4_set
  })
  val replLockMask_s4 = VecInit((0 until ways).map { w =>
    VecInit(io.toDir.indices.map { i =>
      rawLockSetMatchS4(i) && io.toDir(i).REPL_LOCK_WAY === w.U
    }).asUInt.orR
  }).asUInt

  // The S2 mask can be stale under a back-to-back refill. Always exclude ways
  // that are locked now, independent of the pipeline-captured free-way mask.
  val victimMask_s4 = freeWayMask_s4 & ~currentLockMask_s4 & ~replLockMask_s4

  val victimWay  = Mux(victimMask_s4(chosenWay), chosenWay, PriorityEncoder(victimMask_s4))
  val replRetry  = s4_valid && s4_isRepl && (!victimMask_s4.orR || s4_replCAMStale)
  s2_replCAMStale := s2_replCAMHit && replRetry && s4_replCAMAlloc && s4_replCAMIndex === s2_replCAMIndex
  s3_replCAMStaleNow := s3_replCAMHit && replRetry && s4_replCAMAlloc && s4_replCAMIndex === s3_replCAMIndex

  val way_s4   = Mux(s4_isRepl, victimWay, Mux(hit_s4, hitWay, 0.U))
  val entry_s4 = dirData_s4(way_s4)
  val metaValid = (s4_isDirRd && hit_s4) || (s4_isRepl && !replRetry)

  // An Invalid entry must never carry client presence: (I, clients!=0) means a live client
  // copy orphaned from the directory (inclusion breach). Readers rely on state==I implying
  // no eviction work -- the REQ vPipe's evict_victim_invalid skip consumes this on ReplRd.
  when (s4_valid && metaValid) {
    assert(!(entry_s4.meta.state === L2Directory.MetaState.I && entry_s4.meta.clients.asUInt.orR),
      "Directory: read response entry is Invalid but carries client presence")
  }

  val updateRefill = s4_valid && s4_isRepl && !replRetry
  val replWen = (s4_valid && s4_isDirRd && hit_s4 && s4_rrip_update) || updateRefill
  val rrip_way_oh = UIntToOH(way_s4, ways)
  val rrip_prefetch = Mux(s4_isRepl, s4_rrip_refill_prefetch, s4_rrip_prefetch)
  val rrip_req_type = Cat(s4_origin(way_s4), false.B, rrip_prefetch, s4_isRepl)
  val psel = if (useDRRIP) Some(RegInit(512.U(10.W))) else None
  def rrip_match_a(set: UInt): Bool = {
    val halfSetBits = setBits >> 1
    set(setBits - 1, halfSetBits) === set(setBits - halfSetBits - 1, 0)
  }
  def rrip_match_b(set: UInt): Bool = {
    val halfSetBits = setBits >> 1
    set(setBits - 1, halfSetBits) === (~set(setBits - halfSetBits - 1, 0))
  }
  def rrip_policy_for_set(set: UInt): Bool = {
    if (useDRRIP) {
      Mux(rrip_match_a(set), false.B, Mux(rrip_match_b(set), true.B, psel.get(9)))
    } else {
      false.B
    }
  }
  if (useDRRIP) {
    when(updateRefill && rrip_match_a(s4_set) && !hit_s4 && (psel.get =/= 1023.U)) {
      psel.get := psel.get + 1.U
    }.elsewhen(updateRefill && rrip_match_b(s4_set) && !hit_s4 && (psel.get =/= 0.U)) {
      psel.get := psel.get - 1.U
    }
  }
  val rrip_policy = rrip_policy_for_set(s4_set)
  def releaseNextState(base: UInt, set: UInt, way: UInt, origin: Bool, prefetch: Bool): UInt = {
    if (useRRIPRelease) {
      val reqType = Cat(origin, true.B, prefetch, false.B)
      val wayOH = UIntToOH(way, ways)
      if (useDRRIP) {
        rrip.get.get_next_state(base, wayOH, true.B, false.B, rrip_policy_for_set(set), reqType)
      } else {
        rrip.get.get_next_state(base, wayOH, true.B, false.B, reqType)
      }
    } else {
      0.U(replBits.W)
    }
  }
  val replWriteValid = WireDefault(false.B)
  val replWriteRow = WireDefault(0.U(replBits.W))
  val replWriteSet = WireDefault(0.U(setBits.W))
  val replWriteMask = WireDefault(0.U(ways.W))
  def writeReplBacking(valid: Bool, row: UInt, set: UInt, waymask: UInt): Unit = {
    if (useRRIPSram) {
      replWriteValid := valid
      replWriteRow := row
      replWriteSet := set
      replWriteMask := waymask
    }
  }
  val originWriteValid = WireDefault(false.B)
  val originWriteRow = WireDefault(0.U(ways.W))
  val originWriteSet = WireDefault(0.U(setBits.W))
  val originWriteMask = WireDefault(0.U(ways.W))
  def writeOriginBacking(valid: Bool, row: UInt, set: UInt, waymask: UInt): Unit = {
    if (useOriginSram) {
      originWriteValid := valid
      originWriteRow := row
      originWriteSet := set
      originWriteMask := waymask
    } else {
      when (valid) {
        originFF.get(set) := row
      }
    }
  }

  val replWriteValidLaunch_s1 = RegInit(false.B)
  replWriteValidLaunch_s1 := replWriteValid
  val replWriteRowLaunch_s1 = RegInit(0.U(replBits.W))
  replWriteRowLaunch_s1 := replWriteRow
  val replWriteSetLaunch_s1 = RegInit(0.U(setBits.W))
  replWriteSetLaunch_s1 := replWriteSet
  val replWriteMaskLaunch_s1 = RegInit(0.U(ways.W))
  replWriteMaskLaunch_s1 := replWriteMask
  val originWriteValidLaunch_s1 = RegInit(false.B)
  originWriteValidLaunch_s1 := originWriteValid
  val originWriteRowLaunch_s1 = RegInit(0.U(ways.W))
  originWriteRowLaunch_s1 := originWriteRow
  val originWriteSetLaunch_s1 = RegInit(0.U(setBits.W))
  originWriteSetLaunch_s1 := originWriteSet
  val originWriteMaskLaunch_s1 = RegInit(0.U(ways.W))
  originWriteMaskLaunch_s1 := originWriteMask
  if (useRRIPSram) {
    val lanes = VecInit((0 until ways).map(w => replWriteRowLaunch_s1(2 * w + 1, 2 * w)))
    replSRAM.get.io.w(replWriteValidLaunch_s1, lanes, replWriteSetLaunch_s1, replWriteMaskLaunch_s1)
  }
  if (useOriginSram) {
    val lanes = VecInit((0 until ways).map(w => originWriteRowLaunch_s1(w)))
    originSRAM.get.io.w(originWriteValidLaunch_s1, lanes, originWriteSetLaunch_s1, originWriteMaskLaunch_s1)
  }
  writeLaunchPending := tagWriteValidLaunch_s1 || metaWriteValidLaunch_s1 ||
    replWriteValidLaunch_s1 || originWriteValidLaunch_s1

  assert(!(readLaunchValid_s1 && (tagWriteValidLaunch_s1 || metaWriteValidLaunch_s1)),
    "Directory: single-port tag/meta read/write launch overlap")
  if (useRRIPSram) {
    assert(!(replReadValidLaunch_s1 && replWriteValidLaunch_s1),
      "Directory: single-port replacement read/write launch overlap")
  }
  if (useOriginSram) {
    assert(!(originReadValidLaunch_s1 && originWriteValidLaunch_s1),
      "Directory: single-port origin read/write launch overlap")
  }

  val replNextState = Wire(UInt(replBits.W))
  if (useDRRIP) {
    replNextState := rrip.get.get_next_state(s4_repl, rrip_way_oh, s4_isDirRd, hasInvalid, rrip_policy, rrip_req_type)
  } else if (useRRIP) {
    replNextState := rrip.get.get_next_state(s4_repl, rrip_way_oh, s4_isDirRd, hasInvalid, rrip_req_type)
  } else {
    replNextState := PLRU4.getNextState(s4_repl(PLRU4.nBits - 1, 0), way_s4).pad(replBits)
  }
  val nextOrigin = if (useRRIP) {
    Mux(s4_isDirRd, s4_origin | rrip_way_oh, s4_origin & ~rrip_way_oh)
  } else {
    s4_origin
  }
  // Normal replacement decisions are made at S4; DirRel has its separate
  // commit path below. A CAM-resident update writes the next row into the entry.
  // A nonresident update captures its complete backing bundle for the pending
  // commit stage: ReplRd replaces the full row, while a DirRd hit changes only
  // its selected way. In mixed mode a CAM-resident originFF update remains
  // immediate, while a nonresident update uses the same pending stage as RRPV.
  // A retrying allocating ReplRd invalidates its new entry; a younger stale
  // ReplRd retries, while a DirRd update can fall back to a pending masked write.
  //
  // Same-set S3 and S2 CAM hits consume an older S4 next-state rather than a
  // stale registered snapshot. Both forwarding paths are combinational and add
  // no stall. Entries still allocating in launch/S2/S3/S4 are excluded from
  // zero-owner writeback because their first row is not resolved.
  s3_forwardRepl := Mux(replWen && s4_replCAMUse && s4_set === s3_set &&
    s4_replCAMIndex === s3_replCAMIndex, replNextState, s3_repl_reg)
  s3_forwardOrigin := Mux(replWen && s4_replCAMUse && s4_set === s3_set &&
    s4_replCAMIndex === s3_replCAMIndex, nextOrigin, s3_origin_reg)
  s2_forwardRepl := Mux(replWen && s4_replCAMUse && s4_set === s2_set &&
    s4_replCAMIndex === s2_replCAMIndex, replNextState, s2_repl_reg)
  s2_forwardOrigin := Mux(replWen && s4_replCAMUse && s4_set === s2_set &&
    s4_replCAMIndex === s2_replCAMIndex, nextOrigin, s2_origin_reg)
  directBackingCapture := replWen && useRRIPSram.B && !s4_replCAMUse
  writePending_s4 := directBackingCapture
  if (useRRIPSram) {
    val directWriteMask = Mux(s4_isRepl, Fill(ways, true.B), rrip_way_oh)
    when (directBackingCapture) {
      writePendingSet_s4 := s4_set
      writePendingData_s4 := replNextState
      writePendingOrigin_s4 := nextOrigin
      writePendingMask_s4 := directWriteMask
    }
    writeReplBacking(writePending_s4, writePendingData_s4, writePendingSet_s4, writePendingMask_s4)
    writeOriginBacking(writePending_s4, writePendingOrigin_s4, writePendingSet_s4, writePendingMask_s4)
    when (replWen) {
      when (s4_replCAMUse) {
        replCAM.get(s4_replCAMIndex).repl := replNextState
        if (useOriginSram) {
          replCAM.get(s4_replCAMIndex).origin.get := nextOrigin
        } else {
          writeOriginBacking(true.B, nextOrigin, s4_set, directWriteMask)
        }
      }
    }
    when (s4_valid && s4_isRepl && replRetry && s4_replCAMAlloc) {
      replCAM.get(s4_replCAMIndex).valid := false.B
    }
  } else {
    when (replWen) {
      replFF.get(s4_set) := replNextState
      if (useRRIP) {
        originFF.get(s4_set) := nextOrigin
      }
    }
  }

  // Every successful allocating ReplRd acquires a refill lock and increments
  // the resident entry's ownerCount. A request hit does not make the allocation
  // lock-free: conflict-hit and prefill flows can select a different way while
  // older metadata is still present. DirRd updates do not own the entry; DirWb
  // performs no replacement-algorithm update; DirRel is also a non-owner.
  // Consequently ownerCount is not a count of all CAM users or TSHR references.
  // A matching DirWb normally decrements at grant. A final owner also writes the
  // complete RRPV row and, only in full-SRAM mode, origin row and invalidates the
  // entry at that same edge. NeverUnlock suppresses owner retirement as before.
  // Mixed mode needs no origin writeback because originFF is already current.
  // pendClearApply retains only the existing lock-clear timing.
  val refillLockAcquire = s4_valid && s4_isRepl && !replRetry
  val replLockVec = VecInit(io.toDir.map(_.ReplLock))
  refillPending_s4 := refillLockAcquire && !replLockVec(s4_tshr)
  when (refillLockAcquire) {
    refillPendingTshr_s4 := s4_tshr(idxW - 1, 0)
    refillPendingSet_s4 := s4_set
    refillPendingWay_s4 := way_s4
  }
  val dirWbUnlock = grantDirWb && Mux1H(winDirWbOH, dirWbUnlockVec)
  val dirWbOwnerRetire = dirWbUnlock &&
    blockRefillFaultMode =/= L2DirectoryBlockRefillFaultMode.NeverUnlock.U
  val dirWbFinal = dirWbOwnerRetire && dirWbSelectedFinal
  val dirWbReplCAMMatchNow = VecInit((0 until nMSHR).map { camIndex =>
    balancedOr(io.toDir.indices.map { requester =>
      winDirWbOH(requester) && requestReplCAMMatchVec(requester)(camIndex)
    })
  })
  val dirWbReplCAMIndexNow = balancedOneHotMux(winDirWbOH, dirWbReplCAMIndexVec.toSeq)
  pendClearApply := pendClearValid &&
    blockRefillFaultMode =/= L2DirectoryBlockRefillFaultMode.NeverUnlock.U
  if (useRRIPSram) {
    for (i <- 0 until nMSHR) {
      val increment = refillLockAcquire && s4_replCAMUse && s4_replCAMIndex === i.U
      val decrement = dirWbOwnerRetire && dirWbReplCAMMatchNow(i)
      when (increment && !decrement) {
        replCAM.get(i).ownerCount := replCAM.get(i).ownerCount + 1.U
      }.elsewhen (decrement && !increment) {
        assert(replCAM.get(i).ownerCount =/= 0.U, "RRIP CAM owner counter underflow")
        replCAM.get(i).ownerCount := replCAM.get(i).ownerCount - 1.U
      }
    }
  }

  if (useRRIPSram) {
    def allocInFlight(index: UInt): Bool =
      (readLaunchValid_s1 && readLaunchReplCAMAlloc_s1 && readLaunchReplCAMIndex_s1 === index) ||
      (s2_valid && s2_replCAMAlloc && s2_replCAMIndex === index) ||
      (s3_valid && s3_replCAMAlloc && s3_replCAMIndex === index) ||
      (s4_valid && s4_replCAMAlloc && s4_replCAMIndex === index)
    def commitSafe(index: UInt, set: UInt): Bool =
      !directBackingWen && !writeLaunchPending && !allocInFlight(index) &&
        !(readLaunchValid_s1 && readLaunchSet_s1 === set) &&
        !(originReadValidLaunch_s1 && originReadSetLaunch_s1 === set) &&
        !(originReadResponseValid_s1 && originReadSetLaunch_s1 === set) &&
        !(s2_valid && s2_set === set) && !(s3_valid && s3_set === set) &&
        !(s4_valid && s4_set === set) && !relActive

    val finalCandidateIndex = dirWbReplCAMIndexNow
    val finalCandidateSet = replCAM.get(finalCandidateIndex).set
    dirWbCommitBlocked := dirWbSelectedFinal &&
      blockRefillFaultMode =/= L2DirectoryBlockRefillFaultMode.NeverUnlock.U &&
      !commitSafe(finalCandidateIndex, finalCandidateSet)

    val zeroOwnerCandidates = VecInit((0 until nMSHR).map { i =>
      replCAM.get(i).valid && !allocInFlight(i.U) && replCAM.get(i).ownerCount === 0.U
    }).asUInt
    val optionalCommit = grantDirWb && !dirWbFinal && zeroOwnerCandidates.orR
    val commitIndex = Mux(dirWbFinal, dirWbReplCAMIndexNow, PriorityEncoder(zeroOwnerCandidates))
    val commitSet = replCAM.get(commitIndex).set

    // A required final-owner commit takes priority over an unrelated zero-owner
    // candidate. Both use this one DirWb-window write site; an optional candidate
    // is simply skipped when its backing or pipeline hazards are not clear.
    replCAMCommitFire := (dirWbFinal || optionalCommit) && commitSafe(commitIndex, commitSet)
    when (dirWbFinal) {
      assert(replCAMCommitFire, "final-owner DirWb grant without its CAM commit")
    }
    assert(!(dirRelCommitFire && replCAMCommitFire),
      "DirRel commit overlapped a DirWb CAM commit")
    when (replCAMCommitFire) {
      writeReplBacking(true.B, replCAM.get(commitIndex).repl, commitSet, Fill(ways, true.B))
      if (useOriginSram) {
        writeOriginBacking(true.B, replCAM.get(commitIndex).origin.get, commitSet, Fill(ways, true.B))
      }
      replCAM.get(commitIndex).valid := false.B
    }
  }

  // DirRel rechecks CAM residency at commit instead of relying only on the R
  // snapshot. In the SRAM backend, a hit updates the shared row without
  // acquiring ownership. A CAM miss performs selected-way backing writes and
  // never allocates an entry; relBase is zero and no RRPV SRAM read is performed.
  // Full-SRAM mode uses the origin row read at R on a miss, while mixed mode
  // accesses originFF directly. Full-FF mode updates replFF/originFF directly.
  // DirRel never reads Tag/Meta, regardless of the backend or CAM residency.
  val relCommitReplCAMMatchNow = replCAMMatch(relCommitSet)
  val relCommitReplCAMHitNow = relCommitReplCAMMatchNow.orR
  val relCommitReplCAMIndexNow = PriorityEncoder(relCommitReplCAMMatchNow)
  val relOriginBacking = if (useOriginSram) {
    Mux(relOriginHoldValid.get, relOriginHold.get, originSramRead_s2.get)
  } else {
    originFF.get(relCommitSet)
  }
  val relReplCAMOrigin = if (useOriginSram) replCAM.get(relCommitReplCAMIndexNow).origin.get else 0.U(ways.W)
  val relOrigin = if (useRRIPSram) {
    Mux(relCommitReplCAMHitNow && useOriginSram.B, relReplCAMOrigin(relCommitWay), relOriginBacking(relCommitWay))
  } else {
    originFF.get(relCommitSet)(relCommitWay)
  }
  val relBase = if (useRRIPSram) {
    Mux(relCommitReplCAMHitNow, replCAM.get(relCommitReplCAMIndexNow).repl, 0.U(replBits.W))
  } else {
    replFF.get(relCommitSet)
  }
  val relOriginBase = if (useRRIPSram) {
    Mux(relCommitReplCAMHitNow && useOriginSram.B, relReplCAMOrigin, relOriginBacking)
  } else {
    originFF.get(relCommitSet)
  }
  // Local-invalid releases explicitly assume meta.prefetch=0. DirRel never
  // reads Tag or meta arrays to recover provenance; valid-local uses the
  // acceptance-time TSHR-local value captured by the proxy.
  val relPrefetchValue = Mux(relCommitLocalValid, relCommitPrefetch, false.B)
  val relNextState = releaseNextState(relBase, relCommitSet, relCommitWay, relOrigin, relPrefetchValue)
  val relNextOrigin = relOriginBase | UIntToOH(relCommitWay, ways)
  when (dirRelCommitFire) {
    if (useRRIPSram) {
      when (relCommitReplCAMHitNow) {
        replCAM.get(relCommitReplCAMIndexNow).repl := relNextState
      }.otherwise {
        writeReplBacking(true.B, relNextState, relCommitSet, UIntToOH(relCommitWay, ways))
      }
      if (useOriginSram) {
        when (relCommitReplCAMHitNow) {
          replCAM.get(relCommitReplCAMIndexNow).origin.get := relNextOrigin
        }.otherwise {
          writeOriginBacking(true.B, relNextOrigin, relCommitSet, UIntToOH(relCommitWay, ways))
        }
      } else {
        originFF.get(relCommitSet) := relNextOrigin
      }
    } else {
      replFF.get(relCommitSet) := relNextState
      if (useRRIP) {
        originFF.get(relCommitSet) := relNextOrigin
      }
    }
    relActive := false.B
  }
  val dirRelFire = dirRelCommitFire && useRRIPRelease.B

  // Legacy delayed-clear structure. Ordinary refill and hit-hold releases now
  // clear directly at the DirWb commit, so pendClearValid remains deasserted.
  when(pendClearValid) {
    pendClearValid := false.B
  }

  when(refillLockAcquire) {
    // Keep this acquisition identical to the CAM ownerCount update: every
    // allocating replacement reserves its way before the fill commits, even
    // when the request hits an older valid entry. A TSHR may only (re)take its
    // slot once the previous lock is gone; the
    // slot being vacated at this same edge counts as available
    val selfClearing = pendClearApply && pendClearIdx === s4_tshr(idxW - 1, 0)
    assert(!effectiveLockTable(s4_tshr).valid || selfClearing,
      "Directory: TSHR took a second refill lock while one is still held")
    assert(!VecInit(effectiveLockTable.map(e => e.valid && e.set === s4_set && e.way === way_s4)).asUInt.orR,
      "Directory: double refill lock on (set,way)")
  }

  // Release at the owner's fill-commit DirWb grant: ownership is structural
  // per TSHR and exact set/way. A non-owner or meta-only write never releases.
  // An older refill may still retire replCAM ownership after a same-slot
  // hit-hold handoff, but must not clear the new hold.
  val dirWbHitLockReleaseVec = VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
    val lock = effectiveLockTable(i)
    req.REPL_LOCK_RELEASE && lock.valid && lock.hitHold &&
      lock.set === requestSet(req) && lock.way === req.REPL_LOCK_WAY
  })
  val dirWbHitLockRelease = grantDirWb && Mux1H(winDirWbOH, dirWbHitLockReleaseVec)

  when (grantDirWb && winReq.REPL_LOCK_RELEASE) {
    assert(dirWbHitLockRelease, "Directory: ReplLock release without matching hit-hold ownership")
    assert(winReq.WAY === winReq.REPL_LOCK_WAY, "Directory: ReplLock release way mismatch")
    when (winReq.TAG_WEN) {
      assert(dirWbUnlock, "Directory: ReplLock release without final tag commit")
    }
  }

  io.toDir.zipWithIndex.foreach { case (req, i) =>
    val lockSet = requestSet(req)
    val lockWay = req.REPL_LOCK_WAY
    val selfClearing = pendClearApply && pendClearIdx === i.U
    val selfCompatible = !effectiveLockTable(i).valid || selfClearing ||
                         !effectiveLockTable(i).hitHold &&
                         effectiveLockTable(i).set === lockSet && effectiveLockTable(i).way === lockWay

    when (req.ReplLock) {
      assert(!replLockConflictVec(i), "Directory: ReplLock conflict was not diverted to prefill")
      assert(selfCompatible, "Directory: ReplLock overwrote incompatible TSHR ownership")
      // Cancel both an older pending clear and a clear scheduled by this
      // cycle's old refill release, so handoff has no unlocked interval.
      when ((pendClearValid && pendClearIdx === i.U) ||
            ((dirWbUnlock || dirWbHitLockRelease) && winDirWb === i.U)) {
        pendClearValid := false.B
      }
    }
  }

  lockTable.zipWithIndex.foreach { case (lock, i) =>
    val legacyClear = pendClearValid && pendClearIdx === i.U &&
      (blockRefillFaultMode =/= L2DirectoryBlockRefillFaultMode.NeverUnlock.U)
    val refillThisEntry = refillThisEntryVec(i)
    val wbUnlockThisEntry = grantDirWb && winDirWbOH(i) &&
      dirWbUnlockVec(i) && !effectiveLockTable(i).hitHold
    val hitReleaseThisEntry = grantDirWb && winDirWbOH(i) &&
      dirWbHitLockReleaseVec(i)

    when(legacyClear) {
      lock.valid := false.B
    }

    when(refillThisEntry) {
      lock.valid := true.B
      lock.set := refillPendingSet_s4
      lock.way := refillPendingWay_s4
      lock.hitHold := false.B
    }

    when(wbUnlockThisEntry || hitReleaseThisEntry) {
      lock.valid := false.B
    }

    when(io.toDir(i).ReplLock) {
      lock.valid := true.B
      lock.set := requestSet(io.toDir(i))
      lock.way := io.toDir(i).REPL_LOCK_WAY
      lock.hitHold := true.B
    }
  }

  effectiveLockTable.zipWithIndex.foreach { case (e, i) =>
    assert(!(e.valid && io.tshrIdle(i)),
      s"Directory: blockRefill leaked after owner MSHR retired at TSHR $i")
  }

  if (enableDirPlruProbe) {
    io.debugRrip.get.valid := replWen
    io.debugRrip.get.set := s4_set
    io.debugRrip.get.state := Mux(replWen, replNextState, s4_repl)
    io.debugRrip.get.s4Valid := s4_valid
    io.debugRrip.get.s4Tshr := s4_tshr
    io.debugRrip.get.s4IsDirRd := s4_isDirRd
    io.debugRrip.get.s4IsRepl := s4_isRepl
    io.debugRrip.get.wayS4 := way_s4
    io.debugRrip.get.s4Rrip := s4_repl
    io.debugRrip.get.currentSetRrip := s4_repl
    for (i <- 0 until nMSHR) {
      if (useRRIPSram) {
        io.debugReplCAM.get(i).valid := replCAM.get(i).valid
        io.debugReplCAM.get(i).set := replCAM.get(i).set
        io.debugReplCAM.get(i).repl := replCAM.get(i).repl
        io.debugReplCAM.get(i).ownerCount := replCAM.get(i).ownerCount
        if (useOriginSram) {
          io.debugReplCAM.get(i).origin.get := replCAM.get(i).origin.get
        } else {
          io.debugReplCAM.get(i).origin.get := originFF.get(replCAM.get(i).set)
        }
      } else {
        io.debugReplCAM.get(i) := 0.U.asTypeOf(io.debugReplCAM.get(i))
      }
    }
    io.debugRetry.get.replStall := replStall
    io.debugRetry.get.s2Valid := s2_valid
    io.debugRetry.get.s2Tshr := s2_tshr
    io.debugRetry.get.s2IsDirRd := s2_isDirRd
    io.debugRetry.get.s2IsRepl := s2_isRepl
    io.debugRetry.get.s2Set := s2_set
    io.debugRetry.get.s4Valid := s4_valid
    io.debugRetry.get.s4Tshr := s4_tshr
    io.debugRetry.get.s4IsDirRd := s4_isDirRd
    io.debugRetry.get.s4IsRepl := s4_isRepl
    io.debugRetry.get.s4Set := s4_set
    io.debugRetry.get.hasInvalid := hasInvalid
    io.debugRetry.get.invalidWay := invalidWay
    io.debugRetry.get.hitWay := hitWay
    io.debugRetry.get.replWay := replWay
    io.debugRetry.get.chosenWay := chosenWay
    io.debugRetry.get.victimWay := victimWay
    io.debugRetry.get.blockRefillMaskS2 := blockRefillMask_s2
    io.debugRetry.get.freeWayMaskS4 := freeWayMask_s4
    io.debugRetry.get.dirWbDone := grantDirWb
    io.debugRetry.get.dirWbDoneSet := reqSet
    io.debugRetry.get.dirWbDoneWay := winReq.WAY(wayBits - 1, 0)
    // re-synthesized per-(set,way) view of the CAM lock table (probe-only, default off)
    io.debugRetry.get.blockRefillLocked := VecInit((0 until sets).map { s =>
      VecInit((0 until ways).map { w =>
        VecInit(effectiveLockTable.map(e => e.valid && e.set === s.U && e.way === w.U)).asUInt.orR
      }).asUInt
    })
    io.debugRetry.get.ageWb := ageWb
    io.debugRetry.get.ageRd := ageRd
    io.debugRetry.get.rrReplRd := rrReplRd
    io.debugRetry.get.rrDirWb := rrDirWb
    io.debugRetry.get.rrDirRd := rrDirRd
    io.debugRetry.get.replRegs := replFF.getOrElse(VecInit(Seq.fill(sets)(0.U(replBits.W))))
    io.debugRetry.get.originRegs := originFF.getOrElse(VecInit(Seq.fill(sets)(0.U(ways.W))))
    io.debugRetry.get.psel := psel.getOrElse(0.U)
    val dirRelDebug = io.debugRetry.get.dirRelScheduling
    dirRelDebug.grant := dirRelGrant
    dirRelDebug.active := relActive
    dirRelDebug.commit := dirRelCommitFire
    dirRelDebug.wait_cycles := dirRelWait
    dirRelDebug.retry := dirRelRetry
    dirRelDebug.block_normal := dirRelBlockNormal
    dirRelDebug.block_pipeline := dirRelBlockPipeline
    dirRelDebug.block_conflict := dirRelBlockConflict
    dirRelDebug.block_cam_commit := dirRelBlockCAMCommit
    dirRelDebug.block_backing := dirRelBlockBacking
    dirRelDebug.normal_block := normalBlockDirRel
    dirRelDebug.dir_rd_block := dirRdBlockDirRel
    dirRelDebug.dir_wb_block := dirWbBlockDirRel
    dirRelDebug.repl_rd_block := replRdBlockDirRel
    dirRelDebug.normal_block_winning := normalBlockDirRelWinning
    dirRelDebug.dir_rd_block_winning := dirRdBlockDirRelWinning
    dirRelDebug.dir_wb_block_winning := dirWbBlockDirRelWinning
    dirRelDebug.repl_rd_block_winning := replRdBlockDirRelWinning
    dirRelDebug.normal_block_inflight := normalBlockDirRelInflight
    dirRelDebug.dir_rd_block_inflight := dirRdBlockDirRelInflight
    dirRelDebug.dir_wb_block_inflight := dirWbBlockDirRelInflight
    dirRelDebug.repl_rd_block_inflight := replRdBlockDirRelInflight
  }


  // ══════════════════════════════════════════════════════════════════
  // Response (one-to-one per TSHR)
  // ══════════════════════════════════════════════════════════════════
  io.fromDir.zipWithIndex.foreach { case (o, i) =>
    o.TSHRID         := i.U
    o.META           := 0.U.asTypeOf(new L2Directory.MetaReadResult)
    o.REPL           := 0.U.asTypeOf(new L2Directory.ReplReadResult)
    o.DirRdArbComp   := false.B
    o.DirRdResp      := false.B
    o.DirWbArbComp   := false.B
    o.ReplRdArbComp  := false.B
    o.ReplRdResp     := false.B
    o.ReplRdRetryAck := false.B
    o.DirRelArbComp  := false.B
    o.ReplLockConflict := replLockConflictVec(i)
  }

  // ArbComp — combinational at grant cycle (N) for all three operations.
  // For DirWb the write lands at the edge ending the grant cycle; any access
  // granted from cycle N+1 on observes it, so no delayed completion is needed.
  when(grantDirRd)  { io.fromDir(winIdx).DirRdArbComp  := true.B }
  when(grantReplRd) { io.fromDir(winIdx).ReplRdArbComp := true.B }
  when(grantDirWb)  { io.fromDir(winIdx).DirWbArbComp  := true.B }
  // DirRel completes at grant in single-cycle mode and at W otherwise.
  when(dirRelCommitFire) { io.fromDir(relCommitTshr(idxW - 1, 0)).DirRelArbComp := true.B }

  // Resp at S4 — routed one-to-one to the originating TSHR
  when(s4_valid) {
    val o = io.fromDir(s4_tshr)
    val dirRdEntry_s4 = dirData_s4(Mux(hit_s4, hitWay, 0.U))
    when (s4_isDirRd && hit_s4) {
      o.META.state := dirRdEntry_s4.meta.state
      o.META.dirty := dirRdEntry_s4.meta.dirty
      o.META.clients := dirRdEntry_s4.meta.clients
      o.META.alias := dirRdEntry_s4.meta.alias
      o.META.prefetch := dirRdEntry_s4.meta.prefetch
      o.META.way := hitWay
      o.META.hit := true.B
    }
    when(s4_isRepl) {
      o.ReplRdResp := !replRetry
      o.ReplRdRetryAck := replRetry
      when(!replRetry) {
        o.REPL.paddr := Cat(entry_s4.tag, s4_set, 0.U(offsetBits.W))
        o.REPL.way := way_s4
        o.REPL.meta := entry_s4.meta
      }
    }.otherwise {
      o.DirRdResp := true.B
    }
  }

  // ══════════════════════════════════════════════════════════════════
  // Assertions
  // ══════════════════════════════════════════════════════════════════
  io.toDir.zipWithIndex.foreach { case (req, i) =>
    assert(PopCount(Seq(req.ReplRd, req.DirRd, req.DirWb, req.DirRel)) <= 1.U,
      s"Directory: TSHR $i asserts multiple op types")
  }
  assert(PopCount(Seq(grantReplRd, grantDirWb, grantDirRd)) <= 1.U,
    "Directory: multiple grants in one cycle")
  assert(!(grantReplRd && replStall),
    "Directory: ReplRd granted while replStall (replacement-state hazard)")
  assert(!(grantDirRd && !useRRIPSram.B && replStall),
    "Directory: full-FF DirRd granted while replStall (replacement-state hazard)")
  // Functional requests ARE allowed at the directory ports during the power-on
  // init sweep; they are masked out of arbitration (anyReplRd/anyDirWb/anyDirRd
  // are qualified with !initActive) and simply wait until the sweep completes.
  // The invariant is that no functional request is ever arbitrated during init.
  when(initActive) {
    assert(!anyGrant,
      "Directory: functional traffic must not be arbitrated while directory init is in progress")
  }
  when(s4_valid && s4_isDirRd) {
    assert(PopCount(hitVec) <= 1.U,
      "Directory: multiple ways hit (tag uniqueness)")
  }

  // ══════════════════════════════════════════════════════════════════
  // Perf
  // ══════════════════════════════════════════════════════════════════
  XSPerfAccumulate("grant_dirRd",   grantDirRd)
  XSPerfAccumulate("grant_replRd",  grantReplRd)
  XSPerfAccumulate("grant_dirWb",   grantDirWb)
  XSPerfAccumulate("grant_dirRel",  dirRelGrant)
  XSPerfAccumulate("replStall",     replStall)
  XSPerfAccumulate("replRetry",     replRetry)
  XSPerfAccumulate("blockRefill_locked", PopCount(effectiveLockTable.map(_.valid)))
  XSPerfAccumulate("dirRd_hit",     s4_valid && s4_isDirRd && hit_s4)
  XSPerfAccumulate("dirRd_miss",    s4_valid && s4_isDirRd && !hit_s4)
  XSPerfAccumulate("repl_upd",      replWen)
  XSPerfAccumulate("repl_rel_upd",     dirRelFire)
  XSPerfAccumulate("dir_init_active", initActive)
  XSPerfAccumulate("starve_wb",     wbStarved)
  XSPerfAccumulate("starve_rd",     rdStarved)
  // Cycles a pending/active DirRel cannot advance at R or W; init-masked and disabled paths are excluded.
  XSPerfAccumulate("dirRel_wait", dirRelWait)
  // Retry events aborted by relConflict after two-cycle R; single-cycle grants and successful W commits never count.
  XSPerfAccumulate("dirRel_retry", dirRelRetry)
  // R/W wait cycles where a real normal grant is the sole remaining slot blocker; independent hazards are excluded.
  XSPerfAccumulate("dirRel_block_normal", dirRelBlockNormal)
  // R/W wait cycles with older normal S2/S3/S4 occupancy; an idle pipeline never increments this counter.
  XSPerfAccumulate("dirRel_block_pipeline", dirRelBlockPipeline)
  // Two-cycle W waits with same-set/CAM-lifetime conflict; R waits and single-cycle updates never count.
  XSPerfAccumulate("dirRel_block_conflict", dirRelBlockConflict)
  // Two-cycle W waits with an actual DirWb-window CAM commit; mere commit eligibility does not count.
  XSPerfAccumulate("dirRel_block_cam_commit", dirRelBlockCAMCommit)
  // Two-cycle W waits with an actual direct replacement-backing write; CAM-resident updates do not count.
  XSPerfAccumulate("dirRel_block_backing", dirRelBlockBacking)
  // Cycles with a real normal grant suppressed in either DirRel phase; idle/draining/independently blocked normal work and dirrel-low are excluded.
  XSPerfAccumulate("normal_block_dirRel", normalBlockDirRel)
  // Cycles a would-be-winning DirRd loses a real grant to either DirRel phase; normal-group losers and independent hazards are excluded.
  XSPerfAccumulate("dirRd_block_dirRel", dirRdBlockDirRel)
  // Cycles a would-be-winning DirWb loses a real grant to either DirRel phase; normal-group losers and independent hazards are excluded.
  XSPerfAccumulate("dirWb_block_dirRel", dirWbBlockDirRel)
  // Cycles a would-be-winning ReplRd loses a real grant to either DirRel phase; normal-group losers and independent hazards are excluded.
  XSPerfAccumulate("replRd_block_dirRel", replRdBlockDirRel)
  // Cycles a class-winning, pre-inflight DirRel suppresses a real normal grant; idle/commit-blocked/independent stalls and dirrel-low are excluded.
  XSPerfAccumulate("normal_block_dirRel_winning", normalBlockDirRelWinning)
  // Winning-phase cycles suppressing a real would-be-winning DirRd grant; in-flight W and non-grantable DirRd requests are excluded.
  XSPerfAccumulate("dirRd_block_dirRel_winning", dirRdBlockDirRelWinning)
  // Winning-phase cycles suppressing a real would-be-winning DirWb grant; in-flight W and non-grantable DirWb requests are excluded.
  XSPerfAccumulate("dirWb_block_dirRel_winning", dirWbBlockDirRelWinning)
  // Winning-phase cycles suppressing a real would-be-winning ReplRd grant; in-flight W and non-grantable ReplRd requests are excluded.
  XSPerfAccumulate("replRd_block_dirRel_winning", replRdBlockDirRelWinning)
  // Cycles an accepted two-cycle DirRel awaiting W suppresses a real normal grant; idle/single-cycle/dirrel-low traffic is excluded.
  XSPerfAccumulate("normal_block_dirRel_inflight", normalBlockDirRelInflight)
  // In-flight-W cycles suppressing a real would-be-winning DirRd grant; winning/R, single-cycle, and idle-normal cycles are excluded.
  XSPerfAccumulate("dirRd_block_dirRel_inflight", dirRdBlockDirRelInflight)
  // In-flight-W cycles suppressing a real would-be-winning DirWb grant; winning/R, single-cycle, and idle-normal cycles are excluded.
  XSPerfAccumulate("dirWb_block_dirRel_inflight", dirWbBlockDirRelInflight)
  // In-flight-W cycles suppressing a real would-be-winning ReplRd grant; winning/R, single-cycle, and idle-normal cycles are excluded.
  XSPerfAccumulate("replRd_block_dirRel_inflight", replRdBlockDirRelInflight)
}
