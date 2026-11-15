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
// Normal winners enter a 3-stage pipeline: S1 decode + tag/meta/replacement read,
// S2 latch, S3 hit/way/result/update. RRIP SRAM mode caches active sets in a
// TSHR-sized CAM; zero-owner entries write back in exclusive Directory slots.
//
// ArbComp: DirRd/ReplRd/DirWb completion is asserted at the grant cycle.
// Responses are driven at S3 and routed one-to-one to fromDir(s3_tshr).
// DirRel instead completes at its W cycle, after replacement and metadata writes.
//
// replStall protects the FF S3-write/S1-read window for both ReplRd and DirRd.
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
        val s3Valid = Bool()
        val s3Tshr = UInt(mshrIndexWidth.W)
        val s3IsDirRd = Bool()
        val s3IsRepl = Bool()
        val wayS3 = UInt(wayBits.W)
        val s3Rrip = UInt(replBits.W)
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
        val s3Valid = Bool()
        val s3Tshr = UInt(log2Ceil(paramL2.mshrSize).W)
        val s3IsDirRd = Bool()
        val s3IsRepl = Bool()
        val s3Set = UInt(setBits.W)
        val hasInvalid = Bool()
        val invalidWay = UInt(wayBits.W)
        val hitWay = UInt(wayBits.W)
        val replWay = UInt(wayBits.W)
        val chosenWay = UInt(wayBits.W)
        val victimWay = UInt(wayBits.W)
        val blockRefillMaskS2 = UInt(paramL2.ways.W)
        val freeWayMaskS3 = UInt(paramL2.ways.W)
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
  //    rows. The entry cannot write back before its allocating S3 has resolved.
  // 2. Same-set hits reuse the entry. An older S3 update forwards directly into
  //    S2; a same-set ReplRd instead waits while the allocation's refill lock is
  //    not yet visible. There is no extra S2 stall for forwarding.
  // 3. Successful S3 operations modify the CAM row when resident, or use masked
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
  val s2_valid = WireDefault(false.B)
  val s2_replCAMAlloc = WireDefault(false.B)
  val s2_set = Wire(UInt(setBits.W))
  val s3_valid = WireDefault(false.B)
  val s3_replCAMAlloc = WireDefault(false.B)
  val s3_set = Wire(UInt(setBits.W))

  val relActive = RegInit(false.B)
  val relSet = Reg(UInt(setBits.W))
  val relWay = Reg(UInt(wayBits.W))
  val relTshr = Reg(UInt(mshrIndexWidth.W))
  val relLocalValid = Reg(Bool())
  val relPrefetch = Reg(Bool())
  val relReplCAMHit = Reg(Bool())
  // Refill lock table: one slot per TSHR, indexed directly by TSHR ID. CAM
  // ownership retires at the DirWb grant; pendClear only delays the lock clear.
  val lockTable = RegInit(VecInit(Seq.fill(nMSHR)(0.U.asTypeOf(new L2Directory.RefillLockEntry))))
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
    (s2_valid && s2_replCAMAlloc && s2_set === set) ||
    (s3_valid && s3_replCAMAlloc && s3_set === set)
  }
  // ReplRd must see the older refill lock before choosing a victim. A CAM-resident
  // DirRd instead uses S3-to-S2 forwarding and needs no allocation-specific stall.
  val replRdVec = VecInit(io.toDir.map(req => req.ReplRd && !replCAMReadBlocked(req)))
  val dirWbVec  = VecInit(io.toDir.map(_.DirWb))
  val dirWbUnlockVec = VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
    val lock = lockTable(i)
    req.DirWb && req.TAG_WEN && lock.valid &&
      lock.set === requestSet(req) && lock.way === req.WAY(wayBits - 1, 0)
  })
  val dirWbReplCAMIndexVec = VecInit(io.toDir.map(req => PriorityEncoder(replCAMMatch(requestSet(req)))))
  // Candidate decode precedes the winner so final-commit readiness can gate the
  // DirWb grant without a combinational path from the grant back to itself.
  val dirWbFinalVec = if (useRRIPSram) {
    VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
      val camMatch = replCAMMatch(requestSet(req))
      dirWbUnlockVec(i) && camMatch.orR &&
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

  // Replacement-read and pipeline hazards are driven after S2/S3 exist.
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
  val replRdWins = anyReplRd && !anyStarved && !replStall && !directBackingWen

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

  val winReplRd = roundRobinNext(rrReplRd, replRdVec.asUInt, nMSHR)
  val winDirWb  = roundRobinNext(rrDirWb,  dirWbVec.asUInt,  nMSHR)
  val winDirRd  = roundRobinNext(rrDirRd,  dirRdVec.asUInt,  nMSHR)

  // Grantable describes the normal-group result without the DirRel class gate.
  // It includes final-commit and replacement/backing hazards so reverse-stall
  // counters identify only real grants suppressed by winning/in-flight DirRel.
  val normalReplRdGrantable = replRdWins && anyReplRd
  val normalDirWbGrantable = !dirWbCommitBlocked && !replRdWins && anyLevel2 && level2WinIsWb && anyDirWb
  // Full-FF DirRd must also observe every older S3 full-row update. SRAM
  // backends retain their existing CAM/direct interlocks instead of this stall.
  val normalDirRdGrantable = !replRdWins && anyLevel2 && !level2WinIsWb && anyDirRd &&
    (useRRIPSram.B || !replStall) && !directBackingWen
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
    !relActive && !anyGrant && !pipelineBusy
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

  val reqSet = winReq.PADDR(offsetBits + setBits - 1, offsetBits)
  val reqTag = winReq.PADDR(paramL2.physicalAddrWidth - 1, offsetBits + setBits)

  // Only a ReplRd CAM miss allocates an entry. S1 establishes valid/set and a
  // zero ownerCount; the allocating request still takes the backing read at S2
  // because its registered s2_replCAMHit is false. A younger same-set DirRd may
  // encounter the valid entry before that allocation reaches S3, so the older
  // allocating S3 forwards its completed row into the younger S2. Same-set
  // ReplRd requests instead remain blocked until the allocation's lock outcome
  // is visible. CAM-full and duplicate-set allocation are assertions, not stalls.
  val s1ReplCAMMatch = replCAMMatch(reqSet)
  val s1ReplCAMHit = s1ReplCAMMatch.orR
  val s1ReplCAMIndex = PriorityEncoder(s1ReplCAMMatch)
  val s1ReplCAMAlloc = isReplRd && useRRIPSram.B && !replCAMMatch(reqSet).orR
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

  val tagReadData_s2 = tagArray.io.r(isRead, reqSet).resp.data
  val metaReadData_s2 = metaArray.io.r(isRead, reqSet).resp.data
  val dirReadData_s2 = Wire(Vec(ways, new L2Directory.DirEntry))
  for (w <- 0 until ways) {
    dirReadData_s2(w).tag := tagReadData_s2(w)
    dirReadData_s2(w).meta := metaReadData_s2(w)
  }

  val s1ReplCAMRepl = if (useRRIPSram) replCAM.get(s1ReplCAMIndexOrAlloc).repl else 0.U(replBits.W)
  val s1ReplCAMOrigin = if (useOriginSram) replCAM.get(s1ReplCAMIndexOrAlloc).origin.get else 0.U(ways.W)
  val replState_s1 = Mux(s1ReplCAMHit, s1ReplCAMRepl,
    replFF.map(_(reqSet)).getOrElse(0.U(replBits.W)))
  val originState_s1 = Mux(s1ReplCAMHit && useOriginSram.B, s1ReplCAMOrigin,
    originFF.map(_(reqSet)).getOrElse(0.U(ways.W)))
  val replSramRead_s2 = replSRAM.map(_.io.r(isRead && !s1ReplCAMHit, reqSet).resp.data.asUInt)
  val originReadSet = Mux(relNeedsOrigin, dirRelSet, reqSet)
  val originSramRead_s2 = originSRAM.map(_.io.r(
    (isRead && !s1ReplCAMHit) || relNeedsOrigin, originReadSet).resp.data.asUInt)

  // ── S1 → S2 ───────────────────────────────────────────────────────
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
  val s2_repl = Mux(s2_replCAMHit, s2_forwardRepl,
    if (useRRIPSram) replSramRead_s2.get else s2_repl_reg)
  val s2_origin = Mux(s2_replCAMHit, s2_forwardOrigin,
    if (useOriginSram) originSramRead_s2.get else s2_origin_reg)
  val s2_replCAMStale = Wire(Bool())
  val s2_rrip_update = RegNext(winReq.RRIP_UPDATE, false.B)
  val s2_rrip_prefetch = RegNext(winReq.RRIP_PREFETCH, false.B)
  val s2_rrip_refill_prefetch = RegNext(winReq.RRIP_REFILL_PREFETCH, false.B)

  // ── S2 → S3 ───────────────────────────────────────────────────────
  s3_valid := RegNext(s2_valid, false.B)
  val s3_isRepl = RegNext(s2_isRepl, false.B)
  val s3_isDirRd = RegNext(s2_isDirRd, false.B)
  val s3_tshr = RegNext(s2_tshr, 0.U)
  s3_set := RegNext(s2_set, 0.U)
  val s3_tag = RegNext(s2_tag, 0.U)
  val s3_repl = RegNext(s2_repl, 0.U)
  val s3_origin = RegNext(s2_origin, 0.U)
  val s3_replCAMStale = RegNext(s2_replCAMStale, false.B)
  val s3_replCAMUse = RegNext(s2_replCAMUse, false.B) && !s3_replCAMStale
  s3_replCAMAlloc := RegNext(s2_replCAMAlloc, false.B)
  val s3_replCAMIndex = RegNext(s2_replCAMIndex, 0.U)
  val s3_rrip_update = RegNext(s2_rrip_update, false.B)
  val s3_rrip_prefetch = RegNext(s2_rrip_prefetch, false.B)
  val s3_rrip_refill_prefetch = RegNext(s2_rrip_refill_prefetch, false.B)
  pipelineBusy := s2_valid || s3_valid

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
    (s2_valid && s2_set === relSet) || (s3_valid && s3_set === relSet)
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

  // A full-SRAM R miss returns its origin row in the first W cycle. Immediate W
  // uses that response directly; a delayed W uses this per-request copy so an
  // intervening dirrel-low read cannot replace the row before the RMW commit.
  val relOriginHold = if (useOriginSram) Some(Reg(UInt(ways.W))) else None
  val relOriginHoldValid = if (useOriginSram) Some(RegInit(false.B)) else None
  if (useOriginSram) {
    when (relActive && !relReplCAMHit && !relOriginHoldValid.get) {
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

  tagArray.io.w(
    valid   = tagWriteValid,
    data    = dirWriteEntry.tag,
    setIdx  = dirWriteSet,
    waymask = dirWriteWayMask
  )
  metaArray.io.w(
    valid   = metaWriteValid,
    data    = dirWriteEntry.meta,
    setIdx  = dirWriteSet,
    waymask = dirWriteWayMask,
    bitmask = dirWriteMetaMask
  )

  if (enableDirStateProbe) {
    io.debugStateWrite.get.valid := isDirWb
    io.debugStateWrite.get.set := reqSet
    io.debugStateWrite.get.way := winReq.WAY(wayBits - 1, 0)
    io.debugStateWrite.get.state := winReq.META.state
    io.debugStateWrite.get.dirty := winReq.META.dirty
    io.debugStateWrite.get.client0 := winReq.META.clients(0)
  }

  // DirWb completion is combinational at the grant cycle: the write lands at the
  // edge ending that cycle and is observed by any access granted from the next
  // cycle on (single grant per cycle, latency-1 synchronous SRAM).

  // ══════════════════════════════════════════════════════════════════
  // S3: hit detect, way select, PLRU update
  // ══════════════════════════════════════════════════════════════════
  val dirData_s3 = RegEnable(dirReadData_s2, 0.U.asTypeOf(dirReadData_s2), s2_valid)

  val tagMatch = VecInit(dirData_s3.map(_.tag === s3_tag))
  val validVec = VecInit(dirData_s3.map(_.meta.state =/= L2Directory.MetaState.I))
  val hitVec   = VecInit(tagMatch.zip(validVec).map { case (t, v) => t && v })
  val hit_s3   = hitVec.asUInt.orR
  val hitWay   = PriorityEncoder(hitVec)
  val blockRefillMask_s2 = Wire(UInt(ways.W))
  blockRefillMask_s2 := VecInit((0 until ways).map { w =>
    VecInit(lockTable.map(e => e.valid && e.set === s2_set && e.way === w.U)).asUInt.orR
  }).asUInt
  val freeWayMask_s2 = Wire(UInt(ways.W))
  freeWayMask_s2 := (~blockRefillMask_s2)(ways - 1, 0)
  val freeWayMask_s3 = RegInit(((BigInt(1) << ways) - 1).U(ways.W))
  when(s2_isRepl) {
    freeWayMask_s3 := freeWayMask_s2
  }

  // FF retains the original global RAW guard. The SRAM backend needs it only
  // for direct backing operations; CAM-resident updates can proceed by set.
  if (useRRIPSram) {
    replStall := (s2_valid && !s2_replCAMUse) || (s3_valid && !s3_replCAMUse)
  } else {
    replStall :=
      (s2_valid && s2_isRepl) ||
      (s3_valid && s3_isRepl) ||
      (s2_valid && s2_isDirRd) ||
      (s3_valid && s3_isDirRd && hit_s3)
  }

  val invalidVec = VecInit(dirData_s3.map(_.meta.state === L2Directory.MetaState.I))
  val hasInvalid = invalidVec.asUInt.orR
  val invalidWay = PriorityEncoder(invalidVec)
  val replWay    = if (useRRIP) rrip.get.get_replace_way(s3_repl)
                   else         PLRU4.getReplaceWay(s3_repl(PLRU4.nBits - 1, 0))
  val chosenWay  = Mux(hasInvalid, invalidWay, replWay)

  val currentLockMask_s3 = VecInit((0 until ways).map { w =>
    VecInit(lockTable.map(e => e.valid && e.set === s3_set && e.way === w.U)).asUInt.orR
  }).asUInt

  val replLockConflictVec = VecInit(io.toDir.zipWithIndex.map { case (req, i) =>
    val lockSet = requestSet(req)
    val lockWay = req.REPL_LOCK_WAY
    val otherOwnerLock = VecInit(lockTable.zipWithIndex.filter { case (_, j) => j != i }.map {
      case (entry, _) => entry.valid && entry.set === lockSet && entry.way === lockWay
    }).asUInt.orR
    val earlierReplLock = if (i == 0) false.B else VecInit((0 until i).map { j =>
      val earlier = io.toDir(j)
      earlier.ReplLock && requestSet(earlier) === lockSet && earlier.REPL_LOCK_WAY === lockWay
    }).asUInt.orR
    otherOwnerLock || earlierReplLock
  })

  // A simultaneous ReplLock always wins its way against ReplRd. Other existing
  // owners keep their earlier reservation and divert a later ReplLock to prefill.
  val replLockMask_s3 = VecInit((0 until ways).map { w =>
    VecInit(io.toDir.map { req =>
      req.ReplLock && requestSet(req) === s3_set && req.REPL_LOCK_WAY === w.U
    }).asUInt.orR
  }).asUInt

  // The S2 mask can be stale under a back-to-back refill. Always exclude ways
  // that are locked now, independent of the pipeline-captured free-way mask.
  val victimMask_s3 = freeWayMask_s3 & ~currentLockMask_s3 & ~replLockMask_s3

  val victimWay  = Mux(victimMask_s3(chosenWay), chosenWay, PriorityEncoder(victimMask_s3))
  val replRetry  = s3_valid && s3_isRepl && (!victimMask_s3.orR || s3_replCAMStale)
  s2_replCAMStale := s2_replCAMHit && replRetry && s3_replCAMAlloc && s3_replCAMIndex === s2_replCAMIndex

  val way_s3   = Mux(s3_isRepl, victimWay, Mux(hit_s3, hitWay, 0.U))
  val entry_s3 = dirData_s3(way_s3)
  val metaValid = (s3_isDirRd && hit_s3) || (s3_isRepl && !replRetry)

  // An Invalid entry must never carry client presence: (I, clients!=0) means a live client
  // copy orphaned from the directory (inclusion breach). Readers rely on state==I implying
  // no eviction work -- the REQ vPipe's evict_victim_invalid skip consumes this on ReplRd.
  when (s3_valid && metaValid) {
    assert(!(entry_s3.meta.state === L2Directory.MetaState.I && entry_s3.meta.clients.asUInt.orR),
      "Directory: read response entry is Invalid but carries client presence")
  }

  val updateRefill = s3_valid && s3_isRepl && !replRetry
  val replWen = (s3_valid && s3_isDirRd && hit_s3 && s3_rrip_update) || updateRefill
  val rrip_way_oh = UIntToOH(way_s3, ways)
  val rrip_prefetch = Mux(s3_isRepl, s3_rrip_refill_prefetch, s3_rrip_prefetch)
  val rrip_req_type = Cat(s3_origin(way_s3), false.B, rrip_prefetch, s3_isRepl)
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
    when(updateRefill && rrip_match_a(s3_set) && !hit_s3 && (psel.get =/= 1023.U)) {
      psel.get := psel.get + 1.U
    }.elsewhen(updateRefill && rrip_match_b(s3_set) && !hit_s3 && (psel.get =/= 0.U)) {
      psel.get := psel.get - 1.U
    }
  }
  val rrip_policy = rrip_policy_for_set(s3_set)
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
  def writeReplBacking(valid: Bool, row: UInt, set: UInt, waymask: UInt): Unit = {
    if (useRRIPSram) {
      val lanes = VecInit((0 until ways).map(w => row(2 * w + 1, 2 * w)))
      replSRAM.get.io.w(valid, lanes, set, waymask)
    }
  }
  def writeOriginBacking(valid: Bool, row: UInt, set: UInt, waymask: UInt): Unit = {
    if (useOriginSram) {
      val lanes = VecInit((0 until ways).map(w => row(w)))
      originSRAM.get.io.w(valid, lanes, set, waymask)
    } else {
      when (valid) {
        originFF.get(set) := row
      }
    }
  }
  if (useRRIPSram) {
    replSRAM.get.io.w(false.B, VecInit(Seq.fill(ways)(0.U(2.W))), 0.U(setBits.W), 0.U(ways.W))
  }
  if (useOriginSram) {
    originSRAM.get.io.w(false.B, VecInit(Seq.fill(ways)(false.B)), 0.U(setBits.W), 0.U(ways.W))
  }

  val replNextState = Wire(UInt(replBits.W))
  if (useDRRIP) {
    replNextState := rrip.get.get_next_state(s3_repl, rrip_way_oh, s3_isDirRd, hasInvalid, rrip_policy, rrip_req_type)
  } else if (useRRIP) {
    replNextState := rrip.get.get_next_state(s3_repl, rrip_way_oh, s3_isDirRd, hasInvalid, rrip_req_type)
  } else {
    replNextState := PLRU4.getNextState(s3_repl(PLRU4.nBits - 1, 0), way_s3).pad(replBits)
  }
  val nextOrigin = if (useRRIP) {
    Mux(s3_isDirRd, s3_origin | rrip_way_oh, s3_origin & ~rrip_way_oh)
  } else {
    s3_origin
  }
  // Normal replacement updates modify state at S3; DirRel has its separate
  // commit path below. A CAM-resident update writes the next row into the entry.
  // A nonresident update writes backing directly: ReplRd replaces the full row,
  // while a DirRd hit changes only its selected way. In mixed mode originFF is
  // always updated directly, even when the RRPV row is CAM-resident. A retrying
  // allocating ReplRd invalidates its new entry; a younger stale ReplRd retries,
  // while a DirRd update can fall back to its masked direct-backing write.
  //
  // A same-set S2 CAM hit consumes the older S3 next-state rather than a stale
  // S1 CAM snapshot. This combinational S3-to-S2 forwarding covers an allocation
  // already visible as valid and adds no S2 stall. Without an older S3 write,
  // the registered S1 CAM payload is used. Entries still allocating in S2/S3 are
  // excluded from zero-owner writeback because their first row is not resolved.
  s2_forwardRepl := Mux(replWen && s3_replCAMUse && s3_set === s2_set &&
    s3_replCAMIndex === s2_replCAMIndex, replNextState, s2_repl_reg)
  s2_forwardOrigin := Mux(replWen && s3_replCAMUse && s3_set === s2_set &&
    s3_replCAMIndex === s2_replCAMIndex, nextOrigin, s2_origin_reg)
  directBackingWen := replWen && useRRIPSram.B && !s3_replCAMUse
  if (useRRIPSram) {
    val directWriteMask = Mux(s3_isRepl, Fill(ways, true.B), rrip_way_oh)
    when (replWen) {
      when (s3_replCAMUse) {
        replCAM.get(s3_replCAMIndex).repl := replNextState
        if (useOriginSram) {
          replCAM.get(s3_replCAMIndex).origin.get := nextOrigin
        }
      }.otherwise {
        writeReplBacking(true.B, replNextState, s3_set, directWriteMask)
      }
      if (useOriginSram) {
        when (!s3_replCAMUse) {
          writeOriginBacking(true.B, nextOrigin, s3_set, directWriteMask)
        }
      } else {
        writeOriginBacking(true.B, nextOrigin, s3_set, directWriteMask)
      }
    }
    when (s3_valid && s3_isRepl && replRetry && s3_replCAMAlloc) {
      replCAM.get(s3_replCAMIndex).valid := false.B
    }
  } else {
    when (replWen) {
      replFF.get(s3_set) := replNextState
      if (useRRIP) {
        originFF.get(s3_set) := nextOrigin
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
  val refillLockAcquire = s3_valid && s3_isRepl && !replRetry
  val dirWbUnlock = grantDirWb && dirWbUnlockVec(winDirWb)
  val dirWbOwnerRetire = dirWbUnlock &&
    blockRefillFaultMode =/= L2DirectoryBlockRefillFaultMode.NeverUnlock.U
  val dirWbFinal = dirWbOwnerRetire && dirWbFinalVec(winDirWb)
  val dirWbReplCAMMatchNow = replCAMMatch(reqSet)
  val dirWbReplCAMIndexNow = dirWbReplCAMIndexVec(winDirWb)
  pendClearApply := pendClearValid &&
    blockRefillFaultMode =/= L2DirectoryBlockRefillFaultMode.NeverUnlock.U
  if (useRRIPSram) {
    for (i <- 0 until nMSHR) {
      val increment = refillLockAcquire && s3_replCAMUse && s3_replCAMIndex === i.U
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
      (s2_valid && s2_replCAMAlloc && s2_replCAMIndex === index) ||
      (s3_valid && s3_replCAMAlloc && s3_replCAMIndex === index)
    def commitSafe(index: UInt, set: UInt): Bool =
      !directBackingWen && !allocInFlight(index) &&
        !(s2_valid && s2_set === set) && !(s3_valid && s3_set === set) && !relActive

    val finalCandidateIndex = dirWbReplCAMIndexVec(winDirWb)
    val finalCandidateSet = replCAM.get(finalCandidateIndex).set
    dirWbCommitBlocked := dirWbFinalVec(winDirWb) &&
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
    when(blockRefillFaultMode =/= L2DirectoryBlockRefillFaultMode.NeverUnlock.U) {
      lockTable(pendClearIdx).valid := false.B
    }
    pendClearValid := false.B
  }

  when(refillLockAcquire) {
    // Keep this acquisition identical to the CAM ownerCount update: every
    // allocating replacement reserves its way before the fill commits, even
    // when the request hits an older valid entry. A TSHR may only (re)take its
    // slot once the previous lock is gone; the
    // slot being vacated at this same edge counts as available
    val selfClearing = pendClearApply && pendClearIdx === s3_tshr(idxW - 1, 0)
    assert(!lockTable(s3_tshr).valid || selfClearing,
      "Directory: TSHR took a second refill lock while one is still held")
    assert(!VecInit(lockTable.map(e => e.valid && e.set === s3_set && e.way === way_s3)).asUInt.orR,
      "Directory: double refill lock on (set,way)")
    lockTable(s3_tshr).valid := true.B
    lockTable(s3_tshr).set := s3_set
    lockTable(s3_tshr).way := way_s3
    lockTable(s3_tshr).hitHold := false.B
  }

  // Release at the owner's fill-commit DirWb grant: ownership is structural
  // per TSHR and exact set/way. The tag/meta commit is visible from the next
  // cycle, so the way can be released at this same edge without a clear/idle
  // race. A non-owner or meta-only write never releases. An older refill may
  // still retire replCAM ownership after a same-slot hit-hold handoff, but
  // must not clear the new hold.
  when(dirWbUnlock && !lockTable(winDirWb).hitHold) {
    lockTable(winDirWb).valid := false.B
  }

  val dirWbHitLockRelease = grantDirWb && winReq.REPL_LOCK_RELEASE &&
                            lockTable(winDirWb).valid && lockTable(winDirWb).hitHold &&
                            lockTable(winDirWb).set === reqSet &&
                            lockTable(winDirWb).way === winReq.REPL_LOCK_WAY

  when (grantDirWb && winReq.REPL_LOCK_RELEASE) {
    assert(dirWbHitLockRelease, "Directory: ReplLock release without matching hit-hold ownership")
    assert(winReq.WAY === winReq.REPL_LOCK_WAY, "Directory: ReplLock release way mismatch")
    when (winReq.TAG_WEN) {
      assert(dirWbUnlock, "Directory: ReplLock release without final tag commit")
    }
  }

  when (dirWbHitLockRelease) {
    // The final tag/meta write lands at this edge, so a later request can observe
    // the released way immediately; no delayed clear is needed for hit holds.
    lockTable(winDirWb).valid := false.B
  }

  io.toDir.zipWithIndex.foreach { case (req, i) =>
    val lockSet = requestSet(req)
    val lockWay = req.REPL_LOCK_WAY
    val selfClearing = pendClearApply && pendClearIdx === i.U
    val selfCompatible = !lockTable(i).valid || selfClearing ||
                         !lockTable(i).hitHold &&
                         lockTable(i).set === lockSet && lockTable(i).way === lockWay

    when (req.ReplLock) {
      assert(!replLockConflictVec(i), "Directory: ReplLock conflict was not diverted to prefill")
      assert(selfCompatible, "Directory: ReplLock overwrote incompatible TSHR ownership")
      lockTable(i).valid := true.B
      lockTable(i).set := lockSet
      lockTable(i).way := lockWay
      lockTable(i).hitHold := true.B
      // Cancel both an older pending clear and a clear scheduled by this
      // cycle's old refill release, so handoff has no unlocked interval.
      when ((pendClearValid && pendClearIdx === i.U) ||
            ((dirWbUnlock || dirWbHitLockRelease) && winDirWb === i.U)) {
        pendClearValid := false.B
      }
    }
  }

  lockTable.zipWithIndex.foreach { case (e, i) =>
    assert(!(e.valid && io.tshrIdle(i)),
      s"Directory: blockRefill leaked after owner MSHR retired at TSHR $i")
  }

  if (enableDirPlruProbe) {
    io.debugRrip.get.valid := replWen
    io.debugRrip.get.set := s3_set
    io.debugRrip.get.state := Mux(replWen, replNextState, s3_repl)
    io.debugRrip.get.s3Valid := s3_valid
    io.debugRrip.get.s3Tshr := s3_tshr
    io.debugRrip.get.s3IsDirRd := s3_isDirRd
    io.debugRrip.get.s3IsRepl := s3_isRepl
    io.debugRrip.get.wayS3 := way_s3
    io.debugRrip.get.s3Rrip := s3_repl
    io.debugRrip.get.currentSetRrip := s3_repl
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
    io.debugRetry.get.s3Valid := s3_valid
    io.debugRetry.get.s3Tshr := s3_tshr
    io.debugRetry.get.s3IsDirRd := s3_isDirRd
    io.debugRetry.get.s3IsRepl := s3_isRepl
    io.debugRetry.get.s3Set := s3_set
    io.debugRetry.get.hasInvalid := hasInvalid
    io.debugRetry.get.invalidWay := invalidWay
    io.debugRetry.get.hitWay := hitWay
    io.debugRetry.get.replWay := replWay
    io.debugRetry.get.chosenWay := chosenWay
    io.debugRetry.get.victimWay := victimWay
    io.debugRetry.get.blockRefillMaskS2 := blockRefillMask_s2
    io.debugRetry.get.freeWayMaskS3 := freeWayMask_s3
    io.debugRetry.get.dirWbDone := grantDirWb
    io.debugRetry.get.dirWbDoneSet := reqSet
    io.debugRetry.get.dirWbDoneWay := winReq.WAY(wayBits - 1, 0)
    // re-synthesized per-(set,way) view of the CAM lock table (probe-only, default off)
    io.debugRetry.get.blockRefillLocked := VecInit((0 until sets).map { s =>
      VecInit((0 until ways).map { w =>
        VecInit(lockTable.map(e => e.valid && e.set === s.U && e.way === w.U)).asUInt.orR
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

  // Resp at S3 — routed one-to-one to the originating TSHR
  when(s3_valid) {
    val o = io.fromDir(s3_tshr)
    val dirRdEntry_s3 = dirData_s3(Mux(hit_s3, hitWay, 0.U))
    when (s3_isDirRd && hit_s3) {
      o.META.state := dirRdEntry_s3.meta.state
      o.META.dirty := dirRdEntry_s3.meta.dirty
      o.META.clients := dirRdEntry_s3.meta.clients
      o.META.alias := dirRdEntry_s3.meta.alias
      o.META.prefetch := dirRdEntry_s3.meta.prefetch
      o.META.way := hitWay
      o.META.hit := true.B
    }
    when(s3_isRepl) {
      o.ReplRdResp := !replRetry
      o.ReplRdRetryAck := replRetry
      when(!replRetry) {
        o.REPL.paddr := Cat(entry_s3.tag, s3_set, 0.U(offsetBits.W))
        o.REPL.way := way_s3
        o.REPL.meta := entry_s3.meta
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
  when(s3_valid && s3_isDirRd) {
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
  XSPerfAccumulate("blockRefill_locked", PopCount(lockTable.map(_.valid)))
  XSPerfAccumulate("dirRd_hit",     s3_valid && s3_isDirRd && hit_s3)
  XSPerfAccumulate("dirRd_miss",    s3_valid && s3_isDirRd && !hit_s3)
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
  // R/W wait cycles with older normal S2/S3 occupancy; an idle normal pipeline never increments this counter.
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
