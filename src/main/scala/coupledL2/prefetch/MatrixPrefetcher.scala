/**
  * Matrix-guided L2 prefetch support.
  *
  * The matrix address generator and guided prefetcher live in this file so the
  * generic prefetch datapath remains independent of the Matrix extension.
  */

package xscache.coupledL2.prefetch

import chisel3._
import chisel3.util._
import freechips.rocketchip.util.{BundleField, ControlKey}
import org.chipsalliance.cde.config.Parameters
import utility._
import xscache.coupledL2._
import xscache.coupledL2.utils._

case class MatrixPrefetchParameters(inflightEntries: Int = 32) extends PrefetchParameters {
  override val hasPrefetchBit: Boolean = true
  override val hasPrefetchSrc: Boolean = true
}

object MatrixPrefetchStream {
  val width = 3
  val none = 0.U(width.W)
  val a = 1.U(width.W)
  val b = 2.U(width.W)
  val cLoad = 3.U(width.W)
  val cStore = 4.U(width.W)
  val aScale = 5.U(width.W)
  val bScale = 6.U(width.W)
}

object MatrixPrefetchTagCodec {
  val taskIdWidth = 16
  val width = 1 + MatrixPrefetchStream.width + taskIdWidth

  def encode(valid: Bool, stream: UInt, taskId: UInt): UInt =
    Cat(valid, stream(MatrixPrefetchStream.width - 1, 0), taskId(taskIdWidth - 1, 0))

  def valid(tag: UInt): Bool = tag(width - 1)
  def stream(tag: UInt): UInt = tag(taskIdWidth + MatrixPrefetchStream.width - 1, taskIdWidth)
  def taskId(tag: UInt): UInt = tag(taskIdWidth - 1, 0)
}

case object MatrixPrefetchTagKey extends ControlKey[UInt](name = "MatrixPrefetchTag")
case class MatrixPrefetchTagField()
  extends BundleField[UInt](MatrixPrefetchTagKey, Output(UInt(MatrixPrefetchTagCodec.width.W)), _ := 0.U)

class MatrixPrefetchDesc extends Bundle {
  val taskId = UInt(MatrixPrefetchTagCodec.taskIdWidth.W)
  val stream = UInt(MatrixPrefetchStream.width.W)
  val baseAddr = UInt(64.W)
  val outerStride = UInt(64.W)
  val outerCount = UInt(16.W)
  val innerCount = UInt(16.W)
  val rowBytes = UInt(32.W)
  val groupWidth = UInt(16.W)
  val transpose = Bool()
  val pc = UInt(64.W)
}

class MatrixPrefetchControl extends Bundle {
  val allocate = Valid(new MatrixPrefetchDesc)
  val activate = Valid(UInt(MatrixPrefetchTagCodec.width.W))
  val retire = Valid(UInt(MatrixPrefetchTagCodec.width.W))
  val cStoreStart = Valid(UInt(MatrixPrefetchTagCodec.width.W))
  val cStoreEnd = Valid(UInt(MatrixPrefetchTagCodec.width.W))
}

private[prefetch] object FullPriorityOneHot {
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

/** Walks the cache lines touched by a CUTE strided matrix load. */
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
  val bOnASelectedAddress = if (bOnAWalkers == 1) bOnALookaheadGenerators.head.io.address
    else VecInit(bOnALookaheadGenerators.map(_.io.address))(bOnASelectedWalker)
  val bOnASelectedTaskId = if (bOnAWalkers == 1) bOnACurrentDesc.head.taskId
    else VecInit(bOnACurrentDesc.map(_.taskId))(bOnASelectedWalker)
  val bOnASelectedIssued = if (bOnAWalkers == 1) bOnALookaheadIssued.head
    else bOnALookaheadIssued(bOnASelectedWalker)
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
