package xscache.openLLC

import chisel3._
import chisel3.stage.ChiselGeneratorAnnotation
import circt.stage.{ChiselStage, FirtoolOption}
import chisel3.util._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tile.MaxHartIdBits
import freechips.rocketchip.tilelink._
import org.chipsalliance.cde.config._
import xscache.coupledL2._
import xscache.coupledL2.prefetch.{MatrixPrefetchControl, MatrixPrefetchParameters, MatrixPrefetchTagField}
import utility._
import utility.chiron._
import xscache.chi._
import xscache.common.{AliasField, BankBitsKey}

import scala.collection.mutable.ArrayBuffer

// Local test-only variant of TestTopMatrix for tl-test replay against a
// testbench CHI subordinate-node model (ChiSnCore): the openLLC L3, the
// OpenNCB bridge and the AXI memory port are removed, and each L2's
// downstream CHI link is exposed at the top level as a DecoupledPortIO.
// Upstream-facing ports (l1d TileLink-C, matrix, mmio), the L2 geometry
// (512 sets x 8 ways) and all other parameters stay identical to
// TestTopMatrix so coverage numbers remain comparable.
// The CHI flits are not observable in this form, so the CLog.B RN/SN
// logging of TestTopMatrix is dropped (TLLog is unaffected).
object TestTopMatrixL2SNParams {
  val l2Sets: Int = 512
  val l2Ways: Int = 8
}

class TestTopMatrixL2SN(
  numCores: Int = 1,
  numULAgents: Int = 0,
  numMAgents: Int = 1,
  banks: Int = 1,
  matrixPrefetchEnable: Boolean = false,
  issue: String = Issue.Eb,
  extTime: Boolean = true
)(implicit p: Parameters) extends LazyModule with HasCHIMsgParameters {

  /*   L1D(L1I)* Matrix*  L1D(L1I)* Matrix*  ...
   *       \     /              \     /
   *        L2                  L2     ...
   *         |                  |
   *      CHI DecoupledPortIO (to the testbench SN model)
   */
  override lazy val desiredName: String = "TestTop"
  val l2Params = p(L2ParamKey)

  def createClientNode(name: String, sources: Int) = {
    implicit val valName: ValName = ValName(name)
    TLClientNode(Seq(
      TLMasterPortParameters.v2(
        masters = Seq(
          TLMasterParameters.v1(
            name = name,
            sourceId = IdRange(0, sources),
            supportsProbe = TransferSizes(l2Params.blockBytes)
          )
        ),
        channelBytes = TLChannelBeatBytes(l2Params.blockBytes),
        minLatency = 1,
        echoFields = Nil,
        requestFields = Seq(AliasField(2)),
        responseKeys = l2Params.respKey
      )
    ))
  }

  def createMatrixNode(name: String, sources: Int) = {
    implicit val valName: ValName = ValName(name)
    TLClientNode(Seq(
      TLMasterPortParameters.v2(
        masters = Seq(
          TLMasterParameters.v1(
            name = name,
            sourceId = IdRange(0, sources)
          )
        ),
        channelBytes = TLChannelBeatBytes(l2Params.blockBytes),
        minLatency = 1,
        echoFields = Nil,
        requestFields = Seq(MatrixField(2), AmeIndexField(), MatrixPrefetchTagField()),
        responseKeys = l2Params.respKey
      )
    ))
  }

  val l1d_nodes = (0 until numCores).map(i => createClientNode(s"l1d$i", 32))
  val l1i_nodes = (0 until numCores).map { i =>
    (0 until numULAgents).map { j =>
      TLClientNode(Seq(
        TLMasterPortParameters.v1(
          clients = Seq(TLMasterParameters.v1(
            name = s"l1i${i}_${j}",
            sourceId = IdRange(0, 32)
          ))
        )
      ))
    }
  }

  val matrix_nodes = (0 until numCores).map { i =>
    (0 until numMAgents).map { j =>
      createMatrixNode(s"matrix${i}_$j", 32)
    }
  }

  val l2_nodes = (0 until numCores).map(i => LazyModule(new CoupledL2()(new Config((site, here, up) => {
    case L2ParamKey => l2Params.copy(
      name = s"L2_$i",
      hartId = i
    )
    case EnableMatrix => true
    case EnableL2DecoupledDownstreamCHI => true
    case CHIIssue => issue
    case BankBitsKey => log2Ceil(banks)
    case MaxHartIdBits => log2Up(numCores)
    case LogUtilsOptionsKey => LogUtilsOptions(
      false,
      here(L2ParamKey).enablePerf,
      here(L2ParamKey).FPGAPlatform
    )
    case PerfCounterOptionsKey => PerfCounterOptions(
      here(L2ParamKey).enablePerf && !here(L2ParamKey).FPGAPlatform,
      here(L2ParamKey).enableRollingDB && !here(L2ParamKey).FPGAPlatform,
      XSPerfLevel.withName("VERBOSE"),
      i
    )
  }))))

  val bankBinders = (0 until numCores).map(_ => BankBinder(banks, 64))

  val mmio_nodes = l1d_nodes.zip(l2_nodes).zipWithIndex.map { case ((l1d, l2), i) =>
    val l1xbar = TLXbar()
    l1xbar :=
      TLBuffer() :=
      TLLogger(s"L2_L1[${i}].C[0]", !l2Params.FPGAPlatform && l2Params.enableTLLog) :=
      l1d

    l1i_nodes(i).zipWithIndex.foreach { case (l1i, j) =>
      l1xbar :=
        TLBuffer() :=
        TLLogger(s"L2_L1[${i}].UL[${j}]", !l2Params.FPGAPlatform && l2Params.enableTLLog) :=
        l1i
    }

    matrix_nodes(i).zipWithIndex.foreach { case (node, j) =>
      l1xbar :=
        TLBuffer() :=
        TLLogger(s"L2_Matrix[${i}][${j}]", !l2Params.FPGAPlatform && l2Params.enableTLLog) :=
        node
    }

    l2.managerNode :=
      TLXbar() :=*
      bankBinders(i) :*=
      l2.node :*=
      l1xbar

    val mmioClientNode = TLClientNode(Seq(
      TLMasterPortParameters.v1(
        clients = Seq(TLMasterParameters.v1(
          name = "uncache",
          sourceId = IdRange(0, 16)
        ))
      )
    ))
    l2.mmioBridge.mmioNode := mmioClientNode
    mmioClientNode
  }

  lazy val module = new LazyModuleImp(this) {
    val time_sim = if (extTime) {
      IO(Input(UInt(64.W)))
    } else {
      WireDefault(0.U(64.W))
    }

    val log = IO(new Bundle {
      val dump = Input(Bool())
      val clean = Input(Bool())
    })

    val matrixDataOut = IO(Vec(numCores, Vec(banks, DecoupledIO(new MatrixDataBundle()))))
    // tl-test-new replays the CUTE task-control sideband independently of
    // the matrix TileLink ports.  The current matrix test target is
    // single-core; broadcast the same control bundle if this top is
    // elaborated with more.
    val matrixPrefetch = IO(Input(new MatrixPrefetchControl))

    // Expose the otherwise-dangling MMIO client port so the VCS TB can issue
    // uncached Get/Put transactions into l2.mmioBridge.
    mmio_nodes.zipWithIndex.foreach { case (node, i) =>
      node.makeIOs()(ValName(s"mmio_port_$i"))
    }

    val io_l1 = l2_nodes.map { l2_node =>
      IO(new Bundle {
        val l2Hint = Valid(new L2ToL1Hint()(p.alterPartial { case EdgeInKey => l2_node.node.in.head._2 }))
      })
    }

    // Downstream CHI links straight to the testbench SN model. The bundle
    // type is taken from the L2's own port: the inner CHI parameter scope
    // (e.g. the mpam field) differs from the outer one, and a plain
    // `new DecoupledPortIO` here would not monotonically connect.
    val io_chi = IO(Vec(numCores, chiselTypeOf(l2_nodes.head.module.io.decoupledCHI.get)))

    val cycle = RegInit(0.U(64.W))
    cycle := cycle + 1.U

    val timer = WireDefault(0.U(64.W))
    val logEnable = WireDefault(false.B)
    val clean = WireDefault(false.B)
    val dump = WireDefault(false.B)

    timer := { if (extTime) time_sim else cycle }
    logEnable := true.B
    clean := log.clean
    dump := log.dump

    dontTouch(timer)
    dontTouch(logEnable)
    dontTouch(clean)
    dontTouch(dump)

    l1d_nodes.zipWithIndex.foreach {
      case (node, i) =>
        node.makeIOs()(ValName(s"master_port_$i"))
    }
    if (numULAgents != 0) {
      l1i_nodes.zipWithIndex.foreach { case (core, i) =>
        core.zipWithIndex.foreach { case (node, j) =>
          node.makeIOs()(ValName(s"master_ul_port_${i}_${j}"))
        }
      }
    }
    matrix_nodes.zipWithIndex.foreach { case (nodes, i) =>
      nodes.zipWithIndex.foreach { case (node, j) =>
        node.makeIOs()(ValName(s"master_m_port_${i}_${j}"))
      }
    }

    l2_nodes.zipWithIndex.foreach { case (l2, i) =>
      l2.module.io.decoupledCHI.get <> io_chi(i)
      dontTouch(l2.module.io)

      l2.module.io.l2_hint <> io_l1(i).l2Hint

      l2.module.io.hartId := i.U
      // No legacy prefetch engine is selected.  This control is still tied
      // low so that adding another engine to the test configuration cannot
      // silently change the matrix-prefetch A/B comparison.
      l2.module.io.pfCtrlFromCore := 0.U.asTypeOf(l2.module.io.pfCtrlFromCore)
      l2.module.io.matrixPrefetch.foreach(_ := matrixPrefetch)
      l2.module.io.nodeID := i.U(NODEID_WIDTH.W)
      l2.module.io.debugTopDown := DontCare
      matrixDataOut(i) <> l2.module.io.matrixDataOut.get
      l2.module.io.l2_tlb_req <> DontCare
    }

    if (!l2Params.FPGAPlatform && l2Params.enablePerf) {
      XSLog.collect(timer, logEnable, clean, dump)
    }
  }
}

object TestTopMatrixL2SN extends App {
  val usage = """
Usage: TestTopMatrixL2SN [<--option> <values>]

      --core <core_count>       specify core count, 1 by default
      --tl-ul <tl_ul_count>     specify TileLink-UL agent count per core, 0 by default
      --m-agent <m_count>       specify matrix agent count per core, 1 by default
      --bank <bank_count>       specify bank (slice) count, 1 by default
      --fpga <1>                generate for FPGA platform
      --chiseldb <1>            enable ChiselDB
      --tllog <1>               enable TLLogger under ChiselDB
      --matrix-prefetch <1>     statically enable the matrix-guided L2 prefetcher
  """

  if (args.contains("--help"))
  {
    println(usage)
    System.exit(-1)
  }

  var varArgs = ArrayBuffer(args.toIndexedSeq: _*)
  var varArgsDropped = 0

  var numCores: Int = 1
  var numULAgents: Int = 0
  var numMAgents: Int = 1
  var numBanks: Int = 1
  var onFPGAPlatform: Boolean = false
  var enableChiselDB: Boolean = false
  var enableTLLog: Boolean = false
  var matrixPrefetchEnable: Boolean = false

  val varArgsToDrop = args.sliding(2, 1).zipWithIndex.collect {
    case (Array("--core", value), i) => (numCores = value.toInt, i)
    case (Array("--tl-ul", value), i) => (numULAgents = value.toInt, i)
    case (Array("--m-agent", value), i) => (numMAgents = value.toInt, i)
    case (Array("--bank", value), i) => (numBanks = value.toInt, i)
    case (Array("--fpga", value), i) => (onFPGAPlatform = value.toInt != 0, i)
    case (Array("--chiseldb", value), i) => (enableChiselDB = value.toInt != 0, i)
    case (Array("--tllog", value), i) => (enableTLLog = value.toInt != 0, i)
    case (Array("--matrix-prefetch", value), i) => (matrixPrefetchEnable = value.toInt != 0, i)
  }

  varArgsToDrop.map(_._2).foreach { i =>
    varArgs.remove(i - varArgsDropped, 2)
    varArgsDropped = varArgsDropped + 2
  }
  varArgs.trimToSize()

  val FPGAPlatform = onFPGAPlatform
  val enableChiselDB_ = !FPGAPlatform && enableChiselDB
  val enablePerf = !FPGAPlatform

  val config = new Config((_, _, _) => {
    case L2ParamKey => L2Param(
      ways = TestTopMatrixL2SNParams.l2Ways,
      sets = TestTopMatrixL2SNParams.l2Sets,
      clientCaches = Seq(L1Param(aliasBitsOpt = Some(2))),
      prefetch = if (matrixPrefetchEnable) Seq(MatrixPrefetchParameters()) else Nil,
      enablePerf = enablePerf,
      enableRollingDB = false,
      enableMonitor = false,
      enableTLLog = enableTLLog,
      elaboratedTopDown = enablePerf,
      enableMCP2 = false,
      FPGAPlatform = FPGAPlatform,
      dataCheck = Some("oddparity"),
      sam = Seq(AddressSet.everything -> 33)
    )
  })

  CLogB.init(false)
  ChiselDB.init(enableChiselDB_)
  // Standalone tl-test does not link XiangShan's Constantin runtime.  Fold all
  // records to their elaboration defaults; --matrix-prefetch selects 0 or 1.
  Constantin.init(false)

  val top = DisableMonitors(p => LazyModule(new TestTopMatrixL2SN(
    numCores, numULAgents, numMAgents, numBanks, matrixPrefetchEnable
  )(p)))(config)

  (new ChiselStage).execute(varArgs.toArray, Seq(
    ChiselGeneratorAnnotation(() => top.module),
    FirtoolOption("--disable-annotation-unknown"),
    FirtoolOption("--default-layer-specialization=enable")
  ))

  ChiselDB.addToFileRegisters
  FileRegisters.write("./build")
}
