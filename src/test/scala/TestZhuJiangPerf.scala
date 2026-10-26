package zhujiang

import chisel3._
import chisel3.experimental.hierarchy.{Definition, Instance, instantiable, public}
import org.chipsalliance.cde.config.Parameters
import utility.{LogUtilsOptions, LogUtilsOptionsKey, PerfCounterOptions, PerfCounterOptionsKey, XSLog, XSPerfLevel}
import zhujiang.perf._

private class PerfEventSource(implicit p: Parameters) extends Module {
  val value = IO(Input(UInt(8.W)))
  ZJPerf.accumulate("zj_test_accumulate", value)
  ZJPerf.distribution("zj_test_distribution", value, true.B)
}

@instantiable
private class PerfDefinition(implicit p: Parameters) extends Module {
  ZJPerf.beginDefinition()
  @public val value = IO(Input(UInt(8.W)))
  private val source = Module(new PerfEventSource)
  private val second = Module(new PerfEventSource)
  source.value := value
  second.value := value + 1.U
  @public val events = ZJPerf.collect()
}

private class PerfExportTop(implicit p: Parameters) extends Module {
  val values = IO(Input(Vec(2, UInt(8.W))))
  val clean = IO(Input(Bool()))
  val dump = IO(Input(Bool()))
  private val gate = Module(new zhujiang.utils.DoubleCounterClockGate)
  gate.io.te := false.B
  gate.io.inbound := false.B
  gate.io.working := false.B
  dontTouch(gate.io.ock)
  private val definition = Definition(new PerfDefinition)
  private val owners = Seq.fill(2)(Instance(definition))
  owners.zip(values).foreach { case (owner, value) => owner.value := value }
  ZJPerf.consume(owners.zipWithIndex.map { case (owner, i) => s"hnf_$i" -> owner.events })
  XSLog.collect(0.U(64.W), true.B, clean, dump)
}

// Run enabled and disabled modes in separate JVMs: XSLog owns elaboration state.
object TestZhuJiangPerf extends App {
  private val enabled = !args.contains("--disable-perf")
  implicit val p: Parameters = Parameters.empty.alterPartial {
    case PerfCounterOptionsKey => PerfCounterOptions(enabled, false, XSPerfLevel.VERBOSE, 0)
    case LogUtilsOptionsKey => LogUtilsOptions(false, enabled, false)
    case ZJPerfBackendKey => XiangShanUtilityPerfBackend
  }
  private val chirrtl = circt.stage.ChiselStage.emitCHIRRTL(new PerfExportTop)
  require(!chirrtl.contains("extmodule LogPerfHelper"), "XSLog backend still instantiates a local control helper")
  if (enabled) {
    require(chirrtl.contains("hnf_0__source"), "missing first HNF collector hierarchy")
    require(chirrtl.contains("hnf_1__source"), "missing second HNF collector hierarchy")
    require(chirrtl.contains("hnf_0__second"), "missing second owner hierarchy")
    require(chirrtl.contains("hnf_1__second"), "missing second instance owner hierarchy")
    require(chirrtl.contains("zj_test_accumulate"), "missing exported accumulation counter")
    require(chirrtl.contains("zj_test_distribution_min"), "missing exported minimum counter")
    require(chirrtl.contains("zj_test_distribution_max"), "missing exported maximum counter")
  } else {
    require(!chirrtl.contains("XiangShanPerfCollector"), "disabled performance still instantiates collectors")
    require(!chirrtl.contains("zj_test_accumulate"), "disabled performance still registers counters")
  }
  println(s"ZhuJiang performance export regression passed (enabled=$enabled)")
}

object TestZhuJiangPerfCounters extends App with chisel3.simulator.ChiselSim {
  if (args.contains("--simulate")) {
    implicit val p: Parameters = Parameters.empty.alterPartial {
      case PerfCounterOptionsKey => PerfCounterOptions(true, false, XSPerfLevel.VERBOSE, 0)
      case LogUtilsOptionsKey => LogUtilsOptions(false, true, false)
      case ZJPerfBackendKey => XiangShanUtilityPerfBackend
    }
    simulate(new PerfExportTop) { dut =>
      dut.values(0).poke(2.U)
      dut.values(1).poke(5.U)
      dut.dump.poke(false.B)
      dut.clean.poke(true.B)
      dut.clock.step()
      dut.clean.poke(false.B)
      dut.clock.step(3)
      dut.dump.poke(true.B)
      dut.clock.step()
      dut.dump.poke(false.B)
      dut.clean.poke(true.B)
      dut.clock.step()
      dut.clean.poke(false.B)
      dut.dump.poke(true.B)
      dut.clock.step()
    }
  } else {
    // The simulator inherits the process error stream; capture it in a child JVM.
    val javaBin = java.nio.file.Paths.get(System.getProperty("java.home"), "bin", "java").toString
    val process = new ProcessBuilder(javaBin, "-Xmx2g", "-cp", System.getProperty("java.class.path"),
      "zhujiang.TestZhuJiangPerfCounters", "--simulate").redirectErrorStream(true).start()
    val log = new String(process.getInputStream.readAllBytes(), java.nio.charset.StandardCharsets.UTF_8)
    require(process.waitFor() == 0, s"performance simulation failed: $log")
    val counts = "zj_test_accumulate, +([0-9]+)".r.findAllMatchIn(log).map(_.group(1).toInt).toSeq
    require(counts == Seq(8, 12, 20, 24, 2, 3, 5, 6), s"dump/clean counter mismatch: $counts")
    for (suffix <- Seq("min", "max")) {
      val samples = ("zj_test_distribution_" + suffix + ", +([0-9]+)").r
        .findAllMatchIn(log).map(_.group(1).toInt).toSeq
      require(samples == Seq(2, 3, 5, 6, 2, 3, 5, 6), s"$suffix counter mismatch: $samples")
    }
    println("ZhuJiang performance dump/clean simulation passed")
  }
}
