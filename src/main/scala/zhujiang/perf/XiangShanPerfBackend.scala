package chisel3.internal {
  import chisel3.experimental.BaseModule

  // Expose parent/seed so Definition-relative collector names can be rebuilt
  // before the parent circuit has finished naming.
  object ZJPerfModuleInfo {
    def parent(module: BaseModule): Option[BaseModule] = module._parent
    def seedName(module: BaseModule): Option[String] = module.seedOpt
  }
}

package zhujiang.perf {

import chisel3._
import chisel3.experimental.BaseModule
import chisel3.internal.ZJPerfModuleInfo
import chisel3.reflect.DataMirror.isVisible
import chisel3.util.experimental.BoringUtils.{bore, tapAndRead}
import org.chipsalliance.cde.config.Parameters
import utility.{
  LogUtilsOptionsKey,
  PerfCounterOptionsKey,
  XSPerfAccumulate,
  XSPerfHistogram,
  XSPerfLevel => UtilityPerfLevel,
  XSPerfMax,
  XSPerfMin
}

import scala.collection.mutable.ListBuffer

private class XiangShanPerfCollector(specs: Seq[ZJPerfEventSpec])(implicit p: Parameters) extends Module {
  val events = IO(Input(new ZJPerfExport(specs)))

  private def level(value: ZJPerfLevel.Value): UtilityPerfLevel.XSPerfLevel =
    UtilityPerfLevel.withName(value.toString)

  specs.zip(events.events).foreach {
    case (spec: ZJPerfAccumulateSpec, event) =>
      XSPerfAccumulate(spec.name, event.value, level(spec.level))
    case (spec: ZJPerfMaxSpec, event) =>
      XSPerfMax(spec.name, event.value, event.enable, level(spec.level))
    case (spec: ZJPerfDistributionSpec, event) =>
      ZJPerfDistribution.counters(spec.name, event.value, event.enable, spec.ranges).foreach { case (name, value) =>
        XSPerfAccumulate(name, value, level(spec.level))
      }
      if (spec.includeMinMax) {
        XSPerfMin(spec.name, event.value, event.enable, level(spec.level))
        XSPerfMax(spec.name, event.value, event.enable, level(spec.level))
      }
    case (spec: ZJPerfHistogramSpec, event) =>
      XSPerfHistogram(
        spec.name,
        event.value,
        event.enable,
        spec.start,
        spec.stop,
        spec.step,
        spec.leftStrict,
        spec.rightStrict,
        level(spec.level)
      )
  }
}

// XiangShan backend keeps reusable Definition events as hardware exports and
// instantiates XSPerf collectors at the parent that consumes those exports.
object XiangShanUtilityPerfBackend extends ZJPerfBackend {
  private case class PendingEvent(spec: ZJPerfEventSpec, value: UInt, enable: Bool)

  private def level(value: ZJPerfLevel.Value): UtilityPerfLevel.XSPerfLevel =
    UtilityPerfLevel.withName(value.toString)

  override def enabled(implicit p: Parameters): Boolean =
    p(PerfCounterOptionsKey).enablePerfPrint

  private def levelEnabled(value: ZJPerfLevel.Value)(implicit p: Parameters): Boolean =
    p(PerfCounterOptionsKey).enablePerfPrint && level(value) >= p(PerfCounterOptionsKey).perfLevel

  private def tapOrGet[T <: Data](data: T)(implicit p: Parameters): T = {
    if (isVisible(data)) {
      data
    } else if (p(LogUtilsOptionsKey).enableXMR) {
      tapAndRead(data)
    } else {
      bore(data)
    }
  }

  private def emptyExport: ZJPerfExport = {
    val output = Wire(new ZJPerfExport(Seq.empty))
    output := DontCare
    output
  }

  private def legalizeName(raw: String): String = {
    val cleaned = raw.map {
      case ch if ch.isLetterOrDigit || ch == '_' => ch
      case _                                     => '_'
    }.mkString
    val collapsed = cleaned.replaceAll("_+", "_").stripPrefix("_").stripSuffix("_")
    if (collapsed.isEmpty) "mod"
    else if (collapsed.head.isLetter) collapsed
    else s"mod_$collapsed"
  }

  private def ownerInstanceName(module: BaseModule, ownerId: Int): String = {
    ZJPerfModuleInfo.seedName(module).filter(_.nonEmpty).map(legalizeName).getOrElse {
      legalizeName(s"${module.desiredName}_$ownerId")
    }
  }

  private def ownerRelativePath(module: BaseModule, ownerId: Int, root: Option[BaseModule]): String = {
    val names = ListBuffer.empty[String]
    var current = Option(module)
    var depth = 0
    while (current.exists(mod => !root.exists(_ eq mod)) && depth < 64) {
      val owner = current.get
      names += ownerInstanceName(owner, ownerId)
      current = ZJPerfModuleInfo.parent(owner)
      depth += 1
    }
    val path = names.reverse.mkString("__")
    if (path.nonEmpty) path else s"mod_$ownerId"
  }

  private def withOwnerPath(spec: ZJPerfEventSpec, ownerPath: String): ZJPerfEventSpec = spec match {
    case event: ZJPerfAccumulateSpec   => event.copy(ownerPath = ownerPath)
    case event: ZJPerfMaxSpec          => event.copy(ownerPath = ownerPath)
    case event: ZJPerfDistributionSpec => event.copy(ownerPath = ownerPath)
    case event: ZJPerfHistogramSpec    => event.copy(ownerPath = ownerPath)
  }

  private def collectorName(instanceName: String, spec: ZJPerfEventSpec): String = {
    val relative = if (spec.ownerPath.nonEmpty) spec.ownerPath else s"mod_${spec.ownerId}"
    val prefix = legalizeName(instanceName)
    if (relative.isEmpty) prefix else s"${prefix}__$relative"
  }

  override def newScope(kind: ZJPerfScopeKind.Value): ZJPerfBackendScope = new ZJPerfBackendScope {
    private val pending = ListBuffer.empty[PendingEvent]
    private val owners  = ListBuffer.empty[BaseModule]

    private def ownerId: Int = {
      val owner = chisel3.XSCompatibility.currentModule.getOrElse(
        throw new IllegalArgumentException("ZhuJiang performance events must be registered inside a module")
      )
      val existing = owners.indexWhere(_ eq owner)
      if (existing >= 0) {
        existing
      } else {
        owners += owner
        owners.size - 1
      }
    }

    override def accumulate(name: String, value: UInt, perfLevel: ZJPerfLevel.Value)
                           (implicit p: Parameters): Unit = {
      if (kind == ZJPerfScopeKind.Normal) {
        XSPerfAccumulate(name, value, level(perfLevel))
      } else if (levelEnabled(perfLevel)) {
        pending += PendingEvent(ZJPerfAccumulateSpec(name, perfLevel, ownerId), value, true.B)
      }
    }

    override def max(name: String, value: UInt, enable: Bool, perfLevel: ZJPerfLevel.Value)
                    (implicit p: Parameters): Unit = {
      if (kind == ZJPerfScopeKind.Normal) {
        XSPerfMax(name, value, enable, level(perfLevel))
      } else if (levelEnabled(perfLevel)) {
        pending += PendingEvent(ZJPerfMaxSpec(name, perfLevel, ownerId), value, enable)
      }
    }

    override def distribution(
      name: String,
      value: UInt,
      enable: Bool,
      ranges: Seq[HistogramRange],
      includeMinMax: Boolean,
      perfLevel: ZJPerfLevel.Value
    )(implicit p: Parameters): Unit = {
      if (kind == ZJPerfScopeKind.Normal) {
        ZJPerfDistribution.counters(name, value, enable, ranges).foreach { case (counterName, counterValue) =>
          XSPerfAccumulate(counterName, counterValue, level(perfLevel))
        }
        if (includeMinMax) {
          XSPerfMin(name, value, enable, level(perfLevel))
          XSPerfMax(name, value, enable, level(perfLevel))
        }
      } else if (levelEnabled(perfLevel)) {
        pending += PendingEvent(ZJPerfDistributionSpec(name, ranges, includeMinMax, perfLevel, ownerId), value, enable)
      }
    }

    override def histogram(
      name: String,
      value: UInt,
      enable: Bool,
      start: Int,
      stop: Int,
      step: Int,
      leftStrict: Boolean,
      rightStrict: Boolean,
      perfLevel: ZJPerfLevel.Value
    )(implicit p: Parameters): Unit = {
      if (kind == ZJPerfScopeKind.Normal) {
        XSPerfHistogram(
          name,
          value,
          enable,
          start,
          stop,
          step,
          leftStrict,
          rightStrict,
          level(perfLevel)
        )
      } else if (levelEnabled(perfLevel)) {
        pending += PendingEvent(
          ZJPerfHistogramSpec(name, start, stop, step, leftStrict, rightStrict, perfLevel, ownerId),
          value,
          enable
        )
      }
    }

    override def collect()(implicit p: Parameters): ZJPerfExport = {
      if (kind == ZJPerfScopeKind.Normal || !enabled) {
        emptyExport
      } else {
        val root = chisel3.XSCompatibility.currentModule
        val ownerPaths = owners.zipWithIndex.map { case (owner, ownerId) =>
          ownerId -> ownerRelativePath(owner, ownerId, root)
        }.toMap
        val entries = pending.toSeq.map { entry =>
          entry.copy(spec = withOwnerPath(entry.spec, ownerPaths(entry.spec.ownerId)))
        }
        val output  = IO(Output(new ZJPerfExport(entries.map(_.spec))))
        entries.zip(output.events).foreach { case (entry, event) =>
          event.value  := tapOrGet(entry.value)
          event.enable := tapOrGet(entry.enable)
        }
        pending.clear()
        owners.clear()
        output
      }
    }
  }

  override def consume(exports: Seq[(String, ZJPerfExport)])(implicit p: Parameters): Unit = {
    exports.foreach { case (instanceName, output) =>
      output.specs.zipWithIndex.groupBy(_._1.ownerId).toSeq.sortBy(_._1).foreach { case (_, entries) =>
        val specs     = entries.map(_._1)
        val eventIds  = entries.map(_._2)
        val collector = Module(new XiangShanPerfCollector(specs))
        collector.suggestName(collectorName(instanceName, specs.head))
        collector.events.events.zip(eventIds).foreach { case (sink, eventId) =>
          sink := output.events(eventId)
        }
      }
    }
  }
}
}
