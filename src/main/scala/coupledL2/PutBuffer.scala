package xscache.coupledL2

import chisel3._
import chisel3.util._
import chisel3.experimental.BundleLiterals._
import org.chipsalliance.cde.config.Parameters

class PutBufState(implicit p: Parameters) extends L2Bundle {
  val entryIdx = UInt(bufIdxBits.W)
}

class PutBufWrite(implicit p: Parameters) extends L2Bundle {
  val data  = UInt((beatBytes * 8).W)
  val beat  = UInt(beatBits.W)
  val first = Bool()
  val last  = Bool()
}

class PutBufRead(implicit p: Parameters) extends L2Bundle {
  val id = UInt(bufIdxBits.W)
}

class PutBufResp(implicit p: Parameters) extends L2Bundle {
  val data = new DSBlock
}

class PutBuffer(implicit p: Parameters) extends L2Module {
  val io = IO(new Bundle {
    val w = Flipped(DecoupledIO(new PutBufWrite))
    val r = Flipped(ValidIO(new PutBufRead))

    val state = Output(new PutBufState)

    val resp = Output(new PutBufResp)
  })

  private val buffer = RegInit(
    VecInit(
      Seq.fill(bufBlocks)(
        Valid(Vec(beatSize, UInt((beatBytes * 8).W))).Lit(_.valid -> false.B)
      )
    )
  )

  private val valids = VecInit(buffer.map(_.valid))
  private val full = valids.asUInt.andR
  private val freeMask = ~valids.asUInt

  // Hold the selected entry across beats so one multibeat Put cannot drift
  // into a newly-freed, higher-priority entry.
  private val sel_r = RegInit(0.U(bufIdxBits.W))
  private val sel_nxt = PriorityEncoder(freeMask)

  private val sel = Mux(io.w.bits.first, sel_nxt, sel_r)

  buffer.zipWithIndex.foreach { case (entry, i) =>
    val wen = io.w.fire && sel === i.U

    when (wen) {
      entry.bits(io.w.bits.beat) := io.w.bits.data

      when (io.w.bits.last) {
        entry.valid := true.B
      }
    }
  }

  when (io.w.fire && io.w.bits.first) {
    sel_r := sel_nxt
  }

  when (io.r.valid) {
    buffer(io.r.bits.id).valid := false.B
  }

  private val rdata = RegEnable(buffer(io.r.bits.id).bits.asUInt, io.r.valid)

  io.state.entryIdx := sel_r
  io.w.ready := !full
  io.resp.data.data := rdata

  dontTouch(freeMask)
  dontTouch(full)
  dontTouch(sel_r)
  dontTouch(sel_nxt)
  dontTouch(sel)
}
