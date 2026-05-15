package cache

import coupledL2.utils._
import chisel3._
import chisel3.util._
import chisel3.experimental._
import chisel3.simulator.ChiselSim
import chisel3.simulator.HasSimulator
import svsim.verilator.Backend.CompilationSettings.TraceStyle
import svsim.verilator.Backend.CompilationSettings.TraceKind

import coupledL2.utils.PseudoLRU
import freechips.rocketchip.util.UIntToAugmentedUInt

class PLRU(way: Int) extends Module {
  val repl = new PseudoLRU(way)
  val io = IO(new Bundle {
    val s = Input(UInt(repl.nBits.W))
  })


  /** @param state state_reg bits for this sub-tree
    * @param tree_nways number of ways in this sub-tree
    */
  def get_replace_way(state: UInt, tree_nways: Int): UInt = {
    require(state.getWidth == (tree_nways-1), s"wrong state bits width ${state.getWidth} for $tree_nways ways")

    // this algorithm recursively descends the binary tree, filling in the way-to-replace encoded value from msb to lsb
    if (tree_nways > 2) {
      // we are at a branching node in the tree, so recurse
      val right_nways: Int = 1 << (log2Ceil(tree_nways) - 1)  // number of ways in the right sub-tree
      val left_nways:  Int = tree_nways - right_nways         // number of ways in the left sub-tree
      val left_subtree_older  = state(tree_nways-2)
      val left_subtree_state  = state.extract(tree_nways-3, right_nways-1)
      val right_subtree_state = state(right_nways-2, 0)

      if (left_nways > 1) {
        // we are at a branching node in the tree with both left and right sub-trees, so recurse both sub-trees
        Cat(left_subtree_older,      // return the top state bit (current tree node) as msb of the way-to-replace encoded value
          Mux(left_subtree_older,  // if left sub-tree is older, recurse left, else recurse right
            get_replace_way(left_subtree_state,  left_nways),    // recurse left
            get_replace_way(right_subtree_state, right_nways)))  // recurse right
      } else {
        // we are at a branching node in the tree with only a right sub-tree, so recurse only right sub-tree
        Cat(left_subtree_older,      // return the top state bit (current tree node) as msb of the way-to-replace encoded value
          Mux(left_subtree_older,  // if left sub-tree is older, return and do not recurse right
            0.U(1.W),
            get_replace_way(right_subtree_state, right_nways)))  // recurse right
      }
    } else if (tree_nways == 2) {
      // we are at a leaf node at the end of the tree, so just return the single state bit as lsb of the way-to-replace encoded value
      state(0)
    } else {  // tree_nways <= 1
      // we are at an empty node in an unbalanced tree for non-power-of-2 ways, so return single zero bit as lsb of the way-to-replace encoded value
      0.U(1.W)
    }
  }

  def get_replace_way(state: UInt): UInt = get_replace_way(state, way)

  assert(OHToUInt(repl.get_replace_OH(io.s)) === get_replace_way(io.s))
}

object TestPLRU extends App with ChiselSim {
  val dumpVcd = false
  implicit val sim: chisel3.simulator.HasSimulator = if (dumpVcd)
    HasSimulator.simulators.verilator(
      verilatorSettings = svsim.verilator.Backend.CompilationSettings(
        traceStyle = Some(TraceStyle(TraceKind.Vcd)),
        outputSplit = Some(30000),
        outputSplitCFuncs = Some(30000),
        disabledWarnings = Seq("STMTDLY", "WIDTH")
      )
    )
  else
    HasSimulator.default

  simulate(
    new PLRU(way = 11))
  { s =>
    for (i <- 0 until math.pow(2, 10).toInt) {
      println(s"Testing $i")
      s.io.s.poke(i.U)
      s.clock.step(1)
    }
  }
}