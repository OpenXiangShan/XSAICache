/** *************************************************************************************
 * Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
 * Copyright (c) 2020-2021 Peng Cheng Laboratory
 *
 * XiangShan is licensed under Mulan PSL v2.
 * You can use this software according to the terms and conditions of the Mulan PSL v2.
 * You may obtain a copy of Mulan PSL v2 at:
 * http://license.coscl.org.cn/MulanPSL2
 *
 * THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
 * EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
 * MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
 *
 * See the Mulan PSL v2 for more details.
 * *************************************************************************************
 */

package coupledL2

import chisel3._
import chisel3.util._
import utility._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink.TLMessages._
import coupledL2.utils._

class HintQueueEntry(implicit p: Parameters) extends L2Bundle {
  val source = UInt(sourceIdBits.W)
  val isGrantData = Bool()
  val isKeyword = Bool()
  val hasData = Bool()
}

class CustomL1HintIOBundle(implicit p: Parameters) extends L2Bundle {
  // input information
  val mshrHintQInfo = Flipped(ValidIO(new TaskBundle()))
  val sinkCHintQInfo = Flipped(ValidIO(new TaskBundle()))
  val retry_s2 = Input(Bool())

  val s3 = new L2Bundle {
      val task      = Flipped(ValidIO(new TaskBundle()))
      val need_mshr = Input(Bool())
  }

  // output hint
  val l1Hint = DecoupledIO(new L2ToL1HintInsideL2())
}

// grantData hint interface
// use this interface to give a hint to l1 before actually sending a GrantData
class CustomL1Hint(implicit p: Parameters) extends L2Module {
  val io = IO(new CustomL1HintIOBundle)

  val mshr_s1 = io.mshrHintQInfo.bits
  val mshrMerge_s1 = mshr_s1.aMergeTask
  val sinkC_s1 = io.sinkCHintQInfo.bits
  val task_s3 = io.s3.task
  val mshrReq_s3 = task_s3.bits.mshrTask
  val need_mshr_s3 = io.s3.need_mshr

  // ==================== Hint Generation ====================
  // Hint for "MSHRTask and ReleaseAck" will fire@s1
  def isGrantData(t: TaskBundle):  Bool = t.fromA && t.opcode === GrantData
  def isGrant(t: TaskBundle):      Bool = t.fromA && t.opcode === Grant
  def isMergeGrantData(t: TaskBundle): Bool = t.fromA && t.mergeA && t.aMergeTask.opcode === GrantData
  def isMergeGrant(t: TaskBundle):     Bool = t.fromA && t.mergeA && t.aMergeTask.opcode === Grant
  def isAccessAckData(t: TaskBundle):  Bool = t.fromA && t.opcode === AccessAckData
  def isCBOAck(t: TaskBundle):         Bool = t.fromA && t.opcode === CBOAck

  val mshr_GrantData_s1 = io.mshrHintQInfo.valid && (isGrantData(mshr_s1) || isMergeGrantData(mshr_s1))
  val mshr_Grant_s1     = io.mshrHintQInfo.valid && (isGrant(mshr_s1) || isMergeGrant(mshr_s1))
  val mshr_AccessAckData_s1 = io.mshrHintQInfo.valid && isAccessAckData(mshr_s1)
  val chn_Release_s1    = io.sinkCHintQInfo.valid
  assert(Mux(chn_Release_s1, sinkC_s1.fromC, true.B))
  assert(Mux(chn_Release_s1, sinkC_s1.opcode === Release || sinkC_s1.opcode === ReleaseData, true.B))

  val enqBits_s1 = Wire(new HintQueueEntry)
  // enqBits_s1.source := Mux(task_s1.bits.mergeA, task_s1.bits.aMergeTask.sourceId, task_s1.bits.sourceId)
  enqBits_s1.source := Mux1H(Seq(
    (io.mshrHintQInfo.valid && mshr_s1.mergeA) -> mshrMerge_s1.sourceId,
    (io.mshrHintQInfo.valid && !mshr_s1.mergeA) -> mshr_s1.sourceId,
    io.sinkCHintQInfo.valid -> sinkC_s1.sourceId
  ))
  assert(PopCount(Cat(io.mshrHintQInfo.valid && mshr_s1.mergeA, io.mshrHintQInfo.valid && !mshr_s1.mergeA, io.sinkCHintQInfo.valid)) <= 1.U)
  enqBits_s1.isKeyword := Mux(mshr_s1.mergeA, mshrMerge_s1.isKeyword.getOrElse(false.B), mshr_s1.isKeyword.getOrElse(false.B)) 
  enqBits_s1.isGrantData := mshr_GrantData_s1
  enqBits_s1.hasData := mshr_GrantData_s1 || mshr_AccessAckData_s1

  // Hint for "chnTask Hit" will fire@s3
  val chn_Grant_s3     = task_s3.valid && !mshrReq_s3 && !need_mshr_s3 && task_s3.bits.fromA && task_s3.bits.opcode === Grant
  val chn_GrantData_s3 = task_s3.valid && !mshrReq_s3 && !need_mshr_s3 && task_s3.bits.fromA && task_s3.bits.opcode === GrantData
  val chn_AccessAckData_s3 = task_s3.valid && !mshrReq_s3 && !need_mshr_s3 && task_s3.bits.fromA && task_s3.bits.opcode === AccessAckData
  val enqBits_s3 = Wire(new HintQueueEntry)
  val enqValid_s3 = chn_Grant_s3 || chn_GrantData_s3 || chn_AccessAckData_s3
  enqBits_s3.source := task_s3.bits.sourceId
  enqBits_s3.isKeyword := task_s3.bits.isKeyword.getOrElse(false.B)
  enqBits_s3.isGrantData := chn_GrantData_s3
  enqBits_s3.hasData := chn_GrantData_s3 || chn_AccessAckData_s3

  // ==================== Hint Queue ====================
  val hintEntries = mshrsAll
  val hintEntriesWidth = log2Ceil(hintEntries)
  val hintQueue = Module(new Queue(new HintQueueEntry, hintEntries))
  val canFlow_s1 = !hintQueue.io.deq.valid || hintQueue.io.count === 1.U && hintQueue.io.deq.fire
  val valid_s1 = mshr_GrantData_s1 || mshr_Grant_s1 || mshr_AccessAckData_s1 || chn_Release_s1
  val flow_s1, drop_s1, enq_s3 = Wire(Decoupled(new HintQueueEntry))
  // noSpaceForSinkReq in GrantBuffer may ensure that these queues will not overflow
  assert(enq_s3.ready || !enq_s3.valid)

  val hint_s1Queue = Module(new Pipeline(new HintQueueEntry))
  hint_s1Queue.io.in.valid := valid_s1 && (!canFlow_s1 || !flow_s1.ready)
  hint_s1Queue.io.in.bits  := enqBits_s1
  assert(!valid_s1 || hint_s1Queue.io.in.ready || flow_s1.ready)

  drop_s1.valid := hint_s1Queue.io.out.valid && !io.retry_s2
  drop_s1.bits := hint_s1Queue.io.out.bits
  hint_s1Queue.io.out.ready := drop_s1.ready || io.retry_s2

  flow_s1.valid := valid_s1 && canFlow_s1
  flow_s1.bits := enqBits_s1

  enq_s3.valid := enqValid_s3
  enq_s3.bits := enqBits_s3
  arb(Seq(enq_s3, drop_s1, flow_s1), hintQueue.io.enq, Some("Hint"))
  val respWithDataFire = io.l1Hint.valid && io.l1Hint.bits.hasData
  hintQueue.io.deq.ready := io.l1Hint.ready && !RegNext(respWithDataFire, false.B)

  io.l1Hint.valid := hintQueue.io.deq.valid && !(io.retry_s2 && !hint_s1Queue.io.out.valid) && !RegNext(respWithDataFire, false.B)
  io.l1Hint.bits.sourceId := hintQueue.io.deq.bits.source
  io.l1Hint.bits.isKeyword := hintQueue.io.deq.bits.isKeyword
  io.l1Hint.bits.hasData := hintQueue.io.deq.bits.hasData
}