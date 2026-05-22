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

package xscache.coupledL2

import chisel3._
import chisel3.util._
import org.chipsalliance.cde.config.Parameters
import freechips.rocketchip.tilelink.TLPermissions._
import utility.MemReqSource
import xscache.coupledL2.{HasTLChannelBits, DirResult, PipeStatus}
import xscache.chi.HasCHIOpcodes

trait HasCHIChannelBits { this: Bundle =>
  val txChannel = UInt(3.W)
  def toTXREQ = txChannel(0).asBool
  def toTXRSP = txChannel(1).asBool
  def toTXDAT = txChannel(2).asBool
}

class PipeStatusWithCHI(implicit p: Parameters) extends PipeStatus
  with HasCHIChannelBits {
  val mshrTask = Bool()
}

class PCrdQueryBundle(implicit p: Parameters) extends CoupledL2Bundle with HasCHIOpcodes {
  val query = Output(ValidIO(new Bundle() {
    val pCrdType = UInt(PCRDTYPE_WIDTH.W)
    val srcID = UInt(SRCID_WIDTH.W)
  }))
  val grant = Input(Bool())
}
