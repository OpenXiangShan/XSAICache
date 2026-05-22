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

package xscache.common

import chisel3._
import freechips.rocketchip.util.{BundleField, ControlKey}

case object PrefetchKey extends ControlKey[Bool](name = "needHint")

case class PrefetchField() extends BundleField[Bool](PrefetchKey, Output(Bool()), _ := false.B)

case object DirtyKey extends ControlKey[Bool](name = "dirty")

case class DirtyField() extends BundleField[Bool](DirtyKey, Output(Bool()), _ := false.B)

case object AliasKey extends ControlKey[UInt]("alias")

case class AliasField(width: Int) extends BundleField[UInt](AliasKey, Output(UInt(width.W)), _ := 0.U(width.W))

case object IsHitKey extends ControlKey[Bool](name = "isHitInL3")

case class IsHitField() extends BundleField[Bool](IsHitKey, Output(Bool()), _ := true.B)
