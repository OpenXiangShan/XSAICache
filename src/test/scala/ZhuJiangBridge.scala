package zhujiang

import chisel3._
import chisel3.util.Cat
import freechips.rocketchip.amba.axi4.AXI4Bundle
import org.chipsalliance.cde.config.Parameters
import xijiang.{Node, NodeType}
import xscache.chi.{CHIDAT, CHIREQ, CHIRSP, CHISNP, DecoupledPortIO, MemAttr}
import zhujiang.axi.ExtAxiBundle
import zhujiang.chi.{DataFlit, ReqFlit, RespFlit, SnoopFlit}
import zhujiang.device.socket.{SocketDevSide, SocketIcnSideBundle}

trait HasZhuJiangAXI4Bridge {
  protected def connectZJMemToAXI4(zjMem: ExtAxiBundle, axi: AXI4Bundle): Unit = {
    require(zjMem.params.addrBits <= axi.params.addrBits)
    require(zjMem.params.dataBits == axi.params.dataBits)
    require(zjMem.params.idBits <= axi.params.idBits)
    require(zjMem.params.lenBits == axi.params.lenBits)
    require(zjMem.params.sizeBits == axi.params.sizeBits)
    require(zjMem.params.burstBits == axi.params.burstBits)
    require(zjMem.params.lockBits == axi.params.lockBits)
    require(zjMem.params.cacheBits == axi.params.cacheBits)
    require(zjMem.params.qosBits == axi.params.qosBits)

    axi.aw.valid := zjMem.awvalid
    zjMem.awready := axi.aw.ready
    axi.aw.bits.id := zjMem.awid
    axi.aw.bits.addr := zjMem.awaddr
    axi.aw.bits.len := zjMem.awlen
    axi.aw.bits.size := zjMem.awsize
    axi.aw.bits.burst := zjMem.awburst
    axi.aw.bits.lock := zjMem.awlock
    axi.aw.bits.cache := zjMem.awcache
    axi.aw.bits.prot := zjMem.awprot
    axi.aw.bits.qos := zjMem.awqos
    axi.aw.bits.user := DontCare
    axi.aw.bits.echo := DontCare

    axi.ar.valid := zjMem.arvalid
    zjMem.arready := axi.ar.ready
    axi.ar.bits.id := zjMem.arid
    axi.ar.bits.addr := zjMem.araddr
    axi.ar.bits.len := zjMem.arlen
    axi.ar.bits.size := zjMem.arsize
    axi.ar.bits.burst := zjMem.arburst
    axi.ar.bits.lock := zjMem.arlock
    axi.ar.bits.cache := zjMem.arcache
    axi.ar.bits.prot := zjMem.arprot
    axi.ar.bits.qos := zjMem.arqos
    axi.ar.bits.user := DontCare
    axi.ar.bits.echo := DontCare

    axi.w.valid := zjMem.wvalid
    zjMem.wready := axi.w.ready
    axi.w.bits.data := zjMem.wdata
    axi.w.bits.strb := zjMem.wstrb
    axi.w.bits.last := zjMem.wlast.asBool
    axi.w.bits.user := DontCare

    zjMem.bvalid := axi.b.valid
    axi.b.ready := zjMem.bready
    zjMem.bid := axi.b.bits.id
    zjMem.bresp := axi.b.bits.resp
    zjMem.buser := 0.U

    zjMem.rvalid := axi.r.valid
    axi.r.ready := zjMem.rready
    zjMem.rid := axi.r.bits.id
    zjMem.rdata := axi.r.bits.data
    zjMem.rresp := axi.r.bits.resp
    zjMem.rlast := axi.r.bits.last
    zjMem.ruser := 0.U
  }
}

trait HasCHIToZhuJiangBridge {
  protected def connectCHIToZhuJiang(
    chi: DecoupledPortIO,
    socketIO: SocketIcnSideBundle,
    ccNode: Node
  )(implicit p: Parameters): Unit = {
    require(ccNode.nodeType == NodeType.CC)
    require(ccNode.socket == "sync")

    val socket = Module(new SocketDevSide(ccNode))
    socketIO <> socket.io.socket

    // CHI -> ZhuJiang icn.rx
    val txreqBits = Wire(new ReqFlit)
    mapReq(txreqBits, chi.tx.req.bits)
    socket.io.icn.rx.req.get.valid := chi.tx.req.valid
    socket.io.icn.rx.req.get.bits := txreqBits
    chi.tx.req.ready := socket.io.icn.rx.req.get.ready

    val txrspBits = Wire(new RespFlit)
    mapRsp(txrspBits, chi.tx.rsp.bits)
    socket.io.icn.rx.resp.get.valid := chi.tx.rsp.valid
    socket.io.icn.rx.resp.get.bits := txrspBits
    chi.tx.rsp.ready := socket.io.icn.rx.resp.get.ready

    val txdatBits = Wire(new DataFlit)
    mapDat(txdatBits, chi.tx.dat.bits)
    socket.io.icn.rx.data.get.valid := chi.tx.dat.valid
    socket.io.icn.rx.data.get.bits := txdatBits
    chi.tx.dat.ready := socket.io.icn.rx.data.get.ready

    socket.io.icn.rx.hpr.foreach { hpr =>
      hpr.valid := false.B
      hpr.bits := 0.U.asTypeOf(hpr.bits)
    }
    socket.io.icn.rx.snoop.foreach { snp =>
      snp.valid := false.B
      snp.bits := 0.U.asTypeOf(snp.bits)
    }
    socket.io.icn.rx.debug.foreach { dbg =>
      dbg.valid := false.B
      dbg.bits := 0.U.asTypeOf(dbg.bits)
    }

    // ZhuJiang icn.tx -> CHI
    val rxrspBits = Wire(new CHIRSP)
    mapRsp(rxrspBits, socket.io.icn.tx.resp.get.bits.asTypeOf(new RespFlit))
    chi.rx.rsp.valid := socket.io.icn.tx.resp.get.valid
    chi.rx.rsp.bits := rxrspBits
    socket.io.icn.tx.resp.get.ready := chi.rx.rsp.ready

    val rxdatBits = Wire(new CHIDAT)
    mapDat(rxdatBits, socket.io.icn.tx.data.get.bits.asTypeOf(new DataFlit))
    chi.rx.dat.valid := socket.io.icn.tx.data.get.valid
    chi.rx.dat.bits := rxdatBits
    socket.io.icn.tx.data.get.ready := chi.rx.dat.ready

    val rxsnpBits = Wire(new CHISNP)
    mapSnp(rxsnpBits, socket.io.icn.tx.snoop.get.bits.asTypeOf(new SnoopFlit))
    chi.rx.snp.valid := socket.io.icn.tx.snoop.get.valid
    chi.rx.snp.bits := rxsnpBits
    socket.io.icn.tx.snoop.get.ready := chi.rx.snp.ready

    socket.io.icn.tx.req.foreach(_.ready := false.B)
    socket.io.icn.tx.hpr.foreach(_.ready := false.B)
    socket.io.icn.tx.debug.foreach(_.ready := false.B)
  }

  protected def mapReq(dst: ReqFlit, src: CHIREQ): Unit = {
    dst := 0.U.asTypeOf(dst)
    dst.QoS := src.qos
    dst.TgtID := src.tgtID
    dst.SrcID := src.srcID
    dst.TxnID := src.txnID
    dst.Opcode := src.opcode
    dst.Size := src.size
    dst.Addr := src.addr
    dst.Order := src.order
    dst.MemAttr := Cat(src.memAttr.allocate, src.memAttr.cacheable, src.memAttr.device, src.memAttr.ewa)
    dst.SnpAttr := src.snpAttr
    dst.Excl := src.excl
    dst.ExpCompAck := src.expCompAck
    dst.RSVDC := src.rsvdc
    dst.PBHA := src.mpam.map(_.partID).getOrElse(0.U)
    dst.MPAM := src.mpam.map(_.partID).getOrElse(0.U)
    dst.ReturnTxnID.foreach(_ := src.returnTxnID)
    dst.ReturnNID.foreach(_ := src.returnNID)
  }

  protected def mapRsp(dst: RespFlit, src: CHIRSP): Unit = {
    dst := 0.U.asTypeOf(dst)
    dst.QoS := src.qos
    dst.TgtID := src.tgtID
    dst.SrcID := src.srcID
    dst.TxnID := src.txnID
    dst.Opcode := src.opcode
    dst.RespErr := src.respErr
    dst.Resp := src.resp
    dst.FwdState := src.fwdState
    dst.CBusy := src.cBusy.getOrElse(0.U)
    dst.DBID := src.dbID
  }

  protected def mapDat(dst: DataFlit, src: CHIDAT): Unit = {
    dst := 0.U.asTypeOf(dst)
    dst.QoS := src.qos
    dst.TgtID := src.tgtID
    dst.SrcID := src.srcID
    dst.TxnID := src.txnID
    dst.HomeNID := src.homeNID
    dst.Opcode := src.opcode
    dst.RespErr := src.respErr
    dst.Resp := src.resp
    dst.DataSource := src.dataSource
    dst.CBusy := src.cBusy.getOrElse(0.U)
    dst.DBID := src.dbID
    dst.DataID := src.dataID
    dst.RSVDC := src.rsvdc
    dst.BE := src.be
    dst.Data := src.data
  }

  protected def mapSnp(dst: SnoopFlit, src: CHISNP): Unit = {
    dst := 0.U.asTypeOf(dst)
    dst.QoS := src.qos
    dst.TgtID := src.fwdNID
    dst.SrcID := src.srcID
    dst.TxnID := src.txnID
    dst.FwdNID := src.fwdNID
    dst.FwdTxnID := src.fwdTxnID
    dst.Opcode := src.opcode
    dst.Addr := src.addr
    dst.DoNotGoToSD := src.doNotGoToSD
    dst.RetToSrc := src.retToSrc
    dst.MECID := src.mpam.map(_.partID).getOrElse(0.U)
    dst.MPAM := src.mpam.map(_.partID).getOrElse(0.U)
  }

  protected def mapReq(dst: CHIREQ, src: ReqFlit): Unit = {
    dst := 0.U.asTypeOf(dst)
    dst.qos := src.QoS
    dst.tgtID := src.TgtID
    dst.srcID := src.SrcID
    dst.txnID := src.TxnID
    dst.opcode := src.Opcode
    dst.size := src.Size
    dst.addr := src.Addr
    dst.order := src.Order
    dst.memAttr := MemAttr(
      allocate = src.MemAttr(3),
      cacheable = src.MemAttr(2),
      device = src.MemAttr(1),
      ewa = src.MemAttr(0)
    )
    dst.snpAttr := src.SnpAttr
    dst.snoopMe := src.Excl
    dst.expCompAck := src.ExpCompAck
    dst.rsvdc := src.RSVDC
    dst.mpam.foreach { m =>
      m.perfMonGroup := 0.U
      m.partID := src.PBHA
      m.mpamNS := false.B
    }
    dst.returnTxnID := src.ReturnTxnID.getOrElse(0.U)
    dst.returnNID := src.ReturnNID.getOrElse(0.U)
  }

  protected def mapRsp(dst: CHIRSP, src: RespFlit): Unit = {
    dst := 0.U.asTypeOf(dst)
    dst.qos := src.QoS
    dst.tgtID := src.TgtID
    dst.srcID := src.SrcID
    dst.txnID := src.TxnID
    dst.opcode := src.Opcode
    dst.respErr := src.RespErr
    dst.resp := src.Resp
    dst.fwdState := src.FwdState
    dst.cBusy.foreach(_ := src.CBusy)
    dst.dbID := src.DBID
  }

  protected def mapDat(dst: CHIDAT, src: DataFlit): Unit = {
    dst := 0.U.asTypeOf(dst)
    dst.qos := src.QoS
    dst.tgtID := src.TgtID
    dst.srcID := src.SrcID
    dst.txnID := src.TxnID
    dst.homeNID := src.HomeNID
    dst.opcode := src.Opcode
    dst.respErr := src.RespErr
    dst.resp := src.Resp
    dst.dataSource := src.DataSource
    dst.cBusy.foreach(_ := src.CBusy)
    dst.dbID := src.DBID
    dst.dataID := src.DataID
    dst.rsvdc := src.RSVDC
    dst.be := src.BE
    dst.data := src.Data
  }

  protected def mapSnp(dst: CHISNP, src: SnoopFlit): Unit = {
    dst := 0.U.asTypeOf(dst)
    dst.qos := src.QoS
    dst.srcID := src.SrcID
    dst.txnID := src.TxnID
    dst.fwdNID := src.FwdNID
    dst.fwdTxnID := src.FwdTxnID
    dst.opcode := src.Opcode
    dst.addr := src.Addr
    dst.doNotGoToSD := src.DoNotGoToSD
    dst.retToSrc := src.RetToSrc
    dst.mpam.foreach { m =>
      m.perfMonGroup := 0.U
      m.partID := src.MECID
      m.mpamNS := false.B
    }
  }
}
