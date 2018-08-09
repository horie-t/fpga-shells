// See LICENSE for license details.
package sifive.fpgashells.shell.xilinx.u500nexys4ddrshell

import Chisel._
import chisel3.core.{Input, Output, attach}
import chisel3.experimental.{RawModule, Analog, withClockAndReset}

import freechips.rocketchip.config._
import freechips.rocketchip.devices.debug._
import freechips.rocketchip.util.{SyncResetSynchronizerShiftReg, ElaborationArtefacts, HeterogeneousBag}

import sifive.blocks.devices.gpio._
import sifive.blocks.devices.spi._
import sifive.blocks.devices.uart._
import sifive.blocks.devices.chiplink._

import sifive.fpgashells.devices.xilinx.xilinxnexys4ddrmig._
import sifive.fpgashells.ip.xilinx.{IBUFDS, PowerOnResetFPGAOnly, sdio_spi_bridge, Series7MMCM, nexys4ddr_reset}

import sifive.fpgashells.clocks._
//-------------------------------------------------------------------------
// Nexys4DDRShell
//-------------------------------------------------------------------------

trait HasDDR2 { this: U500Nexys4DDRShell =>
  
  require(!p.lift(MemoryXilinxDDRKey).isEmpty)
  val ddr = IO(new XilinxNexys4DDRMIGPads(p(MemoryXilinxDDRKey)))
  
  def connectMIG(dut: HasMemoryXilinxNexys4DDRMIGModuleImp): Unit = {
    // Clock & Reset
    dut.xilinxnexys4ddrmig.sys_clk_i := sys_clock.asUInt
    mig_clock                    := dut.xilinxnexys4ddrmig.ui_clk
    mig_sys_reset                := dut.xilinxnexys4ddrmig.ui_clk_sync_rst
    mig_mmcm_locked              := dut.xilinxnexys4ddrmig.mmcm_locked
    dut.xilinxnexys4ddrmig.aresetn   := mig_resetn
    dut.xilinxnexys4ddrmig.sys_rst   := sys_reset

    ddr <> dut.xilinxnexys4ddrmig
  }
}

abstract class U500Nexys4DDRShell(implicit val p: Parameters) extends RawModule {

  //-----------------------------------------------------------------------
  // Interface
  //-----------------------------------------------------------------------

  // clock
  val clock = IO(Input(Clock()))

  // active low reset
  val resetn               = IO(Input(Bool()))

  // LED
  val led                  = IO(Vec(8, Output(Bool())))

  // UART
  val uart_tx              = IO(Output(Bool()))
  val uart_rx              = IO(Input(Bool()))
  val uart_rtsn            = IO(Output(Bool()))
  val uart_ctsn            = IO(Input(Bool()))

  // SDIO
  val sdio_clk             = IO(Output(Bool()))
  val sdio_cmd             = IO(Analog(1.W))
  val sdio_dat             = IO(Analog(4.W))

  //Buttons
  val btn_0                = IO(Analog(1.W))
  val btn_1                = IO(Analog(1.W))
  val btn_2                = IO(Analog(1.W))
  val btn_3                = IO(Analog(1.W))

  //Sliding switches
  val sw_0                 = IO(Analog(1.W))
  val sw_1                 = IO(Analog(1.W))
  val sw_2                 = IO(Analog(1.W))
  val sw_3                 = IO(Analog(1.W))
  val sw_4                 = IO(Analog(1.W))
  val sw_5                 = IO(Analog(1.W))
  val sw_6                 = IO(Analog(1.W))
  val sw_7                 = IO(Analog(1.W))


  //-----------------------------------------------------------------------
  // Wire declrations
  //-----------------------------------------------------------------------

  val sys_clock       = Wire(Clock())
  val sys_reset       = Wire(Bool())

  val dut_clock       = Wire(Clock())
  val dut_reset       = Wire(Bool())
  val dut_resetn      = Wire(Bool())

  val dut_ndreset     = Wire(Bool())

  val sd_spi_sck      = Wire(Bool())
  val sd_spi_cs       = Wire(Bool())
  val sd_spi_dq_i     = Wire(Vec(4, Bool()))
  val sd_spi_dq_o     = Wire(Vec(4, Bool()))

  val do_reset        = Wire(Bool())

  val mig_mmcm_locked = Wire(Bool())
  val mig_sys_reset   = Wire(Bool())

  val mig_clock       = Wire(Clock())
  val mig_reset       = Wire(Bool())
  val mig_resetn      = Wire(Bool())

  val pcie_dat_reset  = Wire(Bool())
  val pcie_dat_resetn = Wire(Bool())
  val pcie_cfg_reset  = Wire(Bool())
  val pcie_cfg_resetn = Wire(Bool())
  val pcie_dat_clock  = Wire(Clock())
  val pcie_cfg_clock  = Wire(Clock())
  val mmcm_lock_pcie  = Wire(Bool())

  //-----------------------------------------------------------------------
  // System clock and reset
  //-----------------------------------------------------------------------

  // Clock that drives the clock generator and the MIG
  sys_clock := clock

  // Allow the debug module to reset everything. Resets the MIG
  sys_reset := ~resetn | dut_ndreset

  //-----------------------------------------------------------------------
  // Clock Generator
  //-----------------------------------------------------------------------

  //25MHz and multiples
  val nexys4ddr_sys_clock_mmcm0 = Module(new Series7MMCM(PLLParameters(
    "nexys4ddr_sys_clock_mmcm2",
    InClockParameters(100, 50),
    Seq(
      OutClockParameters(12.5),
      OutClockParameters(25),
      OutClockParameters(37.5),
      OutClockParameters(50),
      OutClockParameters(100),
      OutClockParameters(150.00),
      OutClockParameters(100, 180)))))
  
  nexys4ddr_sys_clock_mmcm0.io.clk_in1 := sys_clock.asUInt
  nexys4ddr_sys_clock_mmcm0.io.reset   := ~resetn
  val nexys4ddr_sys_clock_mmcm0_locked = nexys4ddr_sys_clock_mmcm0.io.locked
  val Seq(clk12_5, clk25, clk37_5, clk50, clk100, clk150, clk100_180) = nexys4ddr_sys_clock_mmcm0.getClocks

  //65MHz and multiples
  //val vc707_sys_clock_mmcm1 = Module(new vc707_sys_clock_mmcm1)
  val nexys4ddr_sys_clock_mmcm1 = Module(new Series7MMCM(PLLParameters(
    "nexys4ddr_sys_clock_mmcm1",
    InClockParameters(100, 50),
    Seq(
      OutClockParameters(32.5),
      OutClockParameters(65, 180)))))
  
  nexys4ddr_sys_clock_mmcm1.io.clk_in1 := sys_clock.asUInt
  nexys4ddr_sys_clock_mmcm1.io.reset   := ~resetn
  val clk32_5              = nexys4ddr_sys_clock_mmcm1.io.clk_out1
  val clk65                = nexys4ddr_sys_clock_mmcm1.io.clk_out2
  val nexys4ddr_sys_clock_mmcm1_locked = nexys4ddr_sys_clock_mmcm1.io.locked

  // DUT clock
  dut_clock := clk37_5

  //-----------------------------------------------------------------------
  // System reset
  //-----------------------------------------------------------------------

  do_reset             := !mig_mmcm_locked || mig_sys_reset || !nexys4ddr_sys_clock_mmcm0_locked ||
                          !nexys4ddr_sys_clock_mmcm1_locked
  mig_resetn           := !mig_reset
  dut_resetn           := !dut_reset

  val safe_reset = Module(new nexys4ddr_reset)

  safe_reset.io.areset := do_reset
  safe_reset.io.clock1 := mig_clock
  mig_reset            := safe_reset.io.reset1
  safe_reset.io.clock2 := dut_clock
  dut_reset            := safe_reset.io.reset2

  //overrided in connectMIG
  //provide defaults to allow above reset sequencing logic to work without both
  mig_clock            := dut_clock
  mig_mmcm_locked      := UInt("b1")

  //-----------------------------------------------------------------------
  // UART
  //-----------------------------------------------------------------------

  uart_rtsn := false.B

  def connectUART(dut: HasPeripheryUARTModuleImp): Unit = {
    val uartParams = p(PeripheryUARTKey)
    if (!uartParams.isEmpty) {
      // uart connections
      dut.uart(0).rxd := SyncResetSynchronizerShiftReg(uart_rx, 2, init = Bool(true), name=Some("uart_rxd_sync"))
      uart_tx         := dut.uart(0).txd
    }
  }

  //-----------------------------------------------------------------------
  // SPI
  //-----------------------------------------------------------------------

  def connectSPI(dut: HasPeripherySPIModuleImp): Unit = {
    // SPI
    sd_spi_sck := dut.spi(0).sck
    sd_spi_cs  := dut.spi(0).cs(0)

    dut.spi(0).dq.zipWithIndex.foreach {
      case(pin, idx) =>
        sd_spi_dq_o(idx) := pin.o
        pin.i            := sd_spi_dq_i(idx)
    }

    //-------------------------------------------------------------------
    // SDIO <> SPI Bridge
    //-------------------------------------------------------------------

    val ip_sdio_spi = Module(new sdio_spi_bridge())

    ip_sdio_spi.io.clk   := dut_clock
    ip_sdio_spi.io.reset := dut_reset

    // SDIO
    attach(sdio_dat, ip_sdio_spi.io.sd_dat)
    attach(sdio_cmd, ip_sdio_spi.io.sd_cmd)
    sdio_clk := ip_sdio_spi.io.spi_sck

    // SPI
    ip_sdio_spi.io.spi_sck  := sd_spi_sck
    ip_sdio_spi.io.spi_cs   := sd_spi_cs
    sd_spi_dq_i             := ip_sdio_spi.io.spi_dq_i.toBools
    ip_sdio_spi.io.spi_dq_o := sd_spi_dq_o.asUInt
  }

}
