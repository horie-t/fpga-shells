// See LICENSE for license details.
// See LICENSE.HORIE_Tetsuya for license details.
package sifive.fpgashells.shell.xilinx.nexys4ddrshell

import Chisel._
import chisel3.core.{Input, Output, attach}
import chisel3.experimental.{RawModule, Analog, withClockAndReset}

import freechips.rocketchip.config._
import freechips.rocketchip.devices.debug._

import sifive.blocks.devices.gpio._
import sifive.blocks.devices.pwm._
import sifive.blocks.devices.spi._
import sifive.blocks.devices.uart._
import sifive.blocks.devices.pinctrl.{BasePin}

import sifive.fpgashells.ip.xilinx.{IBUFG, IOBUF, PULLUP, mmcm4, reset_sys, PowerOnResetFPGAOnly}

//-------------------------------------------------------------------------
// Nexys4DDRShell
//-------------------------------------------------------------------------

abstract class Nexys4DDRShell(implicit val p: Parameters) extends RawModule {

  //-----------------------------------------------------------------------
  // Interface
  //-----------------------------------------------------------------------

  // Clock & Reset
  val CLK100MHZ    = IO(Input(Clock()))
  val ck_rst       = IO(Input(Bool()))

  // Green LEDs
  val led_0        = IO(Analog(1.W))
  val led_1        = IO(Analog(1.W))
  val led_2        = IO(Analog(1.W))
  val led_3        = IO(Analog(1.W))
  val led_4        = IO(Analog(1.W))
  val led_5        = IO(Analog(1.W))
  val led_6        = IO(Analog(1.W))
  val led_7        = IO(Analog(1.W))
  val led_8        = IO(Analog(1.W))
  val led_9        = IO(Analog(1.W))
  val led_10        = IO(Analog(1.W))
  val led_11        = IO(Analog(1.W))
  val led_12        = IO(Analog(1.W))
  val led_13        = IO(Analog(1.W))
  val led_14        = IO(Analog(1.W))
  val led_15        = IO(Analog(1.W))

  // 7Segment LEDs
  val seg7_ca        = IO(Analog(1.W))
  val seg7_cb        = IO(Analog(1.W))
  val seg7_cc        = IO(Analog(1.W))
  val seg7_cd        = IO(Analog(1.W))
  val seg7_ce        = IO(Analog(1.W))
  val seg7_cf        = IO(Analog(1.W))
  val seg7_cg        = IO(Analog(1.W))

  val seg7_dp        = IO(Analog(1.W))

  val seg7_an_0      = IO(Analog(1.W))
  val seg7_an_1      = IO(Analog(1.W))
  val seg7_an_2      = IO(Analog(1.W))
  val seg7_an_3      = IO(Analog(1.W))
  val seg7_an_4      = IO(Analog(1.W))
  val seg7_an_5      = IO(Analog(1.W))
  val seg7_an_6      = IO(Analog(1.W))
  val seg7_an_7      = IO(Analog(1.W))

  // RGB LEDs, 3 pins each
  val led0_r       = IO(Analog(1.W))
  val led0_g       = IO(Analog(1.W))
  val led0_b       = IO(Analog(1.W))

  val led1_r       = IO(Analog(1.W))
  val led1_g       = IO(Analog(1.W))
  val led1_b       = IO(Analog(1.W))

  // Sliding switches
  val sw_0         = IO(Analog(1.W))
  val sw_1         = IO(Analog(1.W))
  val sw_2         = IO(Analog(1.W))
  val sw_3         = IO(Analog(1.W))

  // Buttons. First 3 used as GPIO, the last is used as wakeup
  val btn_0        = IO(Analog(1.W))
  val btn_1        = IO(Analog(1.W))
  val btn_2        = IO(Analog(1.W))
  val btn_3        = IO(Analog(1.W))

  // Dedicated QSPI interface
  val qspi_cs      = IO(Analog(1.W))
  val qspi_sck     = IO(Analog(1.W))
  val qspi_dq      = IO(Vec(4, Analog(1.W)))

  // UART0
  val uart_rxd_out = IO(Analog(1.W))
  val uart_txd_in  = IO(Analog(1.W))
  val uart_cts     = IO(Analog(1.W))
  val uart_rts     = IO(Analog(1.W))

  // VGA Diplay
  val vga_red_0    = IO(Analog(1.W))
  val vga_red_1    = IO(Analog(1.W))
  val vga_red_2    = IO(Analog(1.W))
  val vga_red_3    = IO(Analog(1.W))

  val vga_green_0  = IO(Analog(1.W))
  val vga_green_1  = IO(Analog(1.W))
  val vga_green_2  = IO(Analog(1.W))
  val vga_green_3  = IO(Analog(1.W))

  val vga_blue_0   = IO(Analog(1.W))
  val vga_blue_1   = IO(Analog(1.W))
  val vga_blue_2   = IO(Analog(1.W))
  val vga_blue_3   = IO(Analog(1.W))

  val vga_hSync    = IO(Analog(1.W))
  val vga_vSync    = IO(Analog(1.W))

  // JA (Used for more generic GPIOs)
  val ja_0         = IO(Analog(1.W))
  val ja_1         = IO(Analog(1.W))
  val ja_2         = IO(Analog(1.W))
  val ja_3         = IO(Analog(1.W))
  val ja_4         = IO(Analog(1.W))
  val ja_5         = IO(Analog(1.W))
  val ja_6         = IO(Analog(1.W))
  val ja_7         = IO(Analog(1.W))

  // JD (used for JTAG connection)
  val jd_0         = IO(Analog(1.W))  // TDO
  val jd_1         = IO(Analog(1.W))  // TRST_n
  val jd_2         = IO(Analog(1.W))  // TCK
  val jd_4         = IO(Analog(1.W))  // TDI
  val jd_5         = IO(Analog(1.W))  // TMS
  val jd_6         = IO(Analog(1.W))  // SRST_n

  //-----------------------------------------------------------------------
  // Wire declrations
  //-----------------------------------------------------------------------

  // Note: these frequencies are approximate.
  val clock_8MHz     = Wire(Clock())
  val clock_32MHz    = Wire(Clock())
  val clock_65MHz    = Wire(Clock())
  val clock_25MHz    = Wire(Clock())

  val mmcm_locked    = Wire(Bool())

  val reset_core     = Wire(Bool())
  val reset_bus      = Wire(Bool())
  val reset_periph   = Wire(Bool())
  val reset_intcon_n = Wire(Bool())
  val reset_periph_n = Wire(Bool())

  val SRST_n         = Wire(Bool())

  val dut_jtag_TCK   = Wire(Clock())
  val dut_jtag_TMS   = Wire(Bool())
  val dut_jtag_TDI   = Wire(Bool())
  val dut_jtag_TDO   = Wire(Bool())
  val dut_jtag_reset = Wire(Bool())
  val dut_ndreset    = Wire(Bool())

  //-----------------------------------------------------------------------
  // Clock Generator
  //-----------------------------------------------------------------------
  // Mixed-mode clock generator

  val ip_mmcm = Module(new mmcm4())

  ip_mmcm.io.clk_in1 := CLK100MHZ
  clock_8MHz         := ip_mmcm.io.clk_out1  // 8.388 MHz = 32.768 kHz * 256
  clock_65MHz        := ip_mmcm.io.clk_out2  // 65 Mhz
  clock_32MHz        := ip_mmcm.io.clk_out3  // 65/2 Mhz
  clock_25MHz        := ip_mmcm.io.clk_out4  // 25MHz (VGA Px Clock)
  ip_mmcm.io.resetn  := ck_rst
  mmcm_locked        := ip_mmcm.io.locked

  //-----------------------------------------------------------------------
  // System Reset
  //-----------------------------------------------------------------------
  // processor system reset module

  val ip_reset_sys = Module(new reset_sys())

  ip_reset_sys.io.slowest_sync_clk := clock_8MHz
  ip_reset_sys.io.ext_reset_in     := ck_rst & SRST_n
  ip_reset_sys.io.aux_reset_in     := true.B
  ip_reset_sys.io.mb_debug_sys_rst := dut_ndreset
  ip_reset_sys.io.dcm_locked       := mmcm_locked

  reset_core                       := ip_reset_sys.io.mb_reset
  reset_bus                        := ip_reset_sys.io.bus_struct_reset
  reset_periph                     := ip_reset_sys.io.peripheral_reset
  reset_intcon_n                   := ip_reset_sys.io.interconnect_aresetn
  reset_periph_n                   := ip_reset_sys.io.peripheral_aresetn

  //-----------------------------------------------------------------------
  // SPI Flash
  //-----------------------------------------------------------------------

  def connectSPIFlash(dut: HasPeripherySPIFlashModuleImp): Unit = {
    val qspiParams = p(PeripherySPIFlashKey)
    if (!qspiParams.isEmpty) {
      val qspi_params = qspiParams(0)
      val qspi_pins = Wire(new SPIPins(() => {new BasePin()}, qspi_params))

      SPIPinsFromPort(qspi_pins,
        dut.qspi(0),
        dut.clock,
        dut.reset,
        syncStages = qspi_params.sampleDelay
      )

      IOBUF(qspi_sck, dut.qspi(0).sck)
      IOBUF(qspi_cs,  dut.qspi(0).cs(0))

      (qspi_dq zip qspi_pins.dq).foreach {
        case(a, b) => IOBUF(a,b)
      }
    }
  }

  //---------------------------------------------------------------------
  // Debug JTAG
  //---------------------------------------------------------------------

  def connectDebugJTAG(dut: HasPeripheryDebugModuleImp): SystemJTAGIO = {

    //-------------------------------------------------------------------
    // JTAG Reset
    //-------------------------------------------------------------------

    val jtag_power_on_reset = PowerOnResetFPGAOnly(clock_32MHz)

    dut_jtag_reset := jtag_power_on_reset

    //-------------------------------------------------------------------
    // JTAG IOBUFs
    //-------------------------------------------------------------------

    dut_jtag_TCK  := IBUFG(IOBUF(jd_2).asClock)

    dut_jtag_TMS  := IOBUF(jd_5)
    PULLUP(jd_5)

    dut_jtag_TDI  := IOBUF(jd_4)
    PULLUP(jd_4)

    IOBUF(jd_0, dut_jtag_TDO)

    SRST_n := IOBUF(jd_6)
    PULLUP(jd_6)

    //-------------------------------------------------------------------
    // JTAG PINS
    //-------------------------------------------------------------------

    val djtag     = dut.debug.systemjtag.get

    djtag.jtag.TCK := dut_jtag_TCK
    djtag.jtag.TMS := dut_jtag_TMS
    djtag.jtag.TDI := dut_jtag_TDI
    dut_jtag_TDO   := djtag.jtag.TDO.data

    djtag.mfr_id   := p(JtagDTMKey).idcodeManufId.U(11.W)

    djtag.reset    := dut_jtag_reset
    dut_ndreset    := dut.debug.ndreset

    djtag
  }

  //---------------------------------------------------------------------
  // UART
  //---------------------------------------------------------------------

  def connectUART(dut: HasPeripheryUARTModuleImp): Unit = {
    val uartParams = p(PeripheryUARTKey)
    if (!uartParams.isEmpty) {
      IOBUF(uart_rxd_out, dut.uart(0).txd)
      dut.uart(0).rxd := IOBUF(uart_txd_in)
    }
  }

}
