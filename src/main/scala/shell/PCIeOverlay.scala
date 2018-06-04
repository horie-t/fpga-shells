// See LICENSE for license details.
package sifive.fpgashells.shell

import chisel3._
import freechips.rocketchip.config._
import freechips.rocketchip.diplomacy._
import freechips.rocketchip.tilelink._
import freechips.rocketchip.interrupts._
import sifive.fpgashells.clocks._

case class PCIeOverlayParams(
  wrangler: ClockAdapterNode)(
  implicit val p: Parameters)

case object PCIeOverlayKey extends Field[Seq[DesignOverlay[PCIeOverlayParams, (TLNode, IntOutwardNode)]]](Nil)

abstract class PCIeOverlay[IO <: Data](
  val shell: Shell,
  val params: PCIeOverlayParams)
    extends OverlayGenerator[(TLNode, IntOutwardNode), IO]
{
  implicit val p = params.p
}
