# Ref: IBM PS/2 Hardware Technical Reference ­- Keyboard and Auxiliary Device Controller
# Accession: G00031
#
# It is often asserted in materials about the PS/2 protocol with varying degrees of confidence
# that the protocol is byte-oriented. This is, essentially, false. The protocol is packet-oriented,
# with the host always sending 1-byte packets, and the device responding with fixed-size packets,
# although the packet size is not always known in advance, e.g. in case of the identification data
# and keyboard scan codes.
#
# The host and the device may synchronize by using the fact that the device will always interrupt
# the current packet it is sending, whether solicited or unsolicited, and start processing
# the newly sent command. While well-defined on protocol level, this presents a problem for Glasgow
# because buffering results in the host not being always aware of precisely how many bytes of
# the packet have already been sent by the device.
#
# To ensure the host and the device always stay in sync, this implementation of a PS/2 host only
# queues response bytes when explicitly requested

import logging
import operator
import asyncio
from migen import *
from migen.genlib.cdc import MultiReg
from migen.genlib.fifo import SyncFIFOBuffered

from ... import *


class PS2Bus(Module):
    def __init__(self, pads):
        self.sample  = Signal()
        self.setup   = Signal()
        self.clock_i = Signal(reset=1)
        self.clock_o = Signal(reset=1)
        self.data_i  = Signal(reset=1)
        self.data_o  = Signal(reset=1)

        ###

        self.comb += [
            pads.clock_t.o.eq(0),
            pads.clock_t.oe.eq(~self.clock_o),
            pads.data_t.o.eq(0),
            pads.data_t.oe.eq(~self.data_o),
        ]
        self.specials += [
            MultiReg(pads.clock_t.i, self.clock_i, reset=1),
            MultiReg(pads.data_t.i,  self.data_i,  reset=1),
        ]

        clock_s = Signal(reset=1)
        clock_r = Signal(reset=1)
        self.sync += [
            clock_s.eq(self.clock_i),
            clock_r.eq(clock_s),
            self.sample.eq( clock_r & ~clock_s),
            self.setup .eq(~clock_r &  clock_s),
        ]


class PS2HostSubtarget(Module):
    def __init__(self, pads, in_fifo, out_fifo, request_cyc):
        self.submodules.bus = bus = PS2Bus(pads)

        frame_layout = [
            ("start",  1),
            ("data",   8),
            ("parity", 1),
            ("stop",   1),
        ]
        def check_frame(frame):
            return (
                (frame.start == 0) &
                (frame.parity == ~reduce(operator.xor, frame.data)) &
                (frame.stop == 1)
            )
        def prepare_frame(frame, data):
            return [
                NextValue(frame.start,  0),
                NextValue(frame.data,   data),
                NextValue(frame.parity, ~reduce(operator.xor, data)),
                NextValue(frame.stop,   1),
            ]

        self.submodules.rx_fsm = FSM(reset_state="IDLE")
        self.submodules.tx_fsm = FSM(reset_state="IDLE")

        rx_frame = Record(frame_layout)
        rx_bitno = Signal(max=11)

        self.rx_fsm.act("IDLE",
            If(self.tx_fsm.ongoing("IDLE") & ~bus.clock_i,
                NextState("BIT")
            )
        )
        self.rx_fsm.act("BIT",
            If(bus.sample,
                NextValue(rx_frame.raw_bits(), Cat(rx_frame.raw_bits()[1:], bus.data_i)),
                NextValue(rx_bitno, rx_bitno + 1),
                If(rx_bitno == 10,
                    NextValue(rx_bitno, 0),
                    NextState("FRAME")
                )
            )
        )
        self.rx_fsm.act("FRAME",
            If(check_frame(rx_frame),
                If(in_fifo.writable,
                    in_fifo.din.eq(rx_frame.data),
                    in_fifo.we.eq(1),
                    NextState("IDLE-WAIT")
                )
            ).Else(
                NextState("IDLE-WAIT")
            )
        )
        self.rx_fsm.act("IDLE-WAIT",
            If(bus.clock_i,
                NextState("IDLE")
            )
        )

        tx_timer = Signal(max=request_cyc)

        tx_frame = Record(frame_layout)
        tx_bitno = Signal(max=11)

        self.tx_fsm.act("IDLE",
            If(self.rx_fsm.ongoing("IDLE") & bus.clock_i,
                If(out_fifo.readable,
                    NextValue(bus.clock_o, 0),
                    NextValue(tx_timer, request_cyc - 1),
                    NextState("REQUEST")
                )
            )
        )
        self.tx_fsm.act("REQUEST",
            NextValue(tx_timer, tx_timer - 1),
            If(tx_timer == 0,
                NextValue(bus.clock_o, 1),
                NextValue(bus.data_o,  0),
                prepare_frame(tx_frame, out_fifo.dout),
                NextState("BIT")
            )
        )
        self.tx_fsm.act("BIT",
            If(bus.setup,
                NextValue(bus.data_o, tx_frame.raw_bits()[0]),
                NextValue(tx_frame.raw_bits(), tx_frame.raw_bits()[1:]),
                NextValue(tx_bitno, tx_bitno + 1),
                If(tx_bitno == 10,
                    NextValue(tx_bitno, 0),
                    NextState("ACK")
                )
            )
        )
        self.tx_fsm.act("ACK",
            If(bus.sample,
                If(~bus.data_i,
                    out_fifo.re.eq(1),
                ),
                NextState("IDLE-WAIT")
            )
        )
        self.tx_fsm.act("IDLE-WAIT",
            If(bus.clock_i,
                NextState("IDLE")
            )
        )


class PS2HostApplet(GlasgowApplet, name="ps2-host"):
    logger = logging.getLogger(__name__)
    help = "communicate via IBM PS/2 peripherals"
    description = """
    Communicate via IBM PS/2 protocol with peripherals such as keyboards and mice.

    A reset pin is optionally supported. This pin will be asserted when the applet is reset, which
    prevents desynchronization. The reset line is not a part of IBM PS/2, but is present on some
    peripherals, such as IBM TrackPoint™ devices.
    """
    required_revision = "C0"
    preview = True

    __pins = ("clock", "data")

    @classmethod
    def add_build_arguments(cls, parser, access):
        super().add_build_arguments(parser, access)

        for pin in cls.__pins:
            access.add_pin_argument(parser, pin, required=True, default=True)

    def build(self, target, args):
        self.mux_interface = iface = target.multiplexer.claim_interface(self, args)
        iface.add_subtarget(PS2HostSubtarget(
            pads=iface.get_pads(args, pins=self.__pins),
            in_fifo=iface.get_in_fifo(),
            out_fifo=iface.get_out_fifo(),
            request_cyc=int(target.sys_clk_freq * 60e-6),
        ))

    async def run(self, device, args):
        return await device.demultiplexer.claim_interface(self, self.mux_interface, args,
            pull_high={args.pin_clock, args.pin_data})

    async def interact(self, device, args, iface):
        await iface.write(b"\xf4")
        print(list(await iface.read(1)))
        await iface.write(b"\xea")
        print(list(await iface.read(4)))
        await iface.write(b"\xeb")
        print(list(await iface.read(1)))
        # while True:
        #     await iface.write(b"\xea")
        #     print(list(await iface.read(4)))
        while True:
            xx = await iface.read()
            print(xx.hex(), end=" ", flush=True)

# -------------------------------------------------------------------------------------------------

class PS2HostAppletTestCase(GlasgowAppletTestCase, applet=PS2HostApplet):
    @synthesis_test
    def test_build(self):
        self.assertBuilds()
