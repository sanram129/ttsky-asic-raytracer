# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles


@cocotb.test()
async def test_project(dut):
    dut._log.info("Start")

    # Set the clock period to 10 us (100 KHz)
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    # Reset
    dut._log.info("Reset")
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    dut._log.info("Smoke test: wrapper register write/read")

    def pack_uio(addr: int, wr: int = 0, rd: int = 0) -> int:
        return (addr & 0x3F) | ((wr & 1) << 6) | ((rd & 1) << 7)

    async def reg_write(addr: int, data: int):
        dut.ui_in.value = data & 0xFF
        dut.uio_in.value = pack_uio(addr, wr=1, rd=0)
        await ClockCycles(dut.clk, 1)
        dut.uio_in.value = pack_uio(addr, wr=0, rd=0)
        await ClockCycles(dut.clk, 1)

    async def reg_read(addr: int) -> int:
        # Latch read address
        dut.uio_in.value = pack_uio(addr, wr=0, rd=1)
        await ClockCycles(dut.clk, 1)
        dut.uio_in.value = pack_uio(addr, wr=0, rd=0)
        await ClockCycles(dut.clk, 1)
        return int(dut.uo_out.value)

    # Write and read back a job register byte
    await reg_write(0x10, 0x2A)
    val = await reg_read(0x10)
    assert val == 0x2A, f"Expected 0x2A, got 0x{val:02X}"

    # Read status register (should be well-defined)
    status = await reg_read(0x30)
    dut._log.info(f"Status0 (0x30) = 0x{status:02X}")

