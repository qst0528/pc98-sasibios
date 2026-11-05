#!/usr/bin/env python3
"""SASI BIOS の試験（調査側が用意したもの）。実装側の sasibios/sasi.asm を Unicorn で、本体の模型（bios/test/host.asm）と
SASI / SCSI のターゲットの模型の上で動かす。バス（I/O 80h、82h）、8237 のチャネル 0、8259 の IR9（INT 11h）を Python で模す。

- 各試験は、仕様書（docs/sasibios_spec.md）に書いてある振る舞いだけを確かめる。docstring に節を書いてある。
- 試験が仕様書と合わない、または仕様書にない振る舞いを求めていると思ったら、試験を直さずに QUESTIONS.md に書く。
- 使い方: `python3 bios/test/sasi/test_sasi.py [試験の名前 ...]`（nasm と Python の unicorn が要る）。
  BIOS のソースは sasibios/sasi.asm。別の場所なら環境変数 SASI_ASM で指す。ROM は D7000h（ROM_SEG）に置く。
"""

import os
import struct
import subprocess
import sys
import tempfile

from unicorn import Uc, UcError, UC_ARCH_X86, UC_MODE_16, UC_HOOK_INSN, UC_HOOK_INTR, UC_HOOK_CODE
from unicorn.x86_const import (
    UC_X86_INS_IN, UC_X86_INS_OUT,
    UC_X86_REG_AX, UC_X86_REG_BX, UC_X86_REG_CX, UC_X86_REG_DX,
    UC_X86_REG_SI, UC_X86_REG_DI, UC_X86_REG_BP, UC_X86_REG_SP,
    UC_X86_REG_CS, UC_X86_REG_DS, UC_X86_REG_ES, UC_X86_REG_SS,
    UC_X86_REG_EFLAGS, UC_X86_REG_IP,
)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
HOST_ASM = os.path.join(ROOT, "bios", "test", "host.asm")
SASI_ASM = os.environ.get("SASI_ASM", os.path.join(ROOT, "sasibios", "sasi.asm"))
HOST_SEG = 0x1000
ROM_SEG = 0xD700

S_REQ, S_ACK, S_BSY, S_MSG, S_CXD, S_IXO, S_INT = 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x01


def assemble(src, out):
    subprocess.run(["nasm", "-f", "bin", "-w+all", "-o", out, src], check=True)
    with open(out, "rb") as f:
        return f.read()


class Disk:
    def __init__(self, blocks, bs):
        self.blocks, self.bs = blocks, bs
        self.data = {}

    def read(self, lba):
        return self.data.get(lba) or (struct.pack("<I", lba) * (self.bs // 4))

    def write(self, lba, b):
        self.data[lba] = bytes(b)


class SasiController:
    """UX-31 のコントローラの模型（仕様書 2.5、付録 C の「UX-31 のコントローラの振る舞い」）。LUN ごとのドライブ。
    命令は 6 バイト（C2h は 6 バイトのあとに 10 バイトのデータ）。表にない命令は 6 バイトで断る（センス 20h）"""

    def __init__(self, luns):
        self.luns = luns            # lun -> Disk（ブロック数は 256 バイトのセクタで数える）
        self.sense = {}
        self.bs = 256
        self.log = []
        self.corrected_at = None    # この LBA の読み出しで「訂正したデータエラー」

    def reset(self):
        pass

    def cmd_len(self, cdb):
        if len(cdb) >= 1 and cdb[0] == 0xC2:
            return 16
        return 6

    def execute(self, cdb, bus):
        op, lun = cdb[0], cdb[1] >> 5
        self.log.append(bytes(cdb))
        disk = self.luns.get(lun)
        lba = (cdb[1] & 0x1F) << 16 | cdb[2] << 8 | cdb[3]
        n = cdb[4] or 256

        def check(code, addr=None):
            if addr is None:
                self.sense[lun] = bytes((code, lun << 5, 0, 0))
            else:
                self.sense[lun] = bytes((code | 0x80, lun << 5 | addr >> 16 & 0x1F, addr >> 8 & 0xFF, addr & 0xFF))
            bus.to_status(0x02 + lun * 0x20)

        if op == 0x03:
            bus.data_in(self.sense.pop(lun, b"\0\0\0\0")[:4], lambda: bus.to_status(lun * 0x20))
            return
        if op == 0xC2:
            self.bs = 512 if cdb[14] else 256
            bus.to_status(lun * 0x20)
            return
        if op not in (0x00, 0x01, 0x04, 0x05, 0x06, 0x08, 0x0A, 0x0B, 0x0E):
            check(0x20)
            return
        if disk is None:
            check(0x04)
            return
        if op in (0x08, 0x0A, 0x05):
            total = disk.blocks * 256 // self.bs
            if lba + n > total:
                check(0x21, lba)
                return
        if op == 0x08:
            data = b"".join(self.block(disk, lba + i) for i in range(n))
            if self.corrected_at is not None and lba <= self.corrected_at < lba + n:
                k = self.corrected_at - lba + 1
                at = self.corrected_at
                self.corrected_at = None
                bus.data_in(data[:k * self.bs], lambda: check(0x18, at))
                return
            bus.data_in(data, lambda: bus.to_status(lun * 0x20))
        elif op == 0x0A:
            def done(b):
                for i in range(n):
                    self.put_block(disk, lba + i, b[i * self.bs:(i + 1) * self.bs])
                bus.to_status(lun * 0x20)
            bus.data_out(n * self.bs, done)
        elif op == 0x0E:
            bus.data_out(4, lambda b: bus.to_status(lun * 0x20))
        else:
            bus.to_status(lun * 0x20)

    def block(self, disk, lba):
        k = self.bs // 256
        return b"".join(disk.read(lba * k + i) for i in range(k))

    def put_block(self, disk, lba, b):
        k = self.bs // 256
        for i in range(k):
            disk.write(lba * k + i, b[i * 256:(i + 1) * 256])


class ScsiTarget:
    """SCSI のターゲットの模型（LUN 0 だけ。仕様書 付録 C）。電源投入とリセットのあと 1 回 UNIT ATTENTION"""

    def __init__(self, disk):
        self.disk = disk
        self.ua = True
        self.sense = None
        self.log = []
        self.sel_bytes = []

    def reset(self):
        self.ua = True

    def cmd_len(self, cdb):
        if not cdb:
            return 6
        return {0: 6, 1: 10, 2: 10, 5: 12}.get(cdb[0] >> 5, 6)

    def execute(self, cdb, bus):
        op = cdb[0]
        self.log.append(bytes(cdb))
        lun = cdb[1] >> 5

        def check(key, asc):
            self.sense = bytes((0x70, 0, key, 0, 0, 0, 0, 10, 0, 0, 0, 0, asc, 0, 0, 0, 0, 0))
            bus.to_status(0x02)

        if op == 0x03:
            s = self.sense or bytes(18)
            if lun:
                s = bytes((0x70, 0, 5, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0x25, 0, 0, 0, 0, 0))
            self.sense = None
            bus.data_in(s[:cdb[4] or 4], lambda: bus.to_status(0))
            return
        if lun:
            check(5, 0x25)
            return
        if self.ua:
            self.ua = False
            check(6, 0x29)
            return
        d = self.disk
        if op == 0x00:
            bus.to_status(0)
        elif op == 0x25:
            bus.data_in(struct.pack(">II", d.blocks - 1, d.bs), lambda: bus.to_status(0))
        elif op in (0x28, 0x2A):
            lba = struct.unpack(">I", bytes(cdb[2:6]))[0]
            n = cdb[7] << 8 | cdb[8]
            if lba + n > d.blocks:
                check(5, 0x21)
                return
            if op == 0x28:
                bus.data_in(b"".join(d.read(lba + i) for i in range(n)), lambda: bus.to_status(0))
            else:
                def done(b):
                    for i in range(n):
                        d.write(lba + i, b[i * d.bs:(i + 1) * d.bs])
                    bus.to_status(0)
                bus.data_out(n * d.bs, done)
        else:
            check(5, 0x20)


class Bus:
    def __init__(self, m, targets, dip=0xFF):
        self.m = m
        self.targets = targets      # id -> target
        self.dip = dip
        self.ctrl = 0x40
        self.latch = 0
        self.phase = "free"
        self.cur = None
        self.cdb = []
        self.buf = b""
        self.pos = 0
        self.on_done = None
        self.status = 0
        self.irq = False
        self.sel_log = []
        self.cmd_after_status = 0   # 書いてはいけないところで 80h に書いた回数

    def state(self):
        if self.ctrl & 0x40 == 0:
            return self.dip
        p = self.phase
        v = 0
        if p != "free":
            v |= S_BSY
        if p in ("cmd", "din", "dout", "status", "msg"):
            v |= S_REQ
        if p in ("cmd", "status", "msg"):
            v |= S_CXD
        if p in ("din", "status", "msg"):
            v |= S_IXO
        if p == "msg":
            v |= S_MSG
        if self.irq:
            v |= S_INT
        return v

    def update_irq(self):
        new = bool(self.ctrl & 0x01) and self.phase == "status"
        if new and not self.irq:
            self.m.raise_irq()
        self.irq = new

    def write_ctrl(self, v):
        old = self.ctrl
        self.ctrl = v
        if v & 0x08:
            self.phase = "free"
            for t in self.targets.values():
                t.reset()
        elif v & 0x20 and not old & 0x20 and self.phase == "free":
            self.sel_log.append(self.latch)
            for i, t in self.targets.items():
                if self.latch & (1 << i):
                    self.cur = t
                    self.phase = "sel"
                    break
        elif not v & 0x20 and self.phase == "sel":
            self.phase = "cmd"
            self.cdb = []
        self.update_irq()

    def write_data(self, v):
        self.latch = v
        if self.phase == "cmd":
            self.cdb.append(v)
            if len(self.cdb) >= self.cur.cmd_len(self.cdb):
                self.cur.execute(self.cdb, self)
        elif self.phase == "dout":
            self.buf += bytes((v,))
            self.check_out()
        elif self.phase in ("status", "msg", "din"):
            self.cmd_after_status += 1

    def read_data(self):
        if self.phase == "din":
            v = self.buf[self.pos]
            self.pos += 1
            if self.pos == len(self.buf):
                self.on_done()
            return v
        if self.phase == "status":
            self.phase = "msg"
            self.update_irq()
            return self.status
        if self.phase == "msg":
            self.phase = "free"
            self.cur = None
            return 0
        return 0xFF

    def data_in(self, data, done):
        self.buf, self.pos, self.on_done = data, 0, done
        self.phase = "din"
        if not data:
            done()
            return
        if self.ctrl & 0x02:
            self.m.dma_in(self)

    def data_out(self, n, done):
        self.buf, self.need, self.on_done = b"", n, done
        self.phase = "dout"
        if self.ctrl & 0x02:
            self.m.dma_out(self)

    def check_out(self):
        if len(self.buf) == self.need:
            self.on_done(self.buf)

    def to_status(self, status):
        self.status = status
        self.phase = "status"
        self.update_irq()


PAGE_PORTS = (0x27, 0x21, 0x23, 0x25)          # チャネル 0〜3 のページ（仕様書 2.3）
DMA_PORTS = {0x01, 0x03, 0x05, 0x07, 0x09, 0x0B, 0x0D, 0x0F, 0x15, 0x17, *PAGE_PORTS}


def ch_ports(c):
    """チャネル c だけが使う I/O（アドレス、カウント、ページ）。"""
    return {1 + 4 * c, 3 + 4 * c, PAGE_PORTS[c]}


class Machine:
    def __init__(self, rom, host, targets, dip=0xFF, sw2=0xFF, init=True, segs=(ROM_SEG,), hw_ch=0, bios484=0x00):
        self.uc = uc = Uc(UC_ARCH_X86, UC_MODE_16)
        uc.mem_map(0, 0x110000)
        uc.mem_write(0x484, bytes((bios484,)))   # 本体が起動時に作る 0000:0484h（仕様書 2.3、付録 B）
        uc.mem_write(HOST_SEG * 16, host)
        for seg in segs:                        # 同じ ROM を複数の番地に置く試験のため（仕様書 1.2）
            uc.mem_write(seg * 16, rom)
        self.bus = Bus(self, targets, dip)
        self.sw2 = sw2
        # 8237 のチャネル 0〜3。ターゲットの DRQ は hw_ch のチャネルにだけつながる（専用スロットで SW3-3 が ON の機種などを真似る）
        self.dmas = [{"addr": 0, "count": 0, "page": 0, "mode": 0, "mask": 1} for _ in range(4)]
        self.ff = 0
        self.hw_ch = hw_ch
        self.dma_out_log = []                   # DMA の I/O（01h〜0Fh、21h〜27h、15h、17h）への書き込み
        self.pending = False
        uc.hook_add(UC_HOOK_INSN, self.hook_in, None, 1, 0, UC_X86_INS_IN)
        uc.hook_add(UC_HOOK_INSN, self.hook_out, None, 1, 0, UC_X86_INS_OUT)
        uc.hook_add(UC_HOOK_INTR, self.hook_intr)
        uc.hook_add(UC_HOOK_CODE, self.hook_code)
        self.run(0x0000)
        if init:
            for entry in (0x0C, 0x0F, 0x12):
                self.run(0x0003, ax=entry)

    # ------------------------------------------------ 割り込み
    def raise_irq(self):
        self.pending = True

    def push_frame(self, ip, cs):
        uc = self.uc
        sp = uc.reg_read(UC_X86_REG_SP)
        ss = uc.reg_read(UC_X86_REG_SS)
        flags = uc.reg_read(UC_X86_REG_EFLAGS) & 0xFFFF
        for v in (flags, cs, ip):
            sp = (sp - 2) & 0xFFFF
            uc.mem_write(ss * 16 + sp, struct.pack("<H", v))
        uc.reg_write(UC_X86_REG_SP, sp)
        uc.reg_write(UC_X86_REG_EFLAGS, flags & ~0x0300)

    def hook_code(self, uc, addr, size, _):
        if self.pending and uc.reg_read(UC_X86_REG_EFLAGS) & 0x200:
            self.pending = False
            self.push_frame(uc.reg_read(UC_X86_REG_IP), uc.reg_read(UC_X86_REG_CS))
            ip, cs = struct.unpack("<HH", self.mem(0x11 * 4, 4))
            uc.reg_write(UC_X86_REG_CS, cs)
            uc.reg_write(UC_X86_REG_IP, ip)

    def hook_intr(self, uc, n, _):
        self.push_frame(uc.reg_read(UC_X86_REG_IP), uc.reg_read(UC_X86_REG_CS))
        ip, cs = struct.unpack("<HH", self.mem(n * 4, 4))
        uc.reg_write(UC_X86_REG_CS, cs)
        uc.reg_write(UC_X86_REG_IP, ip)

    # ------------------------------------------------ DMA
    @property
    def dma(self):
        return self.dmas[self.hw_ch]

    def dma_addr(self):
        return self.dma["page"] << 16 | self.dma["addr"]

    def dma_in(self, bus):
        d = self.dma
        if d["mask"]:                           # 要求が届くチャネルが準備されていない: 転送は起きず、ターゲットは止まったまま
            return
        while bus.phase == "din" and d["count"] != 0xFFFF + 1:
            v = bus.read_data_dma()
            if d["mode"] & 0x0C == 0x04:
                self.uc.mem_write(self.dma_addr(), bytes((v,)))
            d["addr"] = (d["addr"] + 1) & 0xFFFF
            d["count"] = (d["count"] - 1) & 0xFFFF
            if d["count"] == 0xFFFF:
                break

    def dma_out(self, bus):
        d = self.dma
        if d["mask"]:
            return
        while bus.phase == "dout":
            v = self.mem(self.dma_addr(), 1)[0]
            d["addr"] = (d["addr"] + 1) & 0xFFFF
            d["count"] = (d["count"] - 1) & 0xFFFF
            bus.write_data(v)
            if d["count"] == 0xFFFF:
                break

    # ------------------------------------------------ I/O
    def hook_in(self, uc, port, size, _):
        if port == 0x82:
            return self.bus.state()
        if port == 0x80:
            return self.bus.read_data()
        if port in (0x03, 0x07, 0x0B, 0x0F):
            d = self.dmas[(port - 3) // 4]
            v = d["count"] >> (8 * self.ff) & 0xFF
            self.ff ^= 1
            return v
        if port == 0x08:
            return 0
        if port == 0x0A:
            return 0xFF
        if port == 0x31:
            return self.sw2
        return 0xFF

    def hook_out(self, uc, port, size, value, _):
        if port in DMA_PORTS:
            self.dma_out_log.append((port, value))
        if port == 0x82:
            self.bus.write_ctrl(value)
        elif port == 0x80:
            self.bus.write_data(value)
        elif port == 0x19:
            self.ff = 0
        elif port in (0x01, 0x05, 0x09, 0x0D):
            d = self.dmas[(port - 1) // 4]
            d["addr"] = d["addr"] & (0xFF00 >> 8 * self.ff) | value << 8 * self.ff
            self.ff ^= 1
        elif port in (0x03, 0x07, 0x0B, 0x0F):
            d = self.dmas[(port - 3) // 4]
            d["count"] = d["count"] & (0xFF00 >> 8 * self.ff) | value << 8 * self.ff
            self.ff ^= 1
        elif port in PAGE_PORTS:
            self.dmas[PAGE_PORTS.index(port)]["page"] = value
        elif port == 0x17:
            self.dmas[value & 3]["mode"] = value
        elif port == 0x15:
            self.dmas[value & 3]["mask"] = value & 0x04

    # ------------------------------------------------ 実行
    def run(self, ip, until=None, **regs):
        uc = self.uc
        uc.reg_write(UC_X86_REG_SS, 0)
        uc.reg_write(UC_X86_REG_SP, 0x7C00)
        uc.reg_write(UC_X86_REG_DS, 0)
        uc.reg_write(UC_X86_REG_ES, regs.pop("es", 0))
        uc.reg_write(UC_X86_REG_EFLAGS, 0x0202)
        names = {"ax": UC_X86_REG_AX, "bx": UC_X86_REG_BX, "cx": UC_X86_REG_CX,
                 "dx": UC_X86_REG_DX, "si": UC_X86_REG_SI, "di": UC_X86_REG_DI,
                 "bp": UC_X86_REG_BP}
        for k, v in regs.items():
            uc.reg_write(names[k], v)
        uc.reg_write(UC_X86_REG_CS, HOST_SEG)
        uc.emu_start(HOST_SEG * 16 + ip, until or 0xFFFFFFFF, count=getattr(self, "max_count", 30_000_000))
        return uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP)

    def int1b(self, **regs):
        self.run(0x0009, **regs)
        r = {k: self.uc.reg_read(v) for k, v in (
            ("ax", UC_X86_REG_AX), ("bx", UC_X86_REG_BX), ("cx", UC_X86_REG_CX),
            ("dx", UC_X86_REG_DX), ("bp", UC_X86_REG_BP), ("es", UC_X86_REG_ES),
            ("sp", UC_X86_REG_SP))}
        r["cf"] = self.uc.reg_read(UC_X86_REG_EFLAGS) & 1
        r["ah"] = r["ax"] >> 8
        return r

    def mem(self, addr, n):
        return bytes(self.uc.mem_read(addr, n))

    def byte(self, addr):
        return self.mem(addr, 1)[0]


def _read_data_dma(self):
    v = self.buf[self.pos]
    self.pos += 1
    if self.pos == len(self.buf):
        self.on_done()
    return v


Bus.read_data_dma = _read_data_dma


def build():
    tmp = tempfile.mkdtemp()
    rom = assemble(SASI_ASM, os.path.join(tmp, "sasi.rom"))
    host = assemble(HOST_ASM, os.path.join(tmp, "host.bin"))
    return rom, host


def dip(code0, code1, s512_0=0, s512_1=0):
    return s512_0 << 7 | s512_1 << 6 | code0 << 3 | code1


# ---------------------------------------------------------------- 試験
def test_ux31_one_drive(rom, host):
    """UX-31 型のコントローラ（6 バイト型）1 台。仕様書 4.1、4.2、4.3、2.4、2.5、6.3（センス、新センス）、付録 C（調べ方、ユニットの宛先）。"""
    ctl = SasiController({0: Disk(615 * 8 * 33, 256)})
    m = Machine(rom, host, {0: ctl}, dip=dip(6, 7))
    assert m.byte(0x4D7) == 0xE9
    assert m.byte(0x4B0) == 0xD7 and m.byte(0x4B8) == 0xD7
    assert m.byte(0x55D) & 3 == 1, hex(m.byte(0x55D))
    assert m.byte(0x480) & 0x80
    # READ CAPACITY は 6 バイトで断られ、7 バイト目以降は送られていない
    assert any(c[0] == 0x25 and len(c) == 6 for c in ctl.log), ctl.log[:5]
    assert m.bus.cmd_after_status == 0
    # ID 0（+ ホストの ID 7）。81h の 2 番目の候補として ID 1 も 1 回だけ試す
    assert set(m.bus.sel_log) == {0x81, 0x82} and m.bus.sel_log.count(0x82) == 1, m.bus.sel_log
    assert any(c[0] == 0xC2 and len(c) == 16 for c in ctl.log)
    r = m.int1b(ax=0x8480)
    assert r["cf"] == 0 and r["ah"] == 0x07, r
    assert r["bx"] == 256 and r["cx"] == 614 and r["dx"] == 0x0821, r
    r = m.int1b(ax=0x0480)
    assert r["ah"] == 0x05, r
    # 81h: なし（センスは コントローラの答え: LUN 1 にドライブなし → 60h）
    r = m.int1b(ax=0x0481)
    assert r["cf"] == 1 and r["ah"] == 0x60, r
    assert r["sp"] == 0x7C00


def test_ux31_read_write(rom, host):
    """読み書き、ベリファイ、64 KB の境界、範囲外。仕様書 3.3、6.3、6.4、2.3、4.5。"""
    disk = Disk(615 * 8 * 33, 256)
    ctl = SasiController({0: disk})
    m = Machine(rom, host, {0: ctl}, dip=dip(6, 7))
    # リニア 3 セクタ
    r = m.int1b(ax=0x0600, bx=3 * 256, cx=1000, dx=0, es=0x2000, bp=0x0010)
    assert r["cf"] == 0 and r["ah"] == 0, r
    assert m.mem(0x20010, 768) == b"".join(disk.read(1000 + i) for i in range(3))
    assert ctl.log[-1][:6] == bytes((0x08, 0x00, 0x03, 0xE8, 3, 0))
    # CHS（8 ヘッド、33 セクタ）
    lba = (100 * 8 + 5) * 33 + 7
    r = m.int1b(ax=0x0680, bx=256, cx=100, dx=0x0507, es=0x2000, bp=0)
    assert r["cf"] == 0 and m.mem(0x20000, 256) == disk.read(lba), r
    # 書き込み
    payload = os.urandom(512)
    m.uc.mem_write(0x30005, payload)
    r = m.int1b(ax=0x0500, bx=512, cx=77, dx=0, es=0x3000, bp=0x0005)
    assert r["cf"] == 0 and disk.read(77) + disk.read(78) == payload, r
    # 64 KB の境界
    r = m.int1b(ax=0x0600, bx=1024, cx=0, dx=0, es=0x1FFF, bp=0x000F)
    assert r["cf"] == 1 and r["ah"] == 0x20, r
    # 範囲外
    r = m.int1b(ax=0x0600, bx=512, cx=0xFFFF, dx=0x1F, es=0x2000, bp=0)
    assert r["cf"] == 1 and r["ah"] == 0x38, r
    # ベリファイ
    r = m.int1b(ax=0x0100, bx=512, cx=5, dx=0, es=0, bp=0)
    assert r["cf"] == 0 and ctl.log[-1][0] == 0x08, r


def test_ux31_corrected(rom, host):
    """訂正したデータエラーのあとの読み直し。仕様書 6.4 の手順 5。"""
    disk = Disk(615 * 8 * 33, 256)
    ctl = SasiController({0: disk})
    m = Machine(rom, host, {0: ctl}, dip=dip(6, 7))
    ctl.corrected_at = 12
    r = m.int1b(ax=0x0600, bx=10 * 256, cx=10, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 0 and r["ah"] == 0x08, r
    assert m.mem(0x20000, 2560) == b"".join(disk.read(10 + i) for i in range(10))
    assert ctl.log[-1][:5] == bytes((0x08, 0, 0, 13, 7)), ctl.log[-1]


def test_ux31_two_drives_thin(rom, host):
    """2 台目（LUN 1）と、厚型 / 薄型の CHS の計算。仕様書 3.1、3.4、6.3。"""
    ctl = SasiController({0: Disk(615 * 8 * 33, 256), 1: Disk(615 * 4 * 33, 256)})
    m = Machine(rom, host, {0: ctl}, dip=dip(6, 4))
    assert m.byte(0x55D) & 3 == 3
    r = m.int1b(ax=0x8481)
    assert r["ah"] == 0x03 and r["cx"] == 614 and r["dx"] == 0x0421, r
    # 厚型: CHS の計算は 8 ヘッド。薄型（8Eh）なら 4 ヘッド
    r = m.int1b(ax=0x0681, bx=256, cx=1, dx=0x0000, es=0x2000, bp=0)
    assert ctl.log[-1][1:4] == bytes((0x20, 0x01, 0x08)), ctl.log[-1]     # (1*8)*33 = 264
    m.int1b(ax=0x8E81)
    r = m.int1b(ax=0x0681, bx=256, cx=1, dx=0x0000, es=0x2000, bp=0)
    assert ctl.log[-1][1:4] == bytes((0x20, 0x00, 0x84)), ctl.log[-1]     # (1*4)*33 = 132


def test_ux31_512_closed(rom, host):
    """512 バイトのドライブの閉鎖と 8Eh。仕様書 3.4、6.4 の手順 1、6.3。"""
    ctl = SasiController({0: Disk(615 * 8 * 33, 256)})
    m = Machine(rom, host, {0: ctl}, dip=dip(6, 7, s512_0=1))
    m.int1b(ax=0x0E80)
    r = m.int1b(ax=0x0680, bx=512, cx=0, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 1 and r["ah"] == 0x40, r
    m.int1b(ax=0x8E80)
    r = m.int1b(ax=0x0680, bx=512, cx=0, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 0, r
    r = m.int1b(ax=0x8480, bx=0)
    assert r["bx"] == 512 and r["dx"] & 0xFF == 17, r


def test_no_controller(rom, host):
    """コントローラがない（82h が FFh）。仕様書 2.1（bit5）、4.3。"""
    class Floating(Bus):
        def state(self):
            return 0xFF
    m = Machine(rom, host, {}, init=False)
    m.bus.__class__ = Floating
    for entry in (0x0C, 0x0F, 0x12):
        m.run(0x0003, ax=entry)
    assert m.byte(0x55D) & 3 == 0


def test_no_target(rom, host):
    """ターゲットがいなくても、初期化 1 で登録はする。仕様書 4.2、付録 C。"""
    m = Machine(rom, host, {})
    assert m.byte(0x55D) & 3 == 0
    assert m.byte(0x4B0) == 0xD7


def test_mirror_d5_d7(rom, host):
    """同じ ROM が 2 か所に見えるとき。仕様書 1.2、4.2、4.3。"""
    ctl = SasiController({0: Disk(615 * 8 * 33, 256)})
    m = Machine(rom, host, {0: ctl}, dip=dip(6, 7), segs=(0xD500, 0xD700))
    assert m.byte(0x4D5) == 0xE9 and m.byte(0x4D7) == 0xE9
    assert m.byte(0x4B0) == 0xD5 and m.byte(0x4B8) == 0xD5
    assert m.mem(0x11 * 4 + 2, 2) == bytes((0x00, 0xD5))
    assert m.byte(0x55D) & 3 == 1, hex(m.byte(0x55D))
    probes = sum(1 for c in ctl.log if c[0] == 0x25 and c[1] == 0x00)
    assert probes == 1, ctl.log                 # LUN 0 の確かめは D5000h の 1 回だけ
    r = m.int1b(ax=0x8480)
    assert r["cf"] == 0 and r["cx"] == 614, r


def test_scsi_512(rom, host):
    """10 バイト型、512 バイトのブロック。仕様書 付録 C（調べ方、ジオメトリ、10 バイト型の機能）、6.4。"""
    disk = Disk(1_000_000, 512)                 # 約 488 MiB
    t = ScsiTarget(disk)
    m = Machine(rom, host, {0: t})
    assert m.byte(0x55D) & 3 == 1, hex(m.byte(0x55D))
    r = m.int1b(ax=0x8480)
    assert r["cf"] == 0 and r["ah"] == 0x0F, r
    assert r["bx"] == 512 and r["dx"] == 0x0811 and r["cx"] == 1_000_000 // 136 - 1, r
    # 見分けたあとは閉鎖（仕様書 0.11 版の 3.4、付録 C の手順 4）。読み書きは命令を出さずに AH = 40h
    assert m.byte(0x481) & 1, hex(m.byte(0x481))
    n = len(t.log)
    r = m.int1b(ax=0x0600, bx=256, cx=0, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 1 and r["ah"] == 0x40 and len(t.log) == n, r
    r = m.int1b(ax=0x8E80)                      # 開放
    assert r["cf"] == 0 and m.byte(0x481) & 1 == 0
    # リニアの 32 ビット
    payload = os.urandom(1024)
    m.uc.mem_write(0x30000, payload)
    target = 900_000
    r = m.int1b(ax=0x0500, bx=1024, cx=target & 0xFFFF, dx=target >> 16, es=0x3000, bp=0)
    assert r["cf"] == 0 and disk.read(target) + disk.read(target + 1) == payload, r
    r = m.int1b(ax=0x0600, bx=1024, cx=target & 0xFFFF, dx=target >> 16, es=0x4000, bp=0)
    assert r["cf"] == 0 and m.mem(0x40000, 1024) == payload
    assert t.log[-1][:9] == bytes((0x28, 0, 0, 0x0D, 0xBB, 0xA0, 0, 0, 2)), t.log[-1]
    # CHS（8 × 17）
    c, h, s = 5000, 3, 9
    lba = (c * 8 + h) * 17 + s
    r = m.int1b(ax=0x0680, bx=512, cx=c, dx=h << 8 | s, es=0x2000, bp=0)
    assert r["cf"] == 0 and m.mem(0x20000, 512) == disk.read(lba)
    # 64 KB（BX = 0）
    r = m.int1b(ax=0x0600, bx=0, cx=0, dx=0, es=0x5000, bp=0)
    assert r["cf"] == 0 and t.log[-1][7:9] == bytes((0, 128))
    # 範囲外
    r = m.int1b(ax=0x0600, bx=512, cx=0xFFFF, dx=0x00FF, es=0x2000, bp=0)
    assert r["cf"] == 1 and r["ah"] == 0x38, r
    # シーク、リキャリブレート、リトラクト、モード設定は何もせず正常
    for ax in (0x0080, 0x0780, 0x0F80, 0x8E80, 0x0D80, 0x8D80):
        r = m.int1b(ax=ax)
        assert r["cf"] == 0 and r["ah"] == 0, (hex(ax), r)
    assert m.bus.sel_log[-1] == 0x81
    # 0Eh で閉鎖、8Eh で開放（0.11 版）
    r = m.int1b(ax=0x0E80)
    assert r["cf"] == 0 and r["ah"] == 0 and m.byte(0x481) & 1
    r = m.int1b(ax=0x8E80)
    assert m.byte(0x481) & 1 == 0


def test_scsi_big(rom, host):
    """10 バイト型のジオメトリの境目。仕様書 付録 C。"""
    disk = Disk(16 * 63 * 20000 + 5, 512)      # 約 9.6 GiB
    m = Machine(rom, host, {0: ScsiTarget(disk)})
    r = m.int1b(ax=0x8480)
    assert r["ah"] == 0x0F and r["dx"] == 0x103F and r["cx"] == 19999, r
    disk = Disk(65535 * 8 * 17, 512)
    m = Machine(rom, host, {0: ScsiTarget(disk)})
    r = m.int1b(ax=0x8480)
    assert r["dx"] == 0x0811 and r["cx"] == 65534, r
    disk = Disk(65535 * 8 * 17 + 1, 512)
    m = Machine(rom, host, {0: ScsiTarget(disk)})
    r = m.int1b(ax=0x8480)
    assert r["dx"] == 0x103F, r


def test_scsi_256_small(rom, host):
    """10 バイト型、256 バイトのブロックで小さい容量。仕様書 付録 C（256 バイトの型、どの型にも当たらないユニット）。"""
    for blocks, nsense, cyls, heads in ((614 * 8 * 33, 0x07, 614, 8), (615 * 8 * 33, 0x07, 615, 8),
                                        (310 * 8 * 33, 0x03, 310, 8), (310 * 4 * 33, 0x01, 310, 4),
                                        (153 * 4 * 33, 0x00, 153, 4)):
        m = Machine(rom, host, {0: ScsiTarget(Disk(blocks, 256))})
        r = m.int1b(ax=0x8480)
        assert r["ah"] == nsense and r["cx"] == cyls - 1 and r["dx"] >> 8 == heads and r["bx"] == 256, (blocks, r)
    # ブロック長 256 は 615 × 8 × 33 まで。超えるユニットは使わない（仕様書 0.13 版）
    m = Machine(rom, host, {0: ScsiTarget(Disk(615 * 8 * 33 + 1, 256))})
    assert m.byte(0x55D) & 1 == 0, hex(m.byte(0x55D))
    m = Machine(rom, host, {0: ScsiTarget(Disk(65535 * 8 * 33, 256))})
    assert m.byte(0x55D) & 1 == 0, hex(m.byte(0x55D))
    m = Machine(rom, host, {0: ScsiTarget(Disk(1000, 256))})
    assert m.byte(0x55D) & 1 == 0


class StuckTarget(ScsiTarget):
    """READ(10) で、求められたより 1 ブロック多いデータを出そうとして、データフェーズのまま止まるターゲット（仕様書 0.12 版の試験用）。"""

    stuck = True

    def execute(self, cdb, bus):
        if self.stuck and cdb[0] == 0x28 and not self.ua:
            self.log.append(bytes(cdb))
            n = (cdb[7] << 8 | cdb[8]) + 1
            bus.data_in(bytes(n * self.disk.bs), lambda: bus.to_status(0))
            return
        super().execute(cdb, bus)


def test_completion_timeout(rom, host):
    """完了の割り込みの待ちの時間切れ。仕様書 2.4 の手順 4、7、4.6（0.12 版）。"""
    disk = Disk(200_000, 512)
    t = StuckTarget(disk)
    m = Machine(rom, host, {0: t})
    m.max_count = 200_000_000                   # 約 10 秒の待ちを最後まで回す
    m.int1b(ax=0x8E80)
    r = m.int1b(ax=0x0600, bx=512, cx=0, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 1 and r["ah"] == 0x90, r
    assert m.dma["mask"], "DMA のチャネル 0 にマスクが掛かっていない（2.3、2.4 の手順 4）"
    assert m.bus.ctrl & 0x23 == 0, hex(m.bus.ctrl)   # SEL、INTE、DMAE を落とした（手順 7）
    # 03h（初期化）のあとは、ふつうに読める
    t.stuck = False
    r = m.int1b(ax=0x0300)
    assert r["cf"] == 0, r
    m.int1b(ax=0x8E80)
    r = m.int1b(ax=0x0600, bx=512, cx=0, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 0 and m.mem(0x20000, 512) == disk.read(0), r


def test_scsi_two_ids(rom, host):
    """81h を ID 1 の LUN 0 で受けるとき。仕様書 付録 C（ユニットの宛先）。"""
    d0, d1 = Disk(200_000, 512), Disk(300_000, 512)
    t0, t1 = ScsiTarget(d0), ScsiTarget(d1)
    m = Machine(rom, host, {0: t0, 1: t1})
    assert m.byte(0x55D) & 3 == 3
    assert m.byte(0x481) & 3 == 3               # どちらも 512 バイトなので閉鎖（0.11 版）
    m.int1b(ax=0x8E81)
    r = m.int1b(ax=0x8481)
    assert r["cx"] == 300_000 // 136 - 1, r
    r = m.int1b(ax=0x0601, bx=512, cx=1234, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 0 and m.mem(0x20000, 512) == d1.read(1234)
    assert m.bus.sel_log[-1] == 0x82


def test_boot_scsi(rom, host):
    """ブート（512 バイトのブロック）。仕様書 5。"""
    disk = Disk(200_000, 512)
    disk.write(0, b"\xEB\x0A\x90\x90IPL1" + bytes(504))
    m = Machine(rom, host, {0: ScsiTarget(disk)})
    end = m.run(0x0006, until=0x1FC00, ax=0x0A)
    assert end == 0x1FC00, hex(end)
    assert m.mem(0x1FC00, 512) == disk.read(0)
    assert m.byte(0x584) == 0x80
    assert m.uc.reg_read(UC_X86_REG_BX) == 0x400


def test_boot_sasi(rom, host):
    """ブート（256 バイトのセクタ）。仕様書 5。"""
    disk = Disk(615 * 8 * 33, 256)
    disk.write(0, b"\xEB\x0A\x90\x90IPL1" + bytes(248))
    m = Machine(rom, host, {0: SasiController({0: disk})}, dip=dip(6, 7))
    end = m.run(0x0006, until=0x1FE00, ax=0x0A)
    assert end == 0x1FE00, hex(end)
    assert m.mem(0x1FE00, 256) == disk.read(0)
    assert m.uc.reg_read(UC_X86_REG_BX) == 0x200


def test_boot_no_ipl(rom, host):
    """IPL がないときは起動しない。仕様書 5。"""
    disk = Disk(200_000, 512)
    m = Machine(rom, host, {0: ScsiTarget(disk)})
    end = m.run(0x0006, ax=0x0A)
    assert end != 0x1FC00


def test_sw2_6_on(rom, host):
    """I/O 31h の bit5 が 0（内蔵固定ディスクの切り離し）。仕様書 4.3。"""
    m = Machine(rom, host, {0: ScsiTarget(Disk(200_000, 512))}, sw2=0xDF)
    assert m.byte(0x55D) & 3 == 0


# ---------------------------------------------------------------- DMA のチャネル（仕様書 0.14 版の 2.3、付録 B）
CHANNEL_CASES = ((0x00, 0), (0x40, 0), (0xC0, 0), (0x8F, 0), (0x50, 1), (0x90, 2), (0xD0, 3), (0xFF, 3))


def check_channel_io(m, c):
    """DMA の I/O への書き込みが、チャネル c のものと共通のもの（15h、17h）だけで、15h と 17h の下位 2 ビットが c か。"""
    others = set().union(*(ch_ports(k) for k in range(4) if k != c))
    bad = [(hex(p), hex(v)) for p, v in m.dma_out_log if p in others]
    assert not bad, f"チャネル {c} でないチャネルの I/O に書いた: {bad[:4]}"
    sel = [(hex(p), hex(v)) for p, v in m.dma_out_log if p in (0x15, 0x17) and v & 3 != c]
    assert not sel, f"15h / 17h の下位 2 ビットが {c} でない: {sel[:4]}"


def test_dma_channel_select(rom, host):
    """0000:0484h の bit4 が 1 なら bit7〜6、0 ならチャネル 0。読み、書き、ベリファイ、出口のマスク。0484h は書き換えない。"""
    for v, c in CHANNEL_CASES:
        disk = Disk(615 * 8 * 33, 256)
        m = Machine(rom, host, {0: SasiController({0: disk})}, dip=dip(6, 7), hw_ch=c, bios484=v)
        m.dma_out_log.clear()
        r = m.int1b(ax=0x0600, bx=3 * 256, cx=1000, dx=0, es=0x2000, bp=0x0010)
        assert r["cf"] == 0 and r["ah"] == 0, (hex(v), r)
        assert m.mem(0x20010, 768) == b"".join(disk.read(1000 + i) for i in range(3)), hex(v)
        assert m.dma["mode"] == 0x44 + c, (hex(v), hex(m.dma["mode"]))
        assert m.dma["page"] == 0x02, (hex(v), m.dma["page"])
        payload = os.urandom(512)
        m.uc.mem_write(0x30005, payload)
        r = m.int1b(ax=0x0500, bx=512, cx=77, dx=0, es=0x3000, bp=0x0005)
        assert r["cf"] == 0 and disk.read(77) + disk.read(78) == payload, (hex(v), r)
        assert m.dma["mode"] == 0x48 + c, (hex(v), hex(m.dma["mode"]))
        r = m.int1b(ax=0x0100, bx=512, cx=5, dx=0, es=0, bp=0)
        assert r["cf"] == 0, (hex(v), r)
        assert m.dma["mode"] == 0x40 + c, (hex(v), hex(m.dma["mode"]))
        assert m.dma["mask"], f"0484h = {v:02X}h: 出口でチャネル {c} にマスクが掛かっていない"
        check_channel_io(m, c)
        assert m.byte(0x484) == v, f"0484h を書き換えた: {v:02X}h → {m.byte(0x484):02X}h"


def test_dma_channel_corrected(rom, host):
    """チャネル 1〜3 で、訂正したデータエラーのあとの読み直し（残りをチャネル c のカウントから求める）。仕様書 6.4 の手順 5。"""
    for v, c in ((0x50, 1), (0x90, 2), (0xD0, 3)):
        disk = Disk(615 * 8 * 33, 256)
        ctl = SasiController({0: disk})
        m = Machine(rom, host, {0: ctl}, dip=dip(6, 7), hw_ch=c, bios484=v)
        m.dma_out_log.clear()
        ctl.corrected_at = 12
        r = m.int1b(ax=0x0600, bx=10 * 256, cx=10, dx=0, es=0x2000, bp=0)
        assert r["cf"] == 0 and r["ah"] == 0x08, (c, r)
        assert m.mem(0x20000, 2560) == b"".join(disk.read(10 + i) for i in range(10)), c
        assert ctl.log[-1][:5] == bytes((0x08, 0, 0, 13, 7)), (c, ctl.log[-1])
        check_channel_io(m, c)


def test_dma_channel_timeout(rom, host):
    """チャネル 1 で完了待ちが時間切れになったとき、チャネル 1 にマスクを掛けて AH = 90h。仕様書 2.4 の手順 4、7。"""
    disk = Disk(200_000, 512)
    t = StuckTarget(disk)
    m = Machine(rom, host, {0: t}, hw_ch=1, bios484=0x50)
    m.max_count = 200_000_000
    m.int1b(ax=0x8E80)
    m.dma_out_log.clear()
    r = m.int1b(ax=0x0600, bx=512, cx=0, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 1 and r["ah"] == 0x90, r
    assert m.dmas[1]["mask"], "チャネル 1 にマスクが掛かっていない"
    check_channel_io(m, 1)


def test_dma_channel_wrong(rom, host):
    """ターゲットの要求がチャネル 1 に届くのに 0484h = 00h のとき: チャネル 0 を準備するので転送が起きず、時間切れ（AH = 90h）で返る（止まらない）。"""
    disk = Disk(200_000, 512)
    m = Machine(rom, host, {0: ScsiTarget(disk)}, hw_ch=1, bios484=0x00)
    m.max_count = 200_000_000
    m.int1b(ax=0x8E80)
    r = m.int1b(ax=0x0600, bx=512, cx=0, dx=0, es=0x2000, bp=0)
    assert r["cf"] == 1 and r["ah"] == 0x90, r
    assert m.dmas[0]["mask"], "チャネル 0 にマスクが掛かっていない"


def main():
    rom, host = build()
    assert len(rom) == 0x1000
    names = sys.argv[1:]
    tests = [(k, v) for k, v in globals().items() if k.startswith("test_") and (not names or k in names)]
    failed = 0
    for name, fn in tests:
        try:
            fn(rom, host)
            print("ok  ", name, flush=True)
        except (AssertionError, UcError) as e:
            failed += 1
            print("FAIL", name, repr(e), flush=True)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
