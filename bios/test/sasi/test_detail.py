#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""SASI BIOS の細かい試験（調査側が用意したもの。2026-10-31）。

実装側が仕様書 0.3 版だけから書いた模型と試験（`sasibios/impl_tests/`）を、調査側が仕様書 0.7 版に照らして直して取り込んだもの。
6 バイト型の振る舞い（初期化の順序、センスの読み替え、ステータスのビット、リトライなしのビット、スタックの深さ、時間切れ、ブート）を細かく見る。
付録 C（10 バイト型）は `test_sasi.py` で見る。

- 各試験の docstring に、確かめている仕様書の節がある。試験が仕様書と合わない、または仕様書にない振る舞いを求めていると思ったら、
  試験を直さずに QUESTIONS.md に書く。
- 使い方: `python3 bios/test/sasi/test_detail.py [試験の名前の一部 ...]`（nasm と Python の unicorn が要る）。
  ROM は `sasibios/mkrom.py` の build() で作る（sasi.asm を組み立て、0FFFh で和を合わせる）。
"""

import os
import struct
import subprocess
import sys
import tempfile

from unicorn import Uc, UcError, UC_ARCH_X86, UC_MODE_16, UC_HOOK_INSN, UC_HOOK_INTR, UC_HOOK_CODE, UC_HOOK_MEM_READ
from unicorn.x86_const import (
    UC_X86_INS_IN, UC_X86_INS_OUT,
    UC_X86_REG_AX, UC_X86_REG_BX, UC_X86_REG_CX, UC_X86_REG_DX,
    UC_X86_REG_SI, UC_X86_REG_DI, UC_X86_REG_BP, UC_X86_REG_SP,
    UC_X86_REG_CS, UC_X86_REG_DS, UC_X86_REG_ES, UC_X86_REG_SS,
    UC_X86_REG_EFLAGS, UC_X86_REG_IP,
)

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "sasibios"))
import mkrom                                     # noqa: E402
from models_detail import PIC, DMA, Drive, Target       # noqa: E402

HOST_SEG = 0x1000
SASI_SEG = 0xD700
STACK = 0x7C00


def assemble(src):
    with tempfile.TemporaryDirectory() as tmp:
        out = os.path.join(tmp, "a.bin")
        subprocess.run(["nasm", "-f", "bin", "-w+all", "-o", out, src], check=True)
        with open(out, "rb") as f:
            return f.read()


ROM = mkrom.build()
HOST = assemble(os.path.join(ROOT, "bios", "test", "host.asm"))


class Machine:
    def __init__(self, drives=None, target=None, rom=ROM, segs=(SASI_SEG,), port31=0xFF, sys_type=0x81,
                 init=True):
        self.uc = uc = Uc(UC_ARCH_X86, UC_MODE_16)
        uc.mem_map(0, 0x110000)
        uc.mem_write(HOST_SEG * 16, HOST)
        for s in segs:
            uc.mem_write(s * 16, rom)
        uc.mem_write(0x500, bytes((sys_type,)))
        self.pic = PIC()
        self.dma = DMA(self.mem, lambda a, b: uc.mem_write(a, b))
        self.t = target if target is not None else Target(drives if drives is not None else {0: Drive(1)})
        self.t.dma = self.dma
        self.t.on_change = self.line_changed
        self.port31 = port31
        self.ports = []
        self.line = False
        self.irq_stop = False
        self.wait_hook = None
        self.min_sp = {}
        self.track_sp = None
        uc.hook_add(UC_HOOK_INSN, self.hook_in, None, 1, 0, UC_X86_INS_IN)
        uc.hook_add(UC_HOOK_INSN, self.hook_out, None, 1, 0, UC_X86_INS_OUT)
        uc.hook_add(UC_HOOK_INTR, self.hook_intr)
        uc.hook_add(UC_HOOK_MEM_READ, self.hook_done_read, None, 0x55F, 0x55F)
        self.run(0x0000)
        if init:
            self.init()

    def init(self):
        for entry in (0x0C, 0x0F, 0x12):
            self.run(0x0003, ax=entry)

    # ---- 割り込み
    def line_changed(self):
        line = self.t.int_line()
        if line and not self.line:
            self.pic.irr9 = True
            self.uc.emu_stop()
            self.irq_stop = True
        elif not line and self.line:
            self.pic.irr9 = False
        self.line = line

    def hook_done_read(self, uc, access, addr, size, value, _):
        self.t.poll()

    def hook_wait(self, uc, addr, size, _):
        if self.pic.deliverable() and uc.reg_read(UC_X86_REG_EFLAGS) & 0x200:
            self.irq_stop = True
            uc.emu_stop()

    def push(self, v):
        uc = self.uc
        sp = (uc.reg_read(UC_X86_REG_SP) - 2) & 0xFFFF
        uc.mem_write(uc.reg_read(UC_X86_REG_SS) * 16 + sp, struct.pack("<H", v))
        uc.reg_write(UC_X86_REG_SP, sp)

    def enter(self, n, flags_mask=0x0300):
        uc = self.uc
        flags = uc.reg_read(UC_X86_REG_EFLAGS) & 0xFFFF
        self.push(flags)
        self.push(uc.reg_read(UC_X86_REG_CS))
        self.push(uc.reg_read(UC_X86_REG_IP))
        uc.reg_write(UC_X86_REG_EFLAGS, flags & ~flags_mask)
        ip, cs = struct.unpack("<HH", self.mem(n * 4, 4))
        uc.reg_write(UC_X86_REG_CS, cs)
        uc.reg_write(UC_X86_REG_IP, ip)

    def hook_intr(self, uc, n, _):
        self.enter(n)

    # ---- I/O
    def hook_in(self, uc, port, size, _):
        if port in (0x80, 0x82):
            v = self.t.read_data() if port == 0x80 else self.t.read_ctrl()
        elif port in (0x00, 0x02, 0x08, 0x0A):
            v = self.pic.inp(port)
        elif port in (0x01, 0x03):
            v = self.dma.inp(port)
        elif port == 0x31:
            v = self.port31
        else:
            v = 0xFF
        self.ports.append(("in", port, v))
        return v

    def hook_out(self, uc, port, size, value, _):
        value &= 0xFF
        self.ports.append(("out", port, value))
        if port == 0x80:
            self.t.write_data(value)
        elif port == 0x82:
            self.t.write_ctrl(value)
        elif port in (0x00, 0x02, 0x08, 0x0A):
            self.pic.out(port, value)
        elif port in (0x01, 0x03, 0x15, 0x17, 0x19, 0x27):
            self.dma.out(port, value)
            if port == 0x15:
                self.t.pump()
                self.line_changed()

    def hook_sp(self, uc, addr, size, _):
        sp = uc.reg_read(UC_X86_REG_SP)
        if self.track_sp is None or sp < self.track_sp:
            self.track_sp = sp

    # ---- 実行
    def run(self, ip, until=None, count=50_000_000, **regs):
        uc = self.uc
        uc.reg_write(UC_X86_REG_SS, 0)
        uc.reg_write(UC_X86_REG_SP, STACK)
        uc.reg_write(UC_X86_REG_DS, 0)
        uc.reg_write(UC_X86_REG_ES, regs.pop("es", 0))
        uc.reg_write(UC_X86_REG_EFLAGS, regs.pop("flags", 0x0202))
        names = {"ax": UC_X86_REG_AX, "bx": UC_X86_REG_BX, "cx": UC_X86_REG_CX,
                 "dx": UC_X86_REG_DX, "si": UC_X86_REG_SI, "di": UC_X86_REG_DI,
                 "bp": UC_X86_REG_BP}
        for k, v in regs.items():
            uc.reg_write(names[k], v)
        uc.reg_write(UC_X86_REG_CS, HOST_SEG)
        begin = HOST_SEG * 16 + ip
        while True:
            self.irq_stop = False
            uc.emu_start(begin, until or 0xFFFFFFFF, count=count)
            if not self.irq_stop:
                break
            if self.pic.deliverable() and uc.reg_read(UC_X86_REG_EFLAGS) & 0x200:
                if self.wait_hook is not None:
                    uc.hook_del(self.wait_hook)
                    self.wait_hook = None
                self.pic.acknowledge()
                self.enter(0x11)
            elif self.pic.irr9 and self.wait_hook is None:
                self.wait_hook = uc.hook_add(UC_HOOK_CODE, self.hook_wait)
                uc.ctl_flush_tb()               # 翻訳済みのブロックにもフックを効かせる
            begin = uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP)
            if until is not None and begin == until:
                break
        return uc.reg_read(UC_X86_REG_CS) * 16 + uc.reg_read(UC_X86_REG_IP)

    def int1b(self, sp_check=False, **regs):
        h = None
        if sp_check:
            self.track_sp = None
            h = self.uc.hook_add(UC_HOOK_CODE, self.hook_sp, None, SASI_SEG * 16, SASI_SEG * 16 + 0xFFF)
            self.uc.ctl_flush_tb()
        self.run(0x0009, **regs)
        if h is not None:
            self.uc.hook_del(h)
        uc = self.uc
        r = {k: uc.reg_read(v) for k, v in (
            ("ax", UC_X86_REG_AX), ("bx", UC_X86_REG_BX), ("cx", UC_X86_REG_CX),
            ("dx", UC_X86_REG_DX), ("bp", UC_X86_REG_BP), ("es", UC_X86_REG_ES),
            ("si", UC_X86_REG_SI), ("di", UC_X86_REG_DI), ("ds", UC_X86_REG_DS),
            ("sp", UC_X86_REG_SP))}
        r["cf"] = uc.reg_read(UC_X86_REG_EFLAGS) & 1
        r["ah"] = r["ax"] >> 8
        return r

    def mem(self, addr, n):
        return bytes(self.uc.mem_read(addr, n))

    def byte(self, addr):
        return self.mem(addr, 1)[0]

    def word(self, addr):
        return struct.unpack("<H", self.mem(addr, 2))[0]

    def table(self, unit):
        off, seg = struct.unpack("<HH", self.mem(0x5E8 + unit * 4, 4))
        return seg, off, self.mem(seg * 16 + off, 30)

    def ops(self, start=0):
        return [(op, lun) for op, lun, *_ in self.t.log[start:]]


def tbl(offset):
    return ROM[offset:offset + 30]


TABLE_A = {0: bytes.fromhex("0101000300984000"), 1: bytes.fromhex("0101000301358000"),
           2: bytes.fromhex("0101000501358000"), 3: bytes.fromhex("0101000701358000"),
           4: bytes.fromhex("0101000302660000"), 5: bytes.fromhex("0101000502660000"),
           6: bytes.fromhex("0101000702660000")}
TABLE_B = {0: bytes.fromhex("0101000301544000"), 1: bytes.fromhex("0101000301548000"),
           2: bytes.fromhex("0101000501548000"), 3: bytes.fromhex("0101000701548000"),
           4: bytes.fromhex("010100030 2A00000".replace(" ", "")), 5: bytes.fromhex("01010005 02A00000".replace(" ", "")),
           6: bytes.fromhex("0101000702A00000")}
SENSE_AH = {0: (0x00, 0x00), 1: (0x01, 0x01), 2: (0x02, 0x02), 3: (0x03, 0x03), 4: (0x03, 0x03), 5: (0x04, 0x05),
            6: (0x05, 0x07)}
GEOM = {0: (153, 4), 1: (310, 4), 2: (310, 6), 3: (310, 8), 4: (615, 4), 5: (615, 6), 6: (615, 8)}


# ================================================================ ヘッダと ROM（1.1、1.2、4.1）

def test_header():
    """仕様書 1.1（0000h〜0008h）、1.2（和）、4.1。"""
    assert len(ROM) == 0x1000 and sum(ROM) & 0xFF == 0
    assert ROM[0:9] == b"\xCB\x90\x90" * 3
    assert ROM[9:11] == b"\x55\xAA" and ROM[0x0B] == 0x29
    for off in (0x0C, 0x0F, 0x15, 0x18):
        assert ROM[off] == 0xE9, hex(off)
    assert ROM[0x12] == 0xCB


def drop_probe(log):
    """付録 C の見分けの命令（READ CAPACITY と、その直後の同じ LUN の REQUEST SENSE）を除く。
    見分けと 4.4 の手順 3 の順序はユニットをまたいで問わない（0.7 版の 4.4 の 3、Q46）ので、比べる前に除く。"""
    out, skip = [], None
    for e in log:
        if e[0] == 0x25:
            skip = e[1]
            continue
        if skip is not None and e[0] == 0x03 and e[1] == skip:
            skip = None
            continue
        skip = None
        out.append(e)
    return out


# ================================================================ 初期化 1、2（4.2、4.3、4.4）

def test_init_register():
    """仕様書 4.2、4.3、2.6（05E8h / 05ECh）。"""
    d0, d1 = Drive(1), Drive(3, sec512=True)
    m = Machine({0: d0, 1: d1})
    assert m.byte(0x4D7) == 0xE9
    assert m.byte(0x4B0) == 0xD7 and m.byte(0x4B8) == 0xD7
    assert m.byte(0x480) & 0x80
    assert m.word(0x44) != 0 and m.word(0x46) == SASI_SEG       # INT 11h
    assert not m.pic.imr[0x0A] & 0x02                            # IR9 のマスクを外した
    assert m.byte(0x55D) & 3 == 3
    assert m.byte(0x481) & 3 == 2                                # 0E01h: 512 バイトのユニット 1 は閉鎖
    assert m.byte(0x55F) & 1 == 0
    # 割り込みは EOI まで済んでいる
    assert m.pic.delivered > 0 and m.pic.isr == {0x00: 0, 0x08: 0}
    # ユニット 1 は 512 バイト、容量コード 011b
    assert m.table(1)[2] == tbl(m.table(1)[1]) and m.table(1)[2][:8] == TABLE_A[3]
    assert m.table(1)[0] == SASI_SEG


def test_init_sequence():
    """仕様書 4.4 の 3、4（Q16、Q17）、3.3 のシリンダだけのシーク（Q9）、付録 C の見分けの時期（Q35、Q46）。"""
    d0 = Drive(2)
    m = Machine({0: d0})
    log = m.t.log
    # 付録 C の見分け（READ CAPACITY と REQUEST SENSE）は除いて比べる（Q35、Q46）
    log = drop_probe(log)
    # 組 B のパラメータ設定 → TEST UNIT READY。ユニット 1 は 111b なので最初の段では何もしない
    ops = [op for op, *_ in log]
    assert ops[:2] == [0xC2, 0x00]
    # 後の段は両方のユニットに行う（Q17）。ドライブのない LUN 1 は SEEK と REZERO がエラー（センスを取る）
    assert ops[2:10] == [0xC2, 0xC2, 0x0B, 0x01, 0xC2, 0xC2, 0x0B, 0xC2], [hex(o) for o in ops]
    assert ops[10:] == [0xC2, 0xC2, 0x0B, 0x03, 0x01, 0x03, 0xC2, 0xC2, 0x0B, 0x03, 0xC2], [hex(o) for o in ops]
    assert {lun for _, lun, *_ in log[2:10]} == {0} and {lun for _, lun, *_ in log[10:]} == {1}
    # リキャリブレートのシークはシリンダ 5、リトラクトは組 B の +0Eh（物理ヘッド数で計算、Q9）
    seeks = [lba for op, lun, lba, *_ in log if op == 0x0B and lun == 0]
    assert seeks == [5 * 6 * 33, 0x154 * 6 * 33], seeks
    # 最後に設定したのは組 A
    assert m.t.params[0][:8] == TABLE_A[2] and m.t.params[0][8:] == b"\0\0"
    assert m.byte(0x55D) & 3 == 1
    assert m.t.resets == 2                                       # 初期化 2 の始めと、03h の始め


def test_init_tur_retry():
    """仕様書 4.4 の 3（回数は TEST UNIT READY の合計。Q16。見分けとの順序は Q46）。"""
    d0 = Drive(1)
    d0.not_ready = 5
    m = Machine({0: d0}, sys_type=0x01)                          # 0500h の bit7 = 0: 30 回まで
    assert m.byte(0x55D) & 1
    ops = [op for op, *_ in drop_probe(m.t.log)]                 # 付録 C の見分けを除く（Q35、Q46）
    assert ops[:12] == [0xC2] + [0x00, 0x03] * 5 + [0x00]
    d0 = Drive(1)
    d0.not_ready = 5
    m = Machine({0: d0}, sys_type=0x81)                          # bit7 = 1: 2 回まで
    assert m.byte(0x55D) & 1 == 0
    ops = [op for op, *_ in drop_probe(m.t.log)]
    assert ops[:6] == [0xC2, 0x00, 0x03, 0x00, 0x03, 0xC2]


def test_init_10mb():
    """仕様書 4.3 の 5（Q12）、3.4（10 MB の別の表を保つ。Q11）。"""
    d0 = Drive(1)
    m = Machine({0: d0})
    seg, off, t = m.table(0)
    assert t[:18] == TABLE_A[1] + b"\0\0" + bytes.fromhex("010100030 14C8000".replace(" ", ""))
    # 最後はリトラクト: シリンダ 14Ch へ
    seeks = [lba for op, lun, lba, *_ in m.t.log if op == 0x0B and lun == 0]
    assert seeks[-1] == 0x14C * 4 * 33 and seeks[-2] == 0, seeks
    # シークに時間がかかれば、普通の表のまま
    d0 = Drive(1)
    d0.seek_delay = 100_000
    m = Machine({0: d0})
    assert m.table(0)[2][0x0E:0x10] == b"\x01\x54"
    # 10 MB の別の表は、モード設定（8Eh）をしても保つ（Q11）
    m = Machine({0: Drive(1)})
    r = m.int1b(ax=0x8E00)
    assert r["ah"] == 0 and m.table(0)[2][0x0E:0x10] == b"\x01\x4C"


def test_init_dipsw():
    """仕様書 4.3 の 6（I/O 31h の bit5）。"""
    m = Machine({0: Drive(1)}, port31=0xFF & ~0x20)
    assert m.byte(0x55D) & 3 == 0


def test_no_controller():
    """仕様書 4.3 の 1（登録は取り消さない。Q32）。"""
    m = Machine(target=Target({0: Drive(1)}, present=False))
    assert m.byte(0x4B0) == 0xD7                                 # 登録は初期化 1 で済んでいる
    assert m.word(0x46) == 0                                     # 割り込みベクタは置かない
    assert m.byte(0x55D) == 0


def test_two_copies():
    """仕様書 1.2、4.2、4.3。"""
    m = Machine({0: Drive(1)}, segs=(0xD500, SASI_SEG))
    assert m.byte(0x4D5) == 0xE9 and m.byte(0x4D7) == 0xE9
    assert m.byte(0x4B0) == 0xD5
    assert m.word(0x46) == 0xD500
    assert m.t.resets == 2                                       # 初期化 2 は持ち主だけ
    n = len(m.t.log)
    m.run(0x0006, ax=0x0C)                                       # ブートの AL が対象外: 何もせず戻る
    assert len(m.t.log) == n


# ================================================================ INT 1Bh（6）

def test_unused_codes():
    """仕様書 6.2（該当しない機能コード）、6.3。"""
    m = Machine({0: Drive(1)})
    n = len(m.t.log)
    for ah in (0x02, 0x09, 0x0A, 0x0C, 0x22, 0x89):
        r = m.int1b(ax=ah << 8 | 0x80, bx=0x1234, cx=0x5678, dx=0x9ABC)
        assert r["cf"] == 0 and r["ah"] == 0 and r["bx"] == 0x1234 and r["cx"] == 0x5678 and r["dx"] == 0x9ABC
        assert r["sp"] == STACK
    assert len(m.t.log) == n


def test_sense_all_types():
    """仕様書 3.1、3.2、6.3 の 4h（センスと新センス）。"""
    for code in range(7):
        for sec512 in (False, True):
            d = Drive(code, sec512=sec512)
            m = Machine({1: d})
            m.int1b(ax=0x8E01)                                   # 開放、薄型
            r = m.int1b(ax=0x0481)
            assert r["cf"] == 0 and r["ah"] == SENSE_AH[code][0], (code, r)
            r = m.int1b(ax=0x8481)
            cyl, heads = GEOM[code]
            assert r["ah"] == SENSE_AH[code][1], (code, r)
            assert r["bx"] == (512 if sec512 else 256)
            assert r["cx"] == cyl - 1 and r["dx"] == heads << 8 | (17 if sec512 else 33), (code, r)
            assert m.t.log[-1][0] == 0x00


def test_sense_not_ready():
    """仕様書 4.5、6.3 の 4h。"""
    d = Drive(1)
    m = Machine({0: d})
    d.not_ready = 1
    r = m.int1b(ax=0x8400, bx=0x1111)
    assert r["cf"] == 1 and r["ah"] == 0x60 and r["bx"] == 0x1111, r
    assert [op for op, *_ in m.t.log[-2:]] == [0x00, 0x03]
    assert m.mem(0x586, 4) == b"\x04\x00\x00\x00"


def test_missing_unit():
    """仕様書 6.2（055Dh は見ない）。"""
    m = Machine({0: Drive(1)})
    r = m.int1b(ax=0x0601, bx=256, es=0x2000, bp=0)              # 055Dh は見ずにコマンドを出す
    assert r["cf"] == 1 and r["ah"] == 0x60, r
    assert m.t.log[-2][:2] == (0x08, 1)


def read(m, al, bx, cx, dx, es=0x2000, bp=0, ah=0x06):
    return m.int1b(ax=ah << 8 | al, bx=bx, cx=cx, dx=dx, es=es, bp=bp)


def test_read_linear():
    """仕様書 3.3（リニア指定）、6.4。"""
    d = Drive(3)
    m = Machine({0: d})
    r = read(m, 0x00, 3 * 256, 1000, 0)
    assert r["cf"] == 0 and r["ah"] == 0, r
    assert m.mem(0x20000, 768) == d.sector(1000, 3)
    assert m.t.log[-1] == (0x08, 0, 1000, 3, 0)
    assert m.dma.modes[-1] == 0x44 and m.dma.mask
    assert 0xC3 in m.t.ctrl_writes[-8:]
    # DL の bit4〜0 が LBA の bit20〜16、DH は見ない
    d = Drive(6)
    m = Machine({0: d})
    r = read(m, 0x00, 256, 0x1234, 0xFF01, es=0x3000, bp=0x10)
    assert r["ah"] == 0 and m.t.log[-1][2] == 0x11234
    assert m.mem(0x30010, 256) == d.sector(0x11234)


def test_read_chs():
    """仕様書 3.3（CHS 指定）、6.4。"""
    d = Drive(3)
    m = Machine({0: d})
    lba = (100 * 8 + 5) * 33 + 30
    r = read(m, 0x80, 4 * 256, 100, 0x051E, es=0x1000, bp=0x8000)
    assert r["ah"] == 0, r
    assert m.t.log[-1][2] == lba and m.mem(0x18000, 1024) == d.sector(lba, 4)
    # 512 バイト: S = 17、ブロック数 = BH ÷ 2
    d = Drive(5, sec512=True)
    m = Machine({0: d})
    m.int1b(ax=0x8E00)
    lba = (200 * 6 + 3) * 17 + 16
    r = read(m, 0x80, 0x0600, 200, 0x0310)
    assert r["ah"] == 0 and m.t.log[-1][2:4] == (lba, 3), (r, m.t.log[-1])
    assert m.mem(0x20000, 0x600) == d.sector(lba, 3)


def test_thick_thin():
    """仕様書 3.1、3.4（厚型 / 薄型）。"""
    d = Drive(4)
    m = Machine({0: d})
    # 厚型（初期化のあと）: 8 ヘッドとして計算
    r = read(m, 0x80, 256, 10, 0x0702)
    assert r["ah"] == 0 and m.t.log[-1][2] == (10 * 8 + 7) * 33 + 2
    r = m.int1b(ax=0x8400)
    assert r["dx"] == 0x0421 and r["cx"] == 614
    m.int1b(ax=0x8E00)
    r = read(m, 0x80, 256, 10, 0x0302)
    assert r["ah"] == 0 and m.t.log[-1][2] == (10 * 4 + 3) * 33 + 2


def test_bx_counts():
    """仕様書 6.4 の 3（ブロック数と DMA のカウント）。"""
    d = Drive(3)
    m = Machine({0: d})
    r = read(m, 0x00, 0, 0, 0, es=0x4000)                         # BX = 0: 64 KB、256 ブロック
    assert r["ah"] == 0 and m.t.log[-1][3] == 0 and m.mem(0x40000, 65536) == d.sector(0, 256)
    assert m.dma.mask
    # 512 バイトで BX = 0: 80h ブロック
    d = Drive(6, sec512=True)
    m = Machine({0: d})
    m.int1b(ax=0x8E00)
    r = read(m, 0x00, 0, 0, 0, es=0x4000)
    assert r["ah"] == 0 and m.t.log[-1][3] == 0x80 and m.mem(0x40000, 65536) == d.sector(0, 128)
    r = read(m, 0x00, 0x0200, 7, 0)
    assert m.t.log[-1][3] == 0x01


def test_dma_boundary():
    """仕様書 2.3（64 KB の境界）。"""
    m = Machine({0: Drive(3)})
    n = len(m.t.log)
    r = read(m, 0x00, 0x200, 0, 0, es=0x1FFF, bp=0x0F)
    assert r["cf"] == 1 and r["ah"] == 0x20 and len(m.t.log) == n
    r = read(m, 0x00, 0x400, 0, 0, es=0x1FC0, bp=0)               # ちょうど 20000h で終わる
    assert r["cf"] == 0 and r["ah"] == 0
    r = read(m, 0x00, 0, 0, 0, es=0x1000, bp=1)                    # BX = 0 は 64 KB
    assert r["ah"] == 0x20
    assert m.dma.mask


def test_write():
    """仕様書 6.3 の 5h、6.4。"""
    d = Drive(3)
    m = Machine({0: d})
    payload = bytes((i * 13) & 0xFF for i in range(512))
    m.uc.mem_write(0x30005, payload)
    r = read(m, 0x00, 512, 77, 0, es=0x3000, bp=5, ah=0x05)
    assert r["ah"] == 0 and r["cf"] == 0, r
    assert d.sector(77, 2) == payload and m.dma.modes[-1] == 0x48
    assert m.t.log[-1] == (0x0A, 0, 77, 2, 0)


def test_verify():
    """仕様書 6.3 の 1h（82h の bit1）、2.3（ベリファイ転送）。"""
    m = Machine({0: Drive(3)})
    before = m.mem(0x20000, 512)
    r = read(m, 0x00, 512, 5, 0, ah=0x01)
    assert r["ah"] == 0 and m.t.log[-1][0] == 0x08 and m.dma.modes[-1] == 0x40
    assert m.mem(0x20000, 512) == before
    m = Machine(target=Target({0: Drive(3)}, verify_bit=True))
    r = read(m, 0x00, 512, 5, 0, ah=0x01)
    assert r["ah"] == 0 and m.t.log[-1][0] == 0x05


def test_out_of_range():
    """仕様書 4.5（センス 21h → 38h）。"""
    m = Machine({0: Drive(1)})
    r = read(m, 0x80, 256, 310, 0)                                # シリンダの上限（組 A は 135h）を超える
    assert r["cf"] == 1 and r["ah"] == 0x38, r
    assert m.mem(0x586, 4)[0] == 0xA1
    assert m.byte(0x585) == 0x00                                  # REQUEST SENSE のステータス（Q7）


def test_sense_map():
    """仕様書 4.5（センスの読み替えの表）。"""
    cases = {0x00: 0x40, 0x04: 0x60, 0x10: 0xA0, 0x11: 0xB0, 0x12: 0xE0, 0x13: 0xF0, 0x14: 0xC0, 0x15: 0xC8,
             0x16: 0x40, 0x17: 0x70, 0x19: 0xD0, 0x1C: 0xB8, 0x1E: 0x88, 0x1F: 0x50, 0x21: 0x38, 0x23: 0x30,
             0x20: 0x40, 0x3F: 0x40}
    m = Machine({0: Drive(1)})
    for code, ah in cases.items():
        orig = m.t.command

        def bad(code=code, orig=orig):
            c = m.t.cdb
            if c[0] == 0x0B:
                m.t.log.append((c[0], 0, 0, 0, 0))
                return m.t.finish(0, False, code)
            return orig()
        m.t.command = bad
        r = m.int1b(ax=0x0080, cx=0, dx=0)
        m.t.command = orig
        assert r["ah"] == ah and r["cf"] == (ah >= 0x20), (hex(code), r)


def test_status_bits():
    """仕様書 2.4 の 6（bit1 を先に見る。Q1）。"""
    m = Machine({0: Drive(1)})
    orig = m.t.command
    # bit1 を先に見る: 03h はセンスを取る（センスは 00h なので AH = 40h）。01h、21h はセンスを取らずに 40h（Q1）
    for status, ah, sense in ((0x01, 0x40, False), (0x03, 0x40, True), (0x21, 0x40, False)):
        def odd(status=status):
            c = m.t.cdb
            if c[0] == 0x00:
                m.t.status = status
                m.t.phase = "status"
                return
            return orig()
        m.t.command = odd
        n = len(m.t.log)
        r = m.int1b(ax=0x0400)
        assert r["ah"] == ah, (status, r)
        assert any(op == 0x03 for op, *_ in m.t.log[n:]) == sense, (status, m.t.log[n:])
    m.t.command = orig


def test_retry_bit():
    """仕様書 2.5（制御バイト C0h。Q28）。"""
    m = Machine({0: Drive(3)})
    r = read(m, 0x00, 256, 0, 0, ah=0x26)
    assert r["ah"] == 0 and m.t.log[-1] == (0x08, 0, 0, 1, 0xC0)
    r = read(m, 0x00, 256, 0, 0, ah=0x06)
    assert m.t.log[-1][4] == 0x00


def test_corrected_error():
    """仕様書 6.4 の 5（読み直し。Q19、Q20、Q44）。"""
    d = Drive(3)
    m = Machine({0: d})
    d.corrected = {103}
    r = read(m, 0x00, 8 * 256, 100, 0, es=0x5000)
    assert r["cf"] == 0 and r["ah"] == 0x08, r
    assert m.mem(0x50000, 8 * 256) == d.sector(100, 8)
    assert [x for x in m.t.log[-3:]] == [(0x08, 0, 100, 8, 0), (0x03, 0, 0, 4, 0), (0x08, 0, 104, 4, 0)]
    # 2 回続いても最後まで読む
    d.corrected = {101, 105}
    r = read(m, 0x00, 8 * 256, 100, 0, es=0x5000)
    assert r["ah"] == 0x08 and m.mem(0x50000, 8 * 256) == d.sector(100, 8)
    # 最後のブロックなら読み直さない
    d.corrected = {107}
    n = len(m.t.log)
    r = read(m, 0x00, 8 * 256, 100, 0, es=0x5000)
    assert r["ah"] == 0x08 and len(m.t.log) == n + 2
    # リトライなし: B8h
    d.corrected = {103}
    r = read(m, 0x00, 8 * 256, 100, 0, es=0x5000, ah=0x26)
    assert r["ah"] == 0xB8 and r["cf"] == 1


def test_closed():
    """仕様書 3.4（閉鎖。Q27）。"""
    d = Drive(3, sec512=True)
    m = Machine({0: d})
    assert m.byte(0x481) & 1
    n = len(m.t.log)
    for ah in (0x06, 0x05, 0x01, 0x0D, 0x8D, 0x08, 0x0B, 0x00):
        r = m.int1b(ax=ah << 8, bx=0x200, es=0x2000, bp=0)
        assert r["ah"] == 0x40 and r["cf"] == 1, (hex(ah), r)
    assert len(m.t.log) == n
    r = m.int1b(ax=0x8400)                                       # センスは閉鎖中でも答える
    assert r["ah"] == 0x03 and r["bx"] == 0x200
    r = m.int1b(ax=0x8E00)
    assert r["ah"] == 0 and m.byte(0x481) & 1 == 0
    r = read(m, 0x00, 0x200, 0, 0)
    assert r["ah"] == 0
    r = m.int1b(ax=0x0E00)
    assert m.byte(0x481) & 1
    # 256 バイトのユニットは 0Eh でも開放のまま
    m = Machine({0: Drive(3), 1: Drive(3, sec512=True)})
    m.int1b(ax=0x8E01)
    m.int1b(ax=0x0E00)
    assert m.byte(0x481) & 3 == 0


def test_seek():
    """仕様書 6.3 の 0h（Q13）。"""
    d = Drive(3)
    m = Machine({0: d})
    r = m.int1b(ax=0x0080, cx=100, dx=0x0203)
    assert r["ah"] == 0 and r["cf"] == 0
    assert [x[0] for x in m.t.log[-2:]] == [0xC2, 0x0B]
    assert m.t.log[-1][2] == (100 * 8 + 2) * 33 + 3
    assert m.t.params[0][:8] == TABLE_B[3]                       # 組 B のまま（Q13）
    d.seek_delay = 70_000
    r = m.int1b(ax=0x0080, cx=1, dx=0)
    assert r["ah"] == 0x10 and r["cf"] == 0, r
    d.seek_delay = 1000
    r = m.int1b(ax=0x0080, cx=1, dx=0)
    assert r["ah"] == 0x00, r


def test_recal_retract():
    """仕様書 6.3 の 7h、Fh、3.3 のシリンダだけのシーク（Q9）、2.5（Q28）。"""
    m = Machine({0: Drive(6)})
    n = len(m.t.log)
    r = m.int1b(ax=0x0700)
    assert r["ah"] == 0 and r["cf"] == 0
    assert [(op, lba) for op, _, lba, *_ in m.t.log[n:]] == [(0xC2, 0), (0x0B, 5 * 8 * 33), (0x01, 0), (0xC2, 0)]
    assert m.t.params[0][:8] == TABLE_A[6]
    n = len(m.t.log)
    r = m.int1b(ax=0x0F00)
    assert [(op, lba) for op, _, lba, *_ in m.t.log[n:]] == [(0xC2, 0), (0x0B, 0x2A0 * 8 * 33), (0xC2, 0)]
    r = m.int1b(ax=0x2F00)                                       # MS-DOS の 2Fh
    assert r["ah"] == 0 and m.t.log[-1][4] == 0xC0


def test_format():
    """仕様書 6.3 の Dh（Q18）。"""
    d = Drive(3)
    m = Machine({0: d})
    r = m.int1b(ax=0x0D80, bx=0x0300, cx=2, dx=0x0100)
    assert r["ah"] == 0, r
    lba = (2 * 8 + 1) * 33
    assert m.t.log[-1] == (0x06, 0, lba, 3, 0)
    assert d.sector(lba, 33) == b"\xE5" * 33 * 256 and d.sector(lba + 33)[0] != 0xE5
    n = len(m.t.log)
    r = m.int1b(ax=0x8D80, bx=0x0100, cx=0, dx=0)
    assert r["ah"] == 0
    assert [op for op, *_ in m.t.log[n:]] == [0xC2, 0x0B, 0x01, 0xC2, 0x04]
    assert d.sector(5000)[0] == 0xE5


def test_alt_track():
    """仕様書 6.3 の 8h、Bh。"""
    d = Drive(3)
    m = Machine({0: d})
    m.uc.mem_write(0x30000, b"\xAA" * 8)
    r = m.int1b(ax=0x0880, cx=300, dx=0x0700, es=0x3000, bp=0)
    lba = (300 * 8 + 7) * 33
    assert r["ah"] == 0 and m.mem(0x30000, 6) == bytes((lba >> 16, lba >> 8 & 0xFF, lba & 0xFF, 0, 0xAA, 0xAA))
    n = len(m.t.log)
    r = m.int1b(ax=0x0B80, bx=4, cx=10, dx=0, es=0x3000, bp=0)
    assert r["ah"] == 0, r
    assert m.t.log[n:] == [(0x0E, 0, 10 * 8 * 33, 0, 0)]
    assert d.alt == [(10 * 8 * 33, m.mem(0x30000, 4))]
    assert m.dma.modes[-1] == 0x48


def test_init_function():
    """仕様書 4.4（INT 1Bh の 03h）。"""
    d0 = Drive(1)
    m = Machine({0: d0})
    m.uc.mem_write(0x55D, b"\xFF")
    m.uc.mem_write(0x55F, b"\xFF")
    d0.not_ready = 1
    r = m.int1b(ax=0x0300, sp_check=True)
    assert r["ah"] == 0 and r["cf"] == 0
    assert m.byte(0x55D) == 0xFD                                 # ユニット 0 だけ（2 回目で準備できた）
    assert m.byte(0x55F) == 0xFC
    m2 = Machine(target=Target({0: Drive(1)}, present=False))
    r = m2.int1b(ax=0x0300)
    assert r["ah"] == 0 and r["cf"] == 0


def test_timeout():
    """仕様書 2.4 の 1、7（時間切れの後始末。Q21）。"""
    m = Machine({0: Drive(1)})
    m.t.select_ok = False
    r = read(m, 0x00, 256, 0, 0)
    assert r["ah"] == 0x90 and r["cf"] == 1
    assert m.t.ctrl_writes[-1] == 0x40                           # SEL を落とした（Q21）


def test_stack_depth():
    """仕様書 2.6（スタック。Q40、Q47、0.8 版）。"""
    d = Drive(3)
    m = Machine({0: d})
    worst = 0
    for kw in (dict(ax=0x8D00, bx=0x100), dict(ax=0x0600, bx=512, es=0x2000), dict(ax=0x8400),
               dict(ax=0x0300), dict(ax=0x0680, bx=256, cx=400, es=0x2000)):
        r = m.int1b(sp_check=True, **kw)
        frame_top = STACK - 6 - 18                               # INT の 3 語と、本体が積んだ 9 語
        worst = max(worst, frame_top - m.track_sp)
        assert r["sp"] == STACK
    # 割り込みハンドラを含めて 28 バイトまで（仕様書 0.8 版の 2.6）。
    # 測る範囲は SASI の ROM の中を走った間で、割り込みハンドラの中も含む
    print(f"      INT 1Bh の枠の下で使ったスタック（割り込みハンドラの中を含む）: {worst}")
    assert worst <= 28, worst


# ================================================================ ブート（5）

def ipl(d, tag=b"IPL1"):
    d.data[0:8] = b"\xEB\x0A\x90\x90" + tag


def test_boot_256():
    """仕様書 5（256 バイトのセクタ）。"""
    d = Drive(1)
    ipl(d)
    m = Machine({0: d})
    saved = m.mem(0x4AC, 4)
    end = m.run(0x0006, until=0x1FE00, ax=0x0A)
    assert end == 0x1FE00, hex(end)
    uc = m.uc
    assert m.mem(0x1FE00, 512) == d.sector(0, 2)
    assert m.byte(0x584) == 0x80
    assert uc.reg_read(UC_X86_REG_AX) & 0xFF == 0x80
    assert uc.reg_read(UC_X86_REG_BX) == 0x200 and uc.reg_read(UC_X86_REG_BP) == 0
    assert uc.reg_read(UC_X86_REG_ES) == 0x1FE0 and uc.reg_read(UC_X86_REG_DS) == 0
    assert m.mem(0x4AC, 4) == b"\x00\x00\xE0\x1F"
    # 84h（TEST UNIT READY）、07h（C2h B、SEEK、REZERO、C2h A）、06h（READ）
    assert [op for op, *_ in m.t.log[-6:]] == [0x00, 0xC2, 0x0B, 0x01, 0xC2, 0x08]
    assert m.t.log[-1][:4] == (0x08, 0, 0, 2)
    assert saved != m.mem(0x4AC, 4)


def test_boot_512_unit1():
    """仕様書 5（512 バイトのセクタ、81h）。"""
    d = Drive(3, sec512=True)
    ipl(d)
    m = Machine({0: Drive(1), 1: d})
    assert m.byte(0x481) & 2
    end = m.run(0x0006, until=0x1FC00, ax=0x0B)
    assert end == 0x1FC00, hex(end)
    assert m.mem(0x1FC00, 1024) == d.sector(0, 2)
    assert m.byte(0x584) == 0x81 and m.uc.reg_read(UC_X86_REG_BX) == 0x400
    assert m.byte(0x481) & 2 == 0                                # 8Eh で開放した


def test_boot_no_ipl():
    """仕様書 5 の 4（IPL1 でなければあきらめる）。"""
    d = Drive(1)
    ipl(d, b"IPL0")
    m = Machine({0: d})
    end = m.run(0x0006, ax=0x0A)
    assert end != 0x1FE00
    assert [op for op, *_ in m.t.log].count(0x08) == 1            # やり直さない
    # 04ACh〜04AFh を戻したので、本体の模型は残りの区画を数え終えた（E000:0015h）
    assert m.mem(0x4AC, 4) == b"\x15\x00\x00\xE0"
    # あきらめたあと 0Eh と 0Fh: 最後はリトラクト（C2h B、SEEK、C2h A）
    assert [op for op, *_ in m.t.log[-3:]] == [0xC2, 0x0B, 0xC2]


def test_boot_read_errors():
    """仕様書 5 の 3（8 回まで）。"""
    d = Drive(1)
    ipl(d)
    m = Machine({0: d})
    d.data = bytearray()                                          # どこを読んでも範囲外
    d.blocks = 0
    n = len(m.t.log)
    end = m.run(0x0006, ax=0x0A)
    assert end != 0x1FE00
    assert [op for op, *_ in m.t.log[n:]].count(0x08) == 8


def test_boot_not_present():
    """仕様書 5（055Dh のビットがなければ何もしない）。"""
    m = Machine({0: Drive(1)})
    n = len(m.t.log)
    m.run(0x0006, ax=0x0B)                                        # 81h はない
    m.run(0x0006, ax=0x01)
    assert len(m.t.log) == n


def test_stack_depth_host():
    """仕様書 2.6（スタック）。本体が起動のときに呼ぶ入口（初期化 1〜3、ブート）の深さ。
    上限は仕様書 0.8 版の 2.6: 初期化 1〜3 は 34 バイト、ブートは 50 バイト（本体の INT 1Bh の 24 バイトを含む）。
    深さは far call の戻り番地の下で数える（模型の本体が積む分は引く）。"""
    def track(m):
        m.track_sp = None
        h = m.uc.hook_add(UC_HOOK_CODE, m.hook_sp, None, SASI_SEG * 16, SASI_SEG * 16 + 0xFFF)
        m.uc.ctl_flush_tb()
        return h
    # 初期化 1〜3: host.asm の call_entry は 5 語を積んで far call する（5 語 + 戻り番地 2 語 = 14 バイト）
    depth = {}
    for drives in ({0: Drive(3)}, {0: Drive(1), 1: Drive(3, sec512=True)}):
        m = Machine(drives, init=False)
        h = track(m)
        for entry in (0x0C, 0x0F, 0x12):
            m.track_sp = None
            m.run(0x0003, ax=entry)
            if m.track_sp is not None:
                depth[entry] = max(depth.get(entry, 0), STACK - 14 - m.track_sp)
        m.uc.hook_del(h)
    # ブート: call_boot は 3 語を積んで far call する（3 語 + 戻り番地 2 語 = 10 バイト）。
    # BIOS は本体の INT 1Bh で読むので、本体の INT 1Bh が積む 12 語（24 バイト）も入る
    worst_boot = 0
    for fail in (False, True):
        d = Drive(1)
        ipl(d)
        m = Machine({0: d})
        if fail:                                                  # 読み出しを 8 回やり直す道
            d.data = bytearray()
            d.blocks = 0
        h = track(m)
        m.run(0x0006, until=None if fail else 0x1FE00, ax=0x0A)
        m.uc.hook_del(h)
        worst_boot = max(worst_boot, STACK - 10 - m.track_sp)
    print(f"      本体から呼ぶ入口の深さ: 初期化 1 {depth.get(0x0C, 0)}、初期化 2 {depth.get(0x0F, 0)}、"
          f"初期化 3 {depth.get(0x12, 0)}、ブート {worst_boot}（本体の INT 1Bh の 24 バイトを含む）")
    assert max(depth.values()) <= 34, depth
    assert worst_boot <= 50, worst_boot


def main():
    want = sys.argv[1:]
    tests = [(k, v) for k, v in globals().items() if k.startswith("test_") and (not want or any(w in k for w in want))]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print("ok  ", name)
        except (AssertionError, UcError) as e:
            failed += 1
            import traceback
            tb = traceback.extract_tb(e.__traceback__)[-1]
            print("FAIL", name, f"line {tb.lineno}", repr(e)[:300])
    print(f"{len(tests) - failed} / {len(tests)}")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
