# SPDX-License-Identifier: MIT
"""試験の模型: SASI のターゲット（コントローラとドライブ）、8237 のチャネル 0、8259 の組。

仕様書（docs/sasi_bios_spec.md 0.3 版）の 2 節と、SASI の規格の一般的なフェーズの流れ、8237 / 8259 のデータシートの
振る舞いだけから作った。決まっていない所は、試験に要る最小の振る舞いにしてある（QUESTIONS.md 参照）。
"""

REQ, ACK, BSY, MSG, CXD, IXO, INT = 0x80, 0x40, 0x20, 0x10, 0x08, 0x04, 0x01

PHASE_BITS = {
    "free": 0,
    "sel": BSY,
    "busy": BSY,
    "cmd": REQ | BSY | CXD,
    "dout": REQ | BSY,
    "din": REQ | BSY | IXO,
    "status": REQ | BSY | CXD | IXO,
    "msg": REQ | BSY | MSG | CXD | IXO,
}


class PIC:
    """マスタとスレーブの 8259。IR9（スレーブの IR1）だけを扱う。"""

    def __init__(self):
        self.imr = {0x02: 0x00, 0x0A: 0xFF}     # マスタの IR7（カスケード）は開いている前提、スレーブは閉じている
        self.isr = {0x00: 0, 0x08: 0}
        self.read_isr = {0x00: False, 0x08: False}
        self.irr9 = False
        self.eoi = {0x00: 0, 0x08: 0}
        self.delivered = 0

    def out(self, port, v):
        if port in (0x02, 0x0A):
            self.imr[port] = v
        elif port in (0x00, 0x08):
            if v & 0x10:                        # ICW1
                return
            if v == 0x20:                       # 非特定の EOI
                isr = self.isr[port]
                if isr:
                    self.isr[port] = isr & (isr - 1)
                self.eoi[port] += 1
            elif v & 0x18 == 0x08:              # OCW3
                if v & 0x02:
                    self.read_isr[port] = bool(v & 0x01)

    def inp(self, port):
        if port in (0x02, 0x0A):
            return self.imr[port]
        if port in (0x00, 0x08):
            if self.read_isr[port]:
                return self.isr[port]
            return 0x02 if (port == 0x08 and self.irr9) else 0
        return 0xFF

    def deliverable(self):
        return (self.irr9 and not self.imr[0x0A] & 0x02 and not self.imr[0x02] & 0x80
                and not self.isr[0x08] & 0x02 and not self.isr[0x00] & 0x80)

    def acknowledge(self):
        self.irr9 = False
        self.isr[0x08] |= 0x02
        self.isr[0x00] |= 0x80
        self.delivered += 1


class DMA:
    """8237 のチャネル 0（PC-98 の I/O 01h、03h、15h、17h、19h、27h）。"""

    def __init__(self, mem_read, mem_write):
        self.mem_read, self.mem_write = mem_read, mem_write
        self.addr = self.count = 0
        self.page = 0
        self.mode = None
        self.mask = True
        self.ff = 0
        self.modes = []

    def out(self, port, v):
        if port == 0x01:
            self.addr = (self.addr & 0xFF00) | v if not self.ff else (self.addr & 0xFF) | v << 8
            self.ff ^= 1
        elif port == 0x03:
            self.count = (self.count & 0xFF00) | v if not self.ff else (self.count & 0xFF) | v << 8
            self.ff ^= 1
        elif port == 0x27:
            self.page = v
        elif port == 0x15:
            if v & 3 == 0:
                self.mask = bool(v & 4)
        elif port == 0x17:
            if v & 3 == 0:
                self.mode = v
                self.modes.append(v)
        elif port == 0x19:
            self.ff = 0

    def inp(self, port):
        if port in (0x01, 0x03):
            v = self.addr if port == 0x01 else self.count
            b = v >> 8 if self.ff else v & 0xFF
            self.ff ^= 1
            return b
        return 0xFF

    def kind(self):
        return {0: "verify", 1: "write", 2: "read"}.get((self.mode or 0) >> 2 & 3)

    def cycle(self, to_memory, value=0):
        """1 バイトの転送。チャネルが止まっていれば None。to_memory: I/O → メモリ。"""
        if self.mask or self.mode is None:
            return None
        k = self.kind()
        assert self.mode & 0xC0 == 0x40, f"シングル転送でない: {self.mode:02X}"
        phys = (self.page & 0x0F) << 16 | self.addr
        if to_memory:
            assert k in ("write", "verify"), f"向きが違う: {self.mode:02X}"
            if k == "write":
                self.mem_write(phys, bytes((value,)))
            out = 0
        else:
            assert k in ("read", "verify"), f"向きが違う: {self.mode:02X}"
            out = self.mem_read(phys, 1)[0] if k == "read" else 0
        self.addr = (self.addr + 1) & 0xFFFF
        if self.count == 0:
            self.count = 0xFFFF
            self.mask = True                    # 終わり（自動初期化なし）
        else:
            self.count -= 1
        return out


class Drive:
    def __init__(self, cap_code=1, sec512=False, blocks=None):
        self.cap_code = cap_code
        self.sec512 = sec512
        self.seclen = 512 if sec512 else 256
        cyl, heads = {0: (153, 4), 1: (310, 4), 2: (310, 6), 3: (310, 8),
                      4: (615, 4), 5: (615, 6), 6: (615, 8)}[cap_code]
        self.spt = 17 if sec512 else 33
        self.blocks = blocks if blocks is not None else cyl * heads * self.spt
        self.data = bytearray((i * 7 + 3) & 0xFF for i in range(self.blocks * self.seclen))
        self.not_ready = 0                      # TEST UNIT READY が「準備中」で返る回数
        self.corrected = set()                  # 読むと訂正したデータエラーを返す LBA
        self.seek_delay = 0                     # SEEK の完了までの 055Fh の読み出しの回数
        self.alt = []

    def sector(self, lba, n=1):
        return bytes(self.data[lba * self.seclen:(lba + n) * self.seclen])


class Target:
    """SASI のターゲット ID 0 のコントローラと、LUN 0、1 のドライブ。"""

    def __init__(self, drives, present=True, verify_bit=False):
        self.drives = drives                    # {lun: Drive}
        self.present = present
        self.verify_bit = verify_bit
        self.dma = None
        self.ctrl = 0
        self.latch = 0
        self.phase = "free"
        self.cdb = []
        self.buf = bytearray()
        self.want = 0
        self.status = 0
        self.msg_sent = False
        self.sense = {0: [0, 0, 0, 0], 1: [0, 0, 0, 0]}
        self.params = {}
        self.log = []                           # (命令, LUN, LBA, ブロック数, 制御)
        self.ctrl_writes = []
        self.resets = 0
        self.select_ok = True
        self.busy_left = 0
        self.after_busy = None
        self.on_change = lambda: None

    # ---- バスの見え方
    def dtype(self):
        v = 0
        for lun, shift, bit in ((0, 3, 0x80), (1, 0, 0x40)):
            d = self.drives.get(lun)
            v |= (d.cap_code if d else 7) << shift
            if d and d.sec512:
                v |= bit
        return v

    def int_line(self):
        return self.present and self.phase == "status" and bool(self.ctrl & 0x01)

    def read_ctrl(self):
        if not self.present:
            return 0xFF
        if not self.ctrl & 0x40:
            return self.dtype()
        v = PHASE_BITS[self.phase] | (0x02 if self.verify_bit else 0)
        if self.int_line():
            v |= INT
        return v

    def write_ctrl(self, v):
        old, self.ctrl = self.ctrl, v
        self.ctrl_writes.append(v)
        if not self.present:
            return
        if old & 0x08 and not v & 0x08:
            self.resets += 1
            self.phase = "free"
            self.params = {}
        if v & 0x20 and self.phase == "free" and self.latch & 0x01 and self.select_ok:
            self.phase = "sel"
        elif not v & 0x20 and self.phase == "sel":
            self.phase = "cmd"
            self.cdb = []
        self.pump()
        self.on_change()

    def write_data(self, v):
        self.latch = v
        if not self.present:
            return
        if self.phase == "cmd":
            self.cdb.append(v)
            if len(self.cdb) == 6:
                self.command()
        elif self.phase == "dout":
            self.buf.append(v)
            if len(self.buf) == self.want:
                self.data_out_done()
        self.on_change()

    def read_data(self):
        if not self.present:
            return 0xFF
        v = 0xFF
        if self.phase == "din":
            v = self.buf[self.pos]
            self.pos += 1
            if self.pos == len(self.buf):
                self.data_in_done()
        elif self.phase == "status":
            v = self.status
            self.phase = "msg"
        elif self.phase == "msg":
            v = 0
            self.phase = "free"
        self.on_change()
        return v

    # ---- DMA
    def pump(self):
        if not self.ctrl & 0x02 or self.dma is None:
            return
        if self.phase == "din":
            while self.phase == "din":
                if self.dma.cycle(True, self.buf[self.pos]) is None:
                    return
                self.pos += 1
                if self.pos == len(self.buf):
                    self.data_in_done()
        elif self.phase == "dout" and self.dma_out:
            while self.phase == "dout":
                b = self.dma.cycle(False)
                if b is None:
                    return
                self.buf.append(b)
                if len(self.buf) == self.want:
                    self.data_out_done()

    # ---- 命令
    def finish(self, lun, ok, code=0, lba=None):
        if ok:
            self.status = lun << 5
        else:
            self.status = 0x02 | lun << 5
            if lba is None:
                self.sense[lun] = [code, lun << 5, 0, 0]
            else:
                self.sense[lun] = [code | 0x80, lun << 5 | (lba >> 16 & 0x1F), lba >> 8 & 0xFF, lba & 0xFF]
        self.phase = "status"

    def geometry_ok(self, lun, lba, n, seek=False):
        d = self.drives[lun]
        p = self.params.get(lun)
        if not seek and lba + n > d.blocks:
            return False
        if p is not None:
            heads = p[3] + 1
            spt = 17 if p[8] else 33
            limit = p[4] << 8 | p[5]
            if (lba + n - 1) // (heads * spt) > limit:
                return False
        return True

    def command(self):
        c = self.cdb
        op, lun = c[0], c[1] >> 5
        lba = (c[1] & 0x1F) << 16 | c[2] << 8 | c[3]
        n = c[4]
        self.log.append((op, lun, lba, n, c[5]))
        self.cur = (op, lun, lba, n)
        d = self.drives.get(lun)
        if op == 0x03:
            self.buf = bytearray(self.sense[lun])
            self.sense[lun] = [0, 0, 0, 0]
            self.pos = 0
            self.phase = "din"
            self.after = lambda: self.finish(lun, True)
            return
        if op == 0xC2:
            self.start_out(10, False)
            return
        if op == 0x0E:
            self.start_out(4, True)
            return
        if d is None:
            if op in (0x00, 0x01, 0x04, 0x05, 0x06, 0x08, 0x0A, 0x0B):
                return self.finish(lun, False, 0x04)
            return self.finish(lun, False, 0x20)
        if op == 0x00:
            if d.not_ready:
                d.not_ready -= 1
                return self.finish(lun, False, 0x04)
            return self.finish(lun, True)
        if op == 0x01:
            return self.finish(lun, True)
        if op in (0x08, 0x0A, 0x05):
            count = n or 256
            if not self.geometry_ok(lun, lba, count):
                return self.finish(lun, False, 0x21, lba)
            if op == 0x05:
                return self.finish(lun, True)
            if op == 0x08:
                stop = None
                for i in range(count):
                    if lba + i in d.corrected:
                        d.corrected.discard(lba + i)
                        stop = i
                        break
                k = count if stop is None else stop + 1
                self.buf = bytearray(d.sector(lba, k))
                self.pos = 0
                self.phase = "din"
                if stop is None:
                    self.after = lambda: self.finish(lun, True)
                else:
                    self.after = lambda: self.finish(lun, False, 0x18, lba + stop)
                self.pump()
                return
            self.start_out(count * d.seclen, True)
            return
        if op == 0x0B:                          # 退避位置はドライブの容量の外なので、シリンダの上限だけを見る
            if not self.geometry_ok(lun, lba, 1, seek=True):
                return self.finish(lun, False, 0x21, lba)
            if d.seek_delay:
                self.phase = "busy"
                self.busy_left = d.seek_delay
                self.after_busy = lambda: self.finish(lun, True)
                return
            return self.finish(lun, True)
        if op == 0x06:
            if not self.geometry_ok(lun, lba, 1):
                return self.finish(lun, False, 0x21, lba)
            d.data[lba * d.seclen:(lba + d.spt) * d.seclen] = b"\xE5" * (d.spt * d.seclen)
            return self.finish(lun, True)
        if op == 0x04:
            d.data[:] = b"\xE5" * len(d.data)
            return self.finish(lun, True)
        return self.finish(lun, False, 0x20)

    def start_out(self, want, by_dma):
        self.buf = bytearray()
        self.want = want
        self.dma_out = by_dma
        self.phase = "dout"
        self.pump()

    def data_out_done(self):
        op, lun, lba, n = self.cur
        if op == 0xC2:
            self.params[lun] = bytes(self.buf)
            return self.finish(lun, True)
        if op == 0x0E:
            self.drives.get(lun) and self.drives[lun].alt.append((lba, bytes(self.buf)))
            return self.finish(lun, True)
        d = self.drives[lun]
        d.data[lba * d.seclen:lba * d.seclen + len(self.buf)] = self.buf
        return self.finish(lun, True)

    def data_in_done(self):
        self.after()

    def poll(self):
        """055Fh を BIOS が読むたびに呼ぶ（時間の経過の代わり）。"""
        if self.phase == "busy":
            self.busy_left -= 1
            if self.busy_left <= 0:
                self.after_busy()
                self.on_change()
