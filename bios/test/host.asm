; SPDX-License-Identifier: MIT
; Copyright (c) 2026 geek-masa
; テスト用の本体 BIOS のモック。1000:0000 に置く。
; INT 1Bh ハンドラと拡張 ROM の呼び出し手順は、docs/sasibios_spec.md の 4.1 節と 6.1 節（本体の呼び出しの約束）に合わせる。
; test_bios.py が各ルーチンの先頭から実行し、HLT で止める。

        cpu     186
        bits    16
        org     0

XROM_PTR        equ 04ACh
XROM_ID         equ 04D0h

        jmp     near setup              ; 0000h
        jmp     near call_entry         ; 0003h: AX = 呼ぶエントリ（0Ch / 0Fh / 12h）
        jmp     near call_boot          ; 0006h: AL = ブート装置のコード
        jmp     near do_int1b           ; 0009h: レジスタは test_bios.py が設定する

setup:
        xor     ax, ax
        mov     ds, ax
        mov     word [1Bh * 4], int1b_main
        mov     word [1Bh * 4 + 2], cs
        hlt

; D0000h〜DFFFFh の各区画について、エントリ AX を呼ぶ（AX = 0Ch なら未登録の区画だけ）
call_entry:
        xor     bx, bx
        mov     ds, bx
        mov     [XROM_PTR], ax
        mov     word [XROM_PTR + 2], 0D000h
        mov     bx, XROM_ID
        mov     cx, 16
.slot:
        cmp     word [XROM_PTR], 0Ch
        jne     .claimed
        push    es
        mov     es, [XROM_PTR + 2]
        cmp     word [es:09h], 0AA55h
        pop     es
        jne     .next
        cmp     byte [bx], 0
        jne     .next
        jmp     .call
.claimed:
        test    byte [bx], 40h
        jz      .next
.call:
        push    ax
        push    bx
        push    cx
        push    ds
        push    es
        call    far [XROM_PTR]
        pop     es
        pop     ds
        pop     cx
        pop     bx
        pop     ax
        mov     [XROM_PTR], ax          ; 呼ばれた側が 04ACh を使ってもよいように
.next:
        inc     bx
        add     word [XROM_PTR + 2], 100h
        loop    .slot
        hlt

call_boot:
        mov     ah, al
        xor     bx, bx
        mov     ds, bx
        mov     word [XROM_PTR], 15h
        mov     word [XROM_PTR + 2], 0D000h
        mov     bx, XROM_ID
        mov     cx, 16
.slot:
        test    byte [bx], 40h
        jz      .next
        push    ax
        push    bx
        push    cx
        mov     al, ah
        call    far [XROM_PTR]
        xor     bx, bx
        mov     ds, bx
        pop     cx
        pop     bx
        pop     ax
        mov     word [XROM_PTR], 15h
.next:
        inc     bx
        add     word [XROM_PTR + 2], 100h
        loop    .slot
        hlt                             ; どの ROM も起動しなかった

do_int1b:
        int     1Bh
        hlt

; ES2 の本体 BIOS（F800:7282h）と同じ形
int1b_main:
        sti
        cld
        push    ds
        push    si
        push    di
        push    es
        push    bp
        push    dx
        push    cx
        push    bx
        push    ax
        mov     bp, sp
        xor     bx, bx
        mov     ds, bx
        mov     bl, al
        shr     bl, 4
        mov     al, [04B0h + bx]
        test    al, al
        jz      .none
        mov     word [XROM_PTR], 0018h
        mov     byte [XROM_PTR + 2], 0
        mov     [XROM_PTR + 3], al
        jmp     far [XROM_PTR]
.none:
        mov     byte [bp + 1], 0E0h     ; 本体内の DISK BIOS の代わり: 目印の値を返す
        or      byte [bp + 16h], 01h
        pop     ax
        pop     bx
        pop     cx
        pop     dx
        pop     bp
        pop     es
        pop     di
        pop     si
        pop     ds
        iret
