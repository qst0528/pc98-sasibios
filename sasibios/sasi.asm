; SPDX-License-Identifier: MIT
; PC-9801 ノーマルモード用の SASI 固定ディスク BIOS（クリーンルームの実装）
;
; 根拠は docs/sasibios_spec.md（仕様書 0.14 版）の 0〜9 節と付録 C だけ。付録 A は不採用、付録 B は入れていない。
; 注の「Qn」は QUESTIONS.md の項目番号で、答えの出た項目は仕様書の付録 D にその節がある。
;
; 作り方: python3 sasibios/mkrom.py（nasm で組み立て、4 KB の和を 00h に合わせて sasibios/sasi.rom を書く）。
;         sasi.rom もコミットする。このファイルを直したら、そのたびに作り直して一緒にコミットし、
;         コミットメッセージに組み立てに使った nasm のバージョン（nasm -v）を書く
; 試験:   python3 bios/test/sasi/test_sasi.py、python3 bios/test/sasi/test_detail.py（調査側が用意したもの）
;
; スタック（仕様書 2.6）: INT 1Bh は本体の 9 語の下で割り込みハンドラを含めて 28 バイト、初期化 1〜3 は 34 バイト、
; ブートは 50 バイトまで。そのため次の作りにした。
; - 局所変数を持たない。ユニットは枠の AL の bit0、制御バイトは枠の AH の bit5、ユニットの型と宛先は 05E8h / 05ECh の
;   ポインタの指す表（付録 C の 10 バイト型の記述は +08h に印）から、そのつど求める（marker）。
; - CDB はスタックに置かず、1 バイトずつ作りながら送る。6 バイトの CDB の 1〜4 バイト目は 0587h、0588h、0589h、0586h、
;   10 バイトの CDB の LBA は 0586h〜0589h（ビッグエンディアン）に置く（仕様書 2.6 の 0586h の使い方）。
; - エラーのあとの REQUEST SENSE は、入れ子に呼ばず、sasi_exec の中で続けて出す。センスは先頭 4 バイトを 0586h〜0589h に置き、
;   残りは読み捨てる。
; - 最後に sasi_exec を呼ぶ下回りは jmp で渡し、割り込みを待つところでの呼び出しの深さを 2 段までにする。
; - 初期化 2 は、INT 1Bh の枠の代わりに 1 語（AX の欄だけ）の小さな枠を積んで、機能の処理を直接呼ぶ。
;   初期化（03h）の処理には jmp で入り、終わりで SP = BP なら初期化 2 の続きへ戻る。
; - 初期化（03h）の間は、出力にしか使わない枠の AH の欄の空きビットを作業に使う（bit0 = 元の AL の bit0、bit1〜4 = 回数、
;   bit4 = 81h の次の候補、bit6 = 見分けの間）。INT 1Bh の入口で、仕様書 6.2 で未使用の bit4 と bit6 を 0 にしておく。

        cpu     186
        bits    16
        org     0

; I/O
P_DATA          equ     80h
P_CTRL          equ     82h
PIC_M           equ     00h
PIC_S           equ     08h
PIC_S_IMR       equ     0Ah
DMA_ADDR0       equ     01h             ; チャネル c のアドレスは 01h + 4c、カウントは 03h + 4c（仕様書 2.3）
DMA_COUNT0      equ     03h
DMA_MASK        equ     15h
DMA_MODE        equ     17h
DMA_CLRFF       equ     19h
SYS_PORT_B      equ     31h

; 82h の書き込み
C_NRDSW         equ     40h
C_SEL           equ     20h
C_RST           equ     08h
M_PIO           equ     0C0h            ; CHEN、NRDSW

; 82h の読み出し（NRDSW = 1）
S_REQ           equ     80h
S_ACK           equ     40h
S_BSY           equ     20h
S_MSG           equ     10h
S_CXD           equ     08h
S_IXO           equ     04h
S_INT           equ     01h

HOST_ID_BIT     equ     80h             ; 付録 C: 選択のときホストの ID 7 のビットも出す
SHORT_SEL       equ     8192            ; 付録 C の 1: 見分けのときの選択の時間切れ（Q37）
; 完了の割り込みの待ち（仕様書 4.6）: 055Fh を読む回数の 65536 倍の単位。1 回は V30 8 MHz で約 28 クロック（約 0.23 秒 / 65536 回）
WAIT_NORMAL     equ     41              ; 約 9.4 秒
WAIT_FORMAT     equ     41 * 60         ; 約 9.4 分（FORMAT DRIVE）

; sasi_exec の SI = 印 << 8 | 命令
E_IN            equ     0100h           ; PIO で読む（最大 DH バイト）
E_OUT           equ     0200h           ; PIO で送る（C2h の 10 バイト。ES:DI の 8 バイト、セクタ長、00h）
E_TEN           equ     0400h           ; 10 バイトの CDB
E_INTE          equ     1000h           ; 82h の INTE、完了を割り込みで待つ
E_DMAE          equ     2000h           ; 82h の DMAE
E_C2_512        equ     4000h           ; C2h のデータのバイト 8（512 バイト）。PIO で読む命令では「バイト 4〜5 が 0 でない」の印
E_SLOW          equ     8000h           ; 完了まで約 65536 回より長くかかった

; システム共通域（セグメント 0）
NEW_SENSE       equ     0480h
DMA_SEL         equ     0484h           ; bit4 = 内蔵 SASI あり、bit7〜6 = その DMA チャネル（仕様書 2.3、付録 B）
DISK_CLOSE      equ     0481h
XROM_PTR        equ     04ACh
DISK_XROM0      equ     04B0h
DISK_XROM8      equ     04B8h
SYS_TYPE        equ     0500h
DISK_EQUIP      equ     055Dh
DISK_DONE       equ     055Fh
DISK_BOOT       equ     0584h
DISK_STATUS     equ     0585h
DISK_SENSE      equ     0586h
PARAM_PTR       equ     05E8h           ; ユニット 0 は 05E8h、ユニット 1 は 05ECh

; 枠（仕様書 6.1）
F_AL            equ     0
F_AH            equ     1
F_BX            equ     2
F_BH            equ     3
F_CX            equ     4
F_DX            equ     6
F_DL            equ     6
F_DH            equ     7
F_BP            equ     8
F_ES            equ     0Ah
F_FLAGS         equ     16h
; 初期化（03h）の間の、枠の AH の欄の作業用のビット
W_ORIG0         equ     01h             ; 元の AL の bit0
W_COUNT         equ     1Eh             ; 回数（bit1〜4。見分けの間は bit1〜3 だけ使う）
W_UA            equ     0Eh             ; 見分けの間の UNIT ATTENTION の回数（bit1〜3）
W_ID1           equ     10h             ; 81h の次の候補（ID 1 の LUN 0）を調べている
W_PROBE         equ     40h             ; 見分けの間（10 バイト型として扱う）

; 10 バイト型の記述（30 バイト、3.2 の並び）
X_MARK          equ     08h             ; bit7 = 10 バイト型、bit4 = 512 バイト、bit0 = 81h を ID 1 の LUN 0 で受ける
X_SPT           equ     09h
XM_X            equ     80h
XM_512          equ     10h
XM_ID1          equ     01h

; DX:AX と 32 ビットの定数を符号なしで比べて飛ぶ
%macro jbe32 2
        cmp     dx, (%1) >> 16
        jb      %2
        ja      %%no
        cmp     ax, (%1) & 0FFFFh
        jbe     %2
%%no:
%endmacro
%macro jae32 2
        cmp     dx, (%1) >> 16
        ja      %2
        jb      %%no
        cmp     ax, (%1) & 0FFFFh
        jae     %2
%%no:
%endmacro

; ---------------------------------------------------------------- ヘッダ（仕様書 1.1、4.1）
        db      0CBh, 90h, 90h
        db      0CBh, 90h, 90h
        db      0CBh, 90h, 90h
        db      55h, 0AAh               ; 0009h
        db      29h                     ; 000Bh
        jmp     strict near init1       ; 000Ch
        jmp     strict near init2       ; 000Fh
        retf                            ; 0012h（初期化 3 は何もしない）
        nop
        nop
        jmp     strict near boot        ; 0015h
        jmp     strict near int1b       ; 0018h

; ---------------------------------------------------------------- 初期化 1（仕様書 4.2）
init1:
        push    ax
        push    ds
        xor     ax, ax
        mov     ds, ax
        mov     byte [bx], 0E9h
        cmp     byte [DISK_XROM0], 0
        jne     .ret
        mov     ax, cs
        mov     [DISK_XROM0], ah
        mov     [DISK_XROM8], ah
        or      byte [NEW_SENSE], 80h
.ret:
        pop     ds
        pop     ax
        retf

; ---------------------------------------------------------------- 初期化 2（仕様書 4.3）
; レジスタは全部保存する（18 バイト）。その下に 1 語の枠（AX の欄）を積み、機能の処理を直接呼ぶ。
init2:
        push    ax
        push    bx
        push    cx
        push    dx
        push    si
        push    di
        push    bp
        push    ds
        push    es
        xor     ax, ax
        mov     ds, ax
        mov     ax, cs
        cmp     [DISK_XROM0], ah
        jne     init2_out
        call    reset
        jc      init2_out
        cli
        mov     word [11h * 4], sasi_irq
        mov     [11h * 4 + 2], cs
        sti                             ; 仕様書 2.2（Q6）
        push    ax                      ; 1 語の枠
        mov     bp, sp
        mov     word [bp], 0E00h
        call    f_mode
        mov     word [bp], 0E01h
        call    f_mode
        in      al, PIC_S_IMR
        and     al, 0FDh
        out     PIC_S_IMR, al
        mov     word [bp], 0300h
        jmp     f_init                  ; 終わると SP = BP なので init2_cont へ戻る
init2_cont:
        ; 手順 5: 容量コード 001b の 6 バイト型のユニットにシリンダ 0 へのシーク。すぐ終われば 10 MB の別の表に替え、リトラクト（Q12）
        mov     word [bp], 0080h
.u10:
        call    marker
        test    al, XM_X
        jnz     .n10
        call    cap_code
        cmp     al, 1
        jne     .n10
        call    seek0
        test    ah, ah
        jnz     .r10
        call    ptr_addr
        mov     word [PARAM_PTR + bx], tbl_1x
        mov     [PARAM_PTR + 2 + bx], cs
.r10:
        mov     byte [bp + F_AH], 0Fh
        call    do_retract
        mov     byte [bp + F_AH], 00h
.n10:
        inc     byte [bp + F_AL]
        test    byte [bp + F_AL], 1
        jnz     .u10
        in      al, SYS_PORT_B          ; 手順 6
        test    al, 20h
        jnz     .frame
        and     byte [DISK_EQUIP], 0FCh
.frame:
        pop     ax
init2_out:
        pop     es
        pop     ds
        pop     bp
        pop     di
        pop     si
        pop     dx
        pop     cx
        pop     bx
        pop     ax
        retf

; ---------------------------------------------------------------- ブート（仕様書 5）
; 退避するのは BP、DS、ES と 04ACh〜04AFh だけ（50 バイトの上限のため。Q48）。
; [SS:SI] の 1 語: 下位 = DA/UA、上位 = bit7 が 512 バイト、bit3〜0 が残りの回数
boot:
        push    bp
        push    ds
        push    es
        xor     bx, bx
        mov     ds, bx
        mov     bx, cs
        cmp     [DISK_XROM0], bh
        jne     .ret
        sub     al, 0Ah
        cmp     al, 1
        ja      .ret
        mov     cl, al
        mov     ah, 1
        shl     ah, cl
        test    [DISK_EQUIP], ah
        jz      .ret
        or      al, 80h
        push    word [XROM_PTR]
        push    word [XROM_PTR + 2]
        xor     ah, ah
        push    ax
        mov     si, sp
        mov     ah, 84h
        mov     bx, 0100h
        int     1Bh
        cmp     bx, 0100h
        je      .s256
        or      byte [ss:si + 1], 80h
.s256:
        mov     al, [ss:si]
        mov     ah, 8Eh
        int     1Bh
.again:
        and     byte [ss:si + 1], 80h   ; 8 回から数え直す（仕様書 5。Q22）
        or      byte [ss:si + 1], 8
.try:
        mov     al, [ss:si]
        mov     ah, 07h
        int     1Bh
        jc      .next
        mov     ax, 1FE0h
        mov     bx, 0200h
        test    byte [ss:si + 1], 80h
        jz      .seg
        mov     ax, 1FC0h
        mov     bx, 0400h
.seg:
        mov     es, ax
        mov     al, [ss:si]
        mov     ah, 06h
        xor     cx, cx
        xor     dx, dx
        xor     bp, bp
        int     1Bh
        jc      .next
        cmp     word [es:4], 'IP'
        jne     .giveup
        cmp     word [es:6], 'L1'
        jne     .giveup
        mov     al, [ss:si]
        mov     [DISK_BOOT], al
        mov     word [XROM_PTR], 0
        mov     [XROM_PTR + 2], es
        call    far [XROM_PTR]
        xor     ax, ax
        mov     ds, ax
        mov     si, sp
        jmp     .again
.next:
        dec     byte [ss:si + 1]
        test    byte [ss:si + 1], 0Fh
        jnz     .try
.giveup:
        mov     al, [ss:si]
        mov     ah, 0Eh
        int     1Bh
        mov     al, [ss:si]
        mov     ah, 0Fh
        int     1Bh
        pop     ax
        pop     word [XROM_PTR + 2]
        pop     word [XROM_PTR]
.ret:
        pop     es
        pop     ds
        pop     bp
        retf

; ---------------------------------------------------------------- 割り込みハンドラ（仕様書 2.2）
sasi_irq:
        push    ax
        push    ds
        mov     al, 20h
        out     PIC_S, al
        mov     al, 0Bh
        out     PIC_S, al
        in      al, PIC_S
        test    al, al
        jnz     .slave
        mov     al, 20h
        out     PIC_M, al
.slave:
        in      al, P_CTRL
        mov     ah, al
        and     al, ~02h & 0FFh
        cmp     al, S_REQ | S_BSY | S_CXD | S_IXO | S_INT
        je      .done
        mov     al, ah
        and     al, S_REQ | S_ACK | S_BSY | S_MSG | S_CXD | S_INT
        cmp     al, S_REQ | S_BSY | S_INT
        jne     .ret
.done:
        xor     ax, ax
        mov     ds, ax
        or      byte [DISK_DONE], 01h
        mov     al, M_PIO
        out     P_CTRL, al
.ret:
        pop     ds
        pop     ax
        iret

; ---------------------------------------------------------------- INT 1Bh（仕様書 6）
int1b:
        and     byte [bp + F_AH], ~(W_ID1 | W_PROBE) & 0FFh     ; 6.2 で未使用の bit4、bit6（出力の欄なので消してよい）
        call    marker
        mov     bl, [bp + F_AH]
        and     bx, 0Fh
        shl     bx, 1
        test    al, XM_X
        jz      .call
        add     bx, xfunctab - functab
.call:
        call    [cs:functab + bx]
        mov     [bp + F_AH], ah
        and     byte [bp + F_FLAGS], 0FEh
        cmp     ah, 20h
        jb      .exit
        or      byte [bp + F_FLAGS], 01h
.exit:
        call    dma_mask                ; 仕様書 6.1（チャネル c）
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

functab:
        dw      f_seek, rw, f_none, f_init, f_sense, rw, rw, do_recal
        dw      f_altspec, f_none, f_none, f_altassign, f_none, f_format, f_mode, do_retract
xfunctab:                               ; 10 バイト型（付録 C）
        dw      f_none, rw, f_none, f_init, x_sense, rw, rw, f_none
        dw      f_altspec, f_none, f_none, f_none, f_none, f_none, x_mode, f_none

f_none:
        xor     ah, ah
        ret

; ---- 0h シーク（仕様書 6.3）。組 B のまま戻る（Q13）
f_seek:
        call    check_closed
        jc      seek_ret
        call    params_b
        call    get_table
        call    calc_lba
seek_send:                              ; DX:AX = LBA
        call    cdb6_lba
        mov     si, E_INTE | 0Bh
        call    sasi_exec
        test    ah, ah
        jnz     seek_ret
        test    al, al
        jz      seek_ret
        mov     ah, 10h
seek_ret:
        ret
; 初期化 2 の手順 5: シリンダ 0 へのシーク（INT 1Bh の 00h、AL = 80h / 81h、CX = DX = 0 と同じ）
seek0:
        call    check_closed
        jc      seek_ret
        call    params_b
        xor     ax, ax
        xor     dx, dx
        jmp     seek_send

; ---- 1h ベリファイ、5h 書き込み、6h 読み出し（仕様書 6.4、付録 C）
rw:
        call    check_closed
        jc      .ret
        mov     al, [bp + F_AH]
        and     al, 0Fh
        mov     bl, 44h
        cmp     al, 6
        je      .mode
        mov     bl, 48h
        cmp     al, 5
        je      .mode
        mov     bl, 40h
.mode:
        mov     al, bl
        call    dma_setup
        jc      .e20
        call    marker
        test    al, XM_X
        jnz     .x
        call    get_table
        call    calc_lba
        call    cdb6_lba
        call    count6
        mov     [DISK_SENSE], cl
        call    rw_op6
        mov     ah, (E_DMAE | E_INTE) >> 8
        mov     si, ax
        call    sasi_exec
.check:
        cmp     ah, 08h                 ; 訂正したデータエラー: 読み直すのは 6h だけ（Q19、Q44）
        jne     .ret
        mov     al, [bp + F_AH]
        and     al, 0Fh
        cmp     al, 6
        jne     .ret
        test    byte [bp + F_AH], 20h
        jz      .corr
        mov     ah, 0B8h
.ret:
        ret
.corr:                                  ; 残りを読み直す（仕様書 6.4 の 5。Q20）
        test    byte [DISK_SENSE], 80h
        jz      .e40
        call    is512
        mov     bl, al
        call    dma_chan                ; チャネル c のカウントの I/O（03h + 4c）を読む（仕様書 6.4 の 5）
        shl     al, 2
        add     al, DMA_COUNT0
        mov     dl, al
        xor     dh, dh
        out     DMA_CLRFF, al
        in      al, dx
        mov     cl, al
        in      al, dx
        mov     ch, al
        inc     cx                      ; 残りのバイト数（FFFFh なら 0 = 全部送った）
        jz      .ok08
        test    cl, cl
        jnz     .e40
        test    bl, bl
        jz      .blocks
        test    ch, 1
        jnz     .e40
        shr     ch, 1
.blocks:
        mov     cl, ch                  ; CL = 残りのブロック数
        mov     dl, [DISK_SENSE + 1]
        and     dl, 1Fh
        xor     dh, dh
        mov     ah, [DISK_SENSE + 2]
        mov     al, [DISK_SENSE + 3]
        add     ax, 1
        adc     dx, 0
        call    cdb6_lba
        mov     [DISK_SENSE], cl
        mov     si, E_DMAE | E_INTE | 08h
        call    sasi_exec
        cmp     ah, 08h
        je      .corr
        test    ah, ah
        jnz     .ret
.ok08:
        mov     ah, 08h
        ret
.e40:
        mov     ah, 40h
        ret
.e20:
        mov     ah, 20h
        ret
.x:                                     ; READ(10) 28h / WRITE(10) 2Ah。ベリファイも READ(10)（Q36）
        call    get_table
        call    calc_lba
        mov     [DISK_SENSE], dh
        mov     [DISK_SENSE + 1], dl
        mov     [DISK_SENSE + 2], ah
        mov     [DISK_SENSE + 3], al
        mov     si, E_DMAE | E_INTE | E_TEN | 28h
        mov     al, [bp + F_AH]
        and     al, 0Fh
        cmp     al, 5
        jne     .x10
        mov     si, E_DMAE | E_INTE | E_TEN | 2Ah
.x10:
        jmp     sasi_exec

; 6 バイトの読み書きの命令 → AL
rw_op6:
        mov     al, [bp + F_AH]
        and     al, 0Fh
        cmp     al, 5
        mov     al, 0Ah
        je      .ret
        mov     al, 08h
        jb      .verify
.ret:
        ret
.verify:                                ; 1h: 82h の bit1 が 1 なら VERIFY（仕様書 6.3）
        in      al, P_CTRL
        test    al, 02h
        mov     al, 08h
        jz      .ret
        mov     al, 05h
        ret

; 6 バイトの CDB のブロック数 → CL（仕様書 6.4 の 3）
count6:
        call    is512
        mov     cl, [bp + F_BH]
        test    al, al
        jz      .ret
        shr     cl, 1
        jnz     .ret
        mov     cl, 80h
.ret:
        ret

; ---- 3h 初期化（仕様書 4.4、付録 C の見分け方）
; INT 1Bh からは call で、初期化 2 からは jmp で入る。終わりで SP = BP なら初期化 2 の続きへ。
f_init:
        and     byte [DISK_EQUIP], 0FCh
        and     byte [DISK_DONE], 0FCh
        call    reset
        jc      .done
        mov     al, [bp + F_AL]         ; 元の AL の bit0 を AH の欄の bit0 に預け、ユニット 0 から始める
        and     al, 1
        and     byte [bp + F_AH], ~W_ORIG0 & 0FFh
        or      [bp + F_AH], al
        and     byte [bp + F_AL], 0FEh
.unit:
        call    probe_unit              ; 先の候補: ID 0 の LUN = ユニット
        jnc     .next
        test    byte [bp + F_AL], 1
        jz      .none
        or      byte [bp + F_AH], W_ID1 ; 81h の次の候補: ID 1 の LUN 0
        call    probe_unit
        jnc     .next
.none:                                  ; どちらも「なし」: 6 バイト型として先の候補を宛先にする
        call    set_6byte
.next:
        xor     byte [bp + F_AL], 1
        test    byte [bp + F_AL], 1
        jnz     .unit
.both:                                  ; 手順 4: 6 バイト型の宛先なら、見つからなかったユニットにも行う（Q17）
        call    marker
        test    al, XM_X
        jnz     .skip
        call    params_b
        call    do_recal
        call    do_retract
.skip:
        xor     byte [bp + F_AL], 1
        test    byte [bp + F_AL], 1
        jnz     .both
        mov     al, [bp + F_AH]
        and     al, W_ORIG0
        or      [bp + F_AL], al
.done:
        xor     ah, ah
        cmp     sp, bp
        jne     .ret
        jmp     init2_cont
.ret:
        ret

; いまの候補を調べる（付録 C の 1、2）。見つかれば CF = 0、「なし」なら CF = 1
probe_unit:
        mov     al, [bp + F_AH]
        and     al, ~W_COUNT & 0FFh
        or      al, W_PROBE | (3 << 1)  ; UNIT ATTENTION のやり直しは 2 回まで
        and     al, ~W_ID1 & 0FFh
        test    byte [bp + F_AH], W_ID1
        jz      .set
        or      al, W_ID1
.set:
        mov     [bp + F_AH], al
.again:
        call    read_capacity
        jnc     .x
        cmp     ah, 90h
        je      .none
        cmp     ch, 80h | 06h           ; UNIT ATTENTION
        jne     .notua
        sub     byte [bp + F_AH], 2
        test    byte [bp + F_AH], W_UA
        jnz     .again
        jmp     .none
.notua:
        cmp     ch, 20h                 ; SASI のセンス 20h: 6 バイト型
        jne     .none
        test    byte [bp + F_AH], W_ID1 ; ID 1 の 6 バイト型は扱わない（Q49）
        jnz     .none
        and     byte [bp + F_AH], ~(W_PROBE | W_COUNT) & 0FFh
        call    set_6byte
        call    cap_code
        cmp     al, 7
        je      .none
        call    params_b                ; 結果は 90h のときだけ見る（仕様書 4.4 の 3。Q16）
        cmp     ah, 90h
        je      .found
        mov     al, 15 << 1             ; TEST UNIT READY は 2 回ずつ数える: 30 回 / 2 回
        test    byte [SYS_TYPE], 80h
        jz      .tries
        mov     al, 1 << 1
.tries:
        or      [bp + F_AH], al
.pair:
        call    tur
        test    ah, ah
        jz      .ready
        cmp     ah, 90h
        je      .found
        call    delay
        call    tur
        test    ah, ah
        jz      .ready
        cmp     ah, 90h
        je      .found
        sub     byte [bp + F_AH], 2
        test    byte [bp + F_AH], W_COUNT
        jz      .found
        call    delay
        jmp     .pair
.ready:
        call    equip_on
.found:
        and     byte [bp + F_AH], ~(W_PROBE | W_COUNT) & 0FFh
        clc
        ret
.none:
        and     byte [bp + F_AH], ~(W_PROBE | W_COUNT) & 0FFh
        stc
        ret
.x:                                     ; 10 バイト型: DX:AX = ブロック数 N、CX = ブロック長
        mov     bl, 0
        cmp     cx, 512
        je      .x512
        cmp     cx, 256
        jne     .none
        jbe32   615 * 8 * 33, .x256t
        jmp     .none                   ; ブロック長 256 は 40 MB まで（付録 C、0.13 版）
.x256t:
        mov     bl, 2
        jae32   614 * 8 * 33, .xset
        inc     bl
        jae32   309 * 8 * 33, .xset
        inc     bl
        jae32   309 * 4 * 33, .xset
        inc     bl
        jae32   152 * 4 * 33, .xset
        jmp     .none                   ; 5 MB の型より小さい: 使わない（付録 C）
.x512:
        jbe32   65535 * 8 * 17, .xset
        mov     bl, 1
.xset:                                  ; BL = 記述の番号（0〜5）
        mov     al, XDESC_SIZE
        mul     bl
        add     ax, xdesc
        test    byte [bp + F_AH], W_ID1
        jz      .xptr
        add     ax, XDESC_SIZE * XDESC_COUNT
.xptr:
        push    ax
        call    ptr_addr
        pop     word [PARAM_PTR + bx]
        mov     [PARAM_PTR + 2 + bx], cs
        and     byte [bp + F_AH], ~(W_PROBE | W_COUNT) & 0FFh   ; 記述の表の印を見るため
        call    x_close                 ; ブロック長 512 は閉鎖、256 は開放（付録 C の 4、0.11 版。Q35）
        call    equip_on
        jmp     .found

; READ CAPACITY（25h）を PIO で出す（付録 C）。見分けの間（W_PROBE）は選択の時間切れが短い。
; 正常なら CF = 0、DX:AX = ブロック数 N、CX = ブロック長（使えない値なら 0）。エラーなら CF = 1、AH = 結果、CH = センスの種類
read_capacity:
        xor     ax, ax
        mov     [DISK_SENSE], ax
        mov     [DISK_SENSE + 2], ax
        mov     dh, 8
        mov     si, E_TEN | E_IN | 25h
        call    sasi_exec
        test    ah, ah
        jnz     .err
        cmp     dl, 8
        jne     .short
        mov     cx, di
        test    si, E_C2_512            ; ブロック長の上位 16 ビットが 0 でない
        jz      .n
        xor     cx, cx
.n:
        mov     dh, [DISK_SENSE]
        mov     dl, [DISK_SENSE + 1]
        mov     ah, [DISK_SENSE + 2]
        mov     al, [DISK_SENSE + 3]
        add     ax, 1
        adc     dx, 0
        clc
        ret
.short:
        mov     ax, 40FFh
        mov     ch, al
.err:
        stc
        ret

; ユニットを 6 バイト型にする: ポインタが 10 バイト型の記述を指していれば、容量コードの厚型の表に戻す
set_6byte:
        call    marker
        test    al, XM_X
        jz      .ret
        call    cap_code
        mov     bl, al
        xor     bh, bh
        shl     bx, 1
        mov     ax, [cs:thick_tab + bx]
        call    ptr_addr
        mov     [PARAM_PTR + bx], ax
        mov     [PARAM_PTR + 2 + bx], cs
.ret:
        ret

equip_on:
        call    unit_bit
        or      [DISK_EQUIP], al
        ret

; ---- 4h センス、84h 新センス（6 バイト型）
f_sense:
        call    tur
        test    ah, ah
        jnz     .ret
        call    is512
        call    get_table
        mov     ah, [es:di + 14h]
        test    byte [bp + F_AH], 80h
        jz      .ret
        mov     word [bp + F_BX], 0100h
        mov     byte [bp + F_DL], 33
        test    al, al
        jz      .geom
        mov     word [bp + F_BX], 0200h
        mov     byte [bp + F_DL], 17
.geom:
        mov     al, [es:di + 04h]
        mov     [bp + F_CX + 1], al
        mov     al, [es:di + 05h]
        mov     [bp + F_CX], al
        mov     al, [es:di + 03h]
        inc     al
        mov     [bp + F_DH], al
        mov     ah, [es:di + 1Ch]       ; 上位 4 ビットは 0（仕様書 3.2。Q14）
.ret:
        ret

; ---- 4h センス、84h 新センス（10 バイト型。付録 C）。容量は READ CAPACITY を出し直して得る
x_sense:
        call    tur
        test    ah, ah
        jnz     .ret
        test    byte [bp + F_AH], 80h
        jnz     .new
        call    get_table
        mov     ah, [es:di + 14h]
.ret:
        ret
.new:
        call    read_capacity
        jc      .ret
        mov     [bp + F_BX], cx
        call    get_table
        mov     bl, [es:di + X_SPT]
        mov     [bp + F_DL], bl
        mov     bh, [es:di + 18h]
        mov     [bp + F_DH], bh
        mov     cx, ax                  ; DX:CX = N
        mov     al, bh
        mul     bl
        mov     bx, ax                  ; BX = ヘッド数 × セクタ数
        mov     ax, dx
        xor     dx, dx
        div     bx
        test    ax, ax
        mov     ax, cx
        jnz     .cap                    ; 商が 65536 以上
        div     bx                      ; AX = シリンダ数
        mov     cx, [es:di + 04h]
        xchg    cl, ch                  ; CX = 頭打ちのシリンダ数 − 1
        cmp     ax, cx
        jbe     .cyl
.cap:
        mov     ax, [es:di + 04h]
        xchg    al, ah
        inc     ax
.cyl:
        dec     ax
        mov     [bp + F_CX], ax
        mov     ah, [es:di + 1Ch]
        ret

; ---- 7h リキャリブレート（仕様書 6.3）。AH は最後のパラメータ設定の結果
do_recal:
        call    params_b
        call    get_table
        mov     ax, 5
        call    seek_cyl
        call    cdb_noaddr
        mov     si, E_INTE | 01h
        call    sasi_exec
        jmp     params_a

; ---- Fh リトラクト
do_retract:
        call    params_b
        call    get_table
        mov     ax, [es:di + 0Eh]
        xchg    al, ah
        call    seek_cyl
        jmp     params_a

; ---- 8h 代替トラックの指定（10 バイト型も同じ。LBA は下位 21 ビット。Q42）
f_altspec:
        call    check_closed
        jc      .ret
        call    get_table
        call    calc_lba
        les     di, [bp + F_BP]
        mov     bl, al
        mov     al, dl
        and     al, 1Fh
        stosb
        mov     al, ah
        stosb
        mov     al, bl
        stosb
        xor     al, al
        stosb
        xor     ah, ah
.ret:
        ret

; ---- Bh 代替トラックの割り当て
f_altassign:
        call    check_closed
        jc      .ret
        mov     al, 48h
        call    dma_setup
        jc      .e20
        call    get_table
        call    calc_lba
        call    cdb6_lba
        mov     al, [bp + F_BH]
        mov     [DISK_SENSE], al
        mov     si, E_DMAE | E_INTE | 0Eh
        jmp     sasi_exec
.e20:
        mov     ah, 20h
.ret:
        ret

; ---- Dh フォーマット（仕様書 6.3。Q18）
f_format:
        call    check_closed
        jc      .ret
        test    byte [bp + F_AH], 80h
        jz      .go
        call    do_recal                ; 結果は見ない
.go:
        call    get_table
        call    calc_lba
        call    cdb6_lba
        mov     al, [bp + F_BH]
        mov     [DISK_SENSE], al
        mov     si, E_INTE | 06h
        test    byte [bp + F_AH], 80h
        jz      .exec
        mov     si, E_INTE | 04h
.exec:
        jmp     sasi_exec
.ret:
        ret

; ---- Eh モード設定（仕様書 3.4）
f_mode:
        call    cap_code
        mov     bl, al
        xor     bh, bh
        shl     bx, 1
        mov     di, thick_tab
        test    byte [bp + F_AH], 80h
        jz      .tab
        mov     di, thin_tab
.tab:
        mov     ax, [cs:di + bx]
        call    ptr_addr
        cmp     ax, tbl_1               ; 10 MB の別の表を選んでいたら保つ（Q11）
        jne     .set
        cmp     word [PARAM_PTR + bx], tbl_1x
        jne     .set
        mov     dx, cs
        cmp     [PARAM_PTR + 2 + bx], dx
        jne     .set
        mov     ax, tbl_1x
.set:
        mov     [PARAM_PTR + bx], ax
        mov     [PARAM_PTR + 2 + bx], cs
        test    byte [bp + F_AH], 80h
        jnz     open
        call    is512
        test    al, al
        jz      open
        call    unit_bit
        or      [DISK_CLOSE], al
        xor     ah, ah
        ret
; 10 バイト型の 0Eh / 8Eh: 表は替えず、ブロック長 512 なら 0Eh で閉鎖、8Eh で開放。256 なら開放（仕様書 3.4、0.11 版）
x_mode:
        test    byte [bp + F_AH], 80h
        jnz     open
; 10 バイト型のユニットを、ブロック長 512 なら閉鎖、256 なら開放にする
x_close:
        call    marker
        test    al, XM_512
        jz      open
        call    unit_bit
        or      [DISK_CLOSE], al
        xor     ah, ah
        ret
; 0481h のビットを 0 にする（開放）
open:
        call    unit_bit
        not     al
        and     [DISK_CLOSE], al
        xor     ah, ah
        ret

; ---------------------------------------------------------------- 下回り

; AL = いまのユニットのビット（1 / 2）
unit_bit:
        mov     al, [bp + F_AL]
        and     al, 1
        inc     al
        ret

; 0481h の閉鎖を確かめる（仕様書 3.4。Q27）。閉鎖中なら AH = 40h、CF = 1
check_closed:
        call    unit_bit
        test    [DISK_CLOSE], al
        jz      .ret
        mov     ah, 40h
        stc
.ret:
        ret

; BX = いまのユニットのポインタの番地 − 05E8h（0 / 4）
ptr_addr:
        mov     bl, [bp + F_AL]
        and     bx, 1
        shl     bx, 2
        ret

; ES:DI = いまのユニットのパラメータ表
get_table:
        call    ptr_addr
        les     di, [PARAM_PTR + bx]
        ret

; AL = いまのユニットの印（10 バイト型なら記述の +08h、6 バイト型なら 0）。見分けの間は 10 バイト型として扱う。BX を壊す
marker:
        test    byte [bp + F_AH], W_PROBE
        jz      .tbl
        mov     al, XM_X
        test    byte [bp + F_AH], W_ID1
        jz      .ret
        or      al, XM_ID1
        ret
.tbl:
        call    ptr_addr
        push    es
        les     bx, [PARAM_PTR + bx]
        mov     al, [es:bx + X_MARK]
        pop     es
        test    al, XM_X
        jnz     .ret
        xor     al, al
.ret:
        ret

; AL = LUN × 20h（付録 C の宛先）。BX を壊す
get_lun:
        call    marker
        test    al, XM_ID1
        mov     al, 0
        jnz     .ret
        mov     al, [bp + F_AL]
        and     al, 1
        ror     al, 3
.ret:
        ret

; AL = 1 なら 512 バイトのセクタ（10 バイト型は記述、6 バイト型は 82h の NRDSW = 0）。BX を壊す
is512:
        call    marker
        test    al, XM_X
        jz      .dip
        and     al, XM_512
        jz      .ret
        mov     al, 1
.ret:
        ret
.dip:
        call    read_dtype
        test    byte [bp + F_AL], 1
        jz      .u0
        shl     al, 1
.u0:
        rol     al, 1
        and     al, 1
        ret

; 82h（NRDSW = 0）を読む（00h を書いて読み、40h に戻す。仕様書 2.1。Q8）
read_dtype:
        mov     al, 00h
        out     P_CTRL, al
        in      al, P_CTRL
        mov     ah, al
        mov     al, C_NRDSW
        out     P_CTRL, al
        mov     al, ah
        ret

; AL = いまのユニットの容量コード（0〜7）
cap_code:
        call    read_dtype
        test    byte [bp + F_AL], 1
        jnz     .u1
        shr     al, 3
.u1:
        and     al, 07h
        ret

; アドレスを持たない 6 バイトの CDB（仕様書 2.5。Q15）
cdb_noaddr:
        call    get_lun
        mov     [DISK_SENSE + 1], al
        xor     ax, ax
        mov     [DISK_SENSE + 2], ax
        mov     [DISK_SENSE], al
        ret

; 6 バイトの CDB の 1〜4 バイト目: LUN と DX:AX の LBA（21 ビット）、ブロック数 0。BX を壊す
cdb6_lba:
        mov     [DISK_SENSE + 3], al
        mov     [DISK_SENSE + 2], ah
        mov     byte [DISK_SENSE], 0
        call    get_lun
        and     dl, 1Fh
        or      al, dl
        mov     [DISK_SENSE + 1], al
        ret

; TEST UNIT READY
tur:
        call    cdb_noaddr
        mov     si, E_INTE | 00h
        jmp     sasi_exec

; 枠の AL、CX、DX から LBA を計算する（仕様書 3.3、付録 C）。ES:DI = 表。DX:AX = LBA。BX、CX、SI を壊す
; 6 バイト型は 21 ビットに切る。10 バイト型のリニア指定は DX:CX の 32 ビット
calc_lba:
        call    marker
        mov     cl, al                  ; CL の bit7 = 10 バイト型
        mov     ch, 33
        test    cl, XM_X
        jz      .six
        mov     ch, [es:di + X_SPT]
        jmp     .s
.six:
        call    is512
        test    al, al
        jz      .s
        mov     ch, 17
.s:
        mov     ax, [bp + F_CX]
        test    byte [bp + F_AL], 80h
        jnz     .chs
        mov     dx, [bp + F_DX]
        jmp     .cut
.chs:                                   ; (CX × H + DH) × S + DL を 32 ビットで（Q10）
        xor     bh, bh
        mov     bl, [es:di + 18h]
        mul     bx
        mov     bl, [bp + F_DH]
        add     ax, bx
        adc     dx, 0
        mov     bl, ch
        mov     si, dx
        mul     bx                      ; 下位 16 ビット × S
        xchg    ax, si
        push    dx
        mul     bx                      ; 上位 16 ビット × S
        pop     dx
        add     dx, ax
        mov     ax, si
        mov     bl, [bp + F_DL]
        add     ax, bx
        adc     dx, 0
.cut:
        test    cl, XM_X
        jnz     .ret
        and     dx, 001Fh
.ret:
        ret

; DMA のチャネル c を設定する（仕様書 2.3、6.4）。AL = モード（44h / 48h / 40h）。64 KB の境界をまたぐなら CF = 1（何もしない）
dma_setup:
        mov     bl, al
        mov     ax, [bp + F_ES]
        mov     dx, ax
        shl     ax, 4
        shr     dx, 12
        add     ax, [bp + F_BP]
        adc     dl, 0
        and     dl, 0Fh
        mov     cx, [bp + F_BX]
        dec     cx
        mov     si, ax
        add     si, cx
        jc      .ret
        mov     si, ax                  ; SI = アドレス、BH = ページ、CX = カウント
        mov     bh, dl
        call    dma_chan
        or      bl, al                  ; BL = モード + c
        or      al, 04h
        out     DMA_MASK, al            ; チャネル c にマスク
        out     DMA_CLRFF, al
        mov     al, bl
        out     DMA_MODE, al
        mov     dl, bl
        and     dx, 3
        mov     di, dx
        shl     dl, 2
        add     dl, DMA_ADDR0
        mov     ax, si
        out     dx, al
        mov     al, ah
        out     dx, al
        add     dl, DMA_COUNT0 - DMA_ADDR0
        mov     al, cl
        out     dx, al
        mov     al, ch
        out     dx, al
        mov     dl, [cs:dma_page + di]
        mov     al, bh
        out     dx, al
        mov     al, bl
        and     al, 3
        out     DMA_MASK, al            ; チャネル c のマスクを外す
        clc
.ret:
        ret

; AL = 使う DMA のチャネル c（0000:0484h の bit4 が 1 なら bit7〜6、0 なら 0。仕様書 2.3、付録 B）
dma_chan:
        mov     al, [DMA_SEL]
        test    al, 10h
        jz      .zero
        rol     al, 2
        and     al, 3
        ret
.zero:
        xor     al, al
        ret

; DMA のチャネル c にマスクを掛ける（15h に 04h + c）
dma_mask:
        call    dma_chan
        or      al, 04h
        out     DMA_MASK, al
        ret

; チャネルごとのページの I/O（c = 0〜3）
dma_page:
        db      27h, 21h, 23h, 25h

; パラメータ設定（C2h）の組 B / 組 A。AH = 結果
params_b:
        call    get_table
        add     di, 0Ah
        jmp     params
params_a:
        call    get_table
params:                                 ; ES:DI = 組の 8 バイト
        call    is512
        mov     si, E_OUT | 0C2h
        test    al, al
        jz      .n
        or      si, E_C2_512
.n:
        call    cdb_noaddr
        jmp     sasi_exec

; 組 B を設定したあと、シリンダ AX へシークする（リキャリブレート、リトラクト）。
; LBA = シリンダ × H（表の +18h）× S（仕様書 3.3 の「シリンダだけのシーク」。Q9）。ES:DI = 表
seek_cyl:
        xor     bh, bh
        mov     bl, [es:di + 18h]
        mul     bx
        push    ax
        call    is512
        mov     bx, 33
        test    al, al
        jz      .s
        mov     bx, 17
.s:
        pop     ax
        mul     bx
        call    cdb6_lba
        mov     si, E_INTE | 0Bh
        jmp     sasi_exec

; コントローラのリセット（仕様書 2.4）。コントローラがなければ CF = 1
reset:
        mov     al, C_RST
        out     P_CTRL, al
        mov     cx, 40
        loop    $
        mov     al, 00h
        out     P_CTRL, al
        mov     al, C_NRDSW
        out     P_CTRL, al
        in      al, P_CTRL
        test    al, S_BSY
        jz      .ret
        stc
.ret:
        ret

; 初期化の TEST UNIT READY の間の待ち（仕様書 4.6。V30 8 MHz で約 0.3 秒。Q24）
delay:
        mov     ah, 3
.outer:
        xor     cx, cx
.inner:
        loop    .inner
        dec     ah
        jnz     .outer
        ret

; 82h（NRDSW = 1）を、(値 AND BH) = BL になるまで待つ。65536 回で時間切れ（CF = 1）。AL、CX を壊す
wait_bus:
        xor     cx, cx
.loop:
        in      al, P_CTRL
        and     al, bh
        cmp     al, bl
        je      .ret                    ; 等しければ CF = 0
        loop    .loop
        stc
.ret:
        ret

; REQ = 1、BSY = 1 を待つ。AL = 82h の値。時間切れなら CF = 1。CX を壊す
wait_req:
        xor     cx, cx
.loop:
        in      al, P_CTRL
        test    al, S_REQ
        jz      .next
        test    al, S_BSY
        jnz     .ret                    ; CF = 0
.next:
        loop    .loop
        stc
.ret:
        ret

; CDB の DL バイト目 → AL（BX = LUN と制御バイト、10 バイトの読み書きでは BH = ブロック数の下位、DH = 上位）
cdb_byte:
        mov     ax, si
        test    dl, dl
        jz      .ret                    ; 命令
        test    ah, E_TEN >> 8
        jnz     .ten
        cmp     dl, 5
        jne     .six
        mov     al, bh                  ; 制御バイト
.ret:
        ret
.six:                                   ; 1〜3 → 0587h〜0589h、4 → 0586h
        push    bx
        mov     bl, dl
        and     bx, 3
        mov     al, [DISK_SENSE + bx]
        pop     bx
        ret
.ten:
        cmp     dl, 1
        jne     .t2
        mov     al, bl                  ; LUN
        ret
.t2:
        cmp     dl, 6
        jae     .t6
        push    bx                      ; 2〜5 → 0586h〜0589h（LBA）
        mov     bl, dl
        xor     bh, bh
        mov     al, [DISK_SENSE - 2 + bx]
        pop     bx
        ret
.t6:
        cmp     al, 25h                 ; READ CAPACITY は 6〜9 バイト目が 0
        je      .zero
        cmp     dl, 7
        je      .hi
        cmp     dl, 8
        jne     .zero                   ; 6、9（制御バイトは 00h。Q43）
        mov     al, bh
        ret
.hi:
        mov     al, dh
        ret
.zero:
        xor     al, al
        ret

; 命令を 1 つ実行する（仕様書 2.4、付録 C の「ホストの手順の決まり」）。呼んだ側のレジスタは AX、BX、CX、DX、SI、DI とも壊れる。
; 入力: SI = 印 << 8 | 命令（E_xxx）。CDB は上の cdb_byte のとおり。PIO で読むなら DH = 求めるバイト数。
;       C2h なら ES:DI = 組の 8 バイト。見分けの間（W_PROBE）は選択の時間切れが短い。
; 出力: AH = 結果（00h、仕様書 4.5 / 付録 C の値、40h、90h）、AL = 1 なら完了まで約 65536 回より長くかかった、
;       DL = PIO で読んだバイト数（最初の 4 バイトは 0586h〜0589h、最後の 2 バイトは DI）、
;       CH = チェックコンディションのときのセンスの種類（SASI のセンスはバイト 0 の bit5〜0、拡張センスは 80h + センスキー。ほかは FFh）
; 割り込みを待つところ（.wait）では、この中で何も積まない（仕様書 2.6 の 28 バイト）。
sasi_exec:
.start:
        call    marker
        mov     ah, al                  ; AH = 印
        xor     bx, bx                  ; BH = 制御バイト、BL = LUN
        test    ah, XM_X
        jnz     .c0
        test    byte [bp + F_AH], 20h   ; 6 バイト型でリトライなし（Q28）。10 バイト型と見分けの間は 00h（Q43、Q45）
        jz      .c0
        mov     bh, 0C0h
.c0:
        test    ah, XM_ID1
        jnz     .l0
        mov     bl, [bp + F_AL]
        and     bl, 1
        ror     bl, 3
.l0:
        mov     al, ah
        mov     cx, si
        cmp     cl, 03h                 ; REQUEST SENSE: 4 バイト、10 バイト型と見分けの間は 18 バイト（Q38）
        jne     .count
        mov     [DISK_SENSE + 1], bl
        mov     word [DISK_SENSE + 2], 0
        mov     dh, 4
        test    al, XM_X
        jz      .rs
        mov     dh, 18
.rs:
        mov     [DISK_SENSE], dh
        jmp     .sel
.count:                                 ; 10 バイトの読み書き: ブロック数を BH、DH に
        test    ch, E_TEN >> 8
        jz      .sel
        cmp     cl, 25h
        je      .sel
        mov     cl, [bp + F_BH]
        xor     ch, ch
        test    al, XM_512
        jz      .c256
        shr     cl, 1
        jnz     .cset
        mov     cl, 80h
        jmp     .cset
.c256:
        test    cl, cl
        jnz     .cset
        inc     ch
.cset:
        mov     bh, cl
        mov     dh, ch
.sel:                                   ; 選択（仕様書 2.4 の 1、付録 C）
        xor     cx, cx
.w0:
        in      al, P_CTRL
        test    al, S_BSY
        jz      .s1
        loop    .w0
        jmp     .timeout
.s1:
        mov     al, 01h
        test    ah, XM_ID1
        jz      .id
        mov     al, 02h
.id:
        or      al, HOST_ID_BIT
        out     P_DATA, al
        mov     al, C_NRDSW | C_SEL
        out     P_CTRL, al
        xor     cx, cx
        test    byte [bp + F_AH], W_PROBE
        jz      .w1
        mov     cx, SHORT_SEL
.w1:
        in      al, P_CTRL
        test    al, S_BSY
        jnz     .s2
        loop    .w1
        jmp     .timeout
.s2:
        mov     ax, si
        mov     al, ah
        shr     al, 4
        and     al, 03h                 ; INTE、DMAE
        or      al, M_PIO
        out     P_CTRL, al
        and     byte [DISK_DONE], 0FEh  ; 仕様書 2.2（Q26）
        xor     dl, dl
.cmd:                                   ; 各バイトの前にフェーズを確かめる
        call    wait_req
        jc      .timeout
        test    al, S_CXD
        jz      .timeout                ; コマンドの途中でデータフェーズ: 手順の誤りとして時間切れと同じに
        test    al, S_IXO
        jnz     .early                  ; ステータスフェーズ: 残りは送らない
        call    cdb_byte
        out     P_DATA, al
        inc     dl
        mov     al, 6
        test    si, E_TEN
        jz      .len
        mov     al, 10
.len:
        cmp     dl, al
        jb      .cmd
        mov     ax, si
        test    ah, (E_IN | E_OUT) >> 8
        jz      .data_done
        xor     dl, dl
        xor     bx, bx
.pio:
        call    wait_req
        jc      .timeout
        mov     ah, al
        test    si, E_IN
        jz      .out
        and     ah, S_CXD | S_MSG | S_IXO       ; 読む: データイン（CXD = 0、MSG = 0、IXO = 1）だけ（Q30）
        cmp     ah, S_IXO
        jne     .data_done
        in      al, P_DATA
        cmp     dl, 4
        jae     .keep
        push    bx
        mov     bl, dl
        xor     bh, bh
        mov     [DISK_SENSE + bx], al
        pop     bx
        jmp     .next
.keep:                                  ; 5 バイト目からは最後の 2 バイトだけを BX に残す（読み捨て。0.8 版）
        cmp     dl, 6
        jne     .shift
        test    bx, bx
        jz      .shift
        or      si, E_C2_512
.shift:
        mov     bh, bl
        mov     bl, al
.next:
        inc     dl
        cmp     dl, dh
        jb      .pio
        jmp     .data_done
.out:                                   ; 送る: MSG = 0、IXO = 0（CXD は問わない。Q3）
        test    ah, S_MSG | S_IXO
        jnz     .data_done
        mov     al, [es:di]
        cmp     dl, 8
        jb      .put
        mov     al, 0
        ja      .put
        test    si, E_C2_512
        jz      .put
        inc     al
.put:
        out     P_DATA, al
        inc     di
        inc     dl
        cmp     dl, 10
        jb      .pio
        jmp     .data_done
.early:
        xor     dl, dl
.data_done:
        mov     di, bx                  ; 読んだ最後の 2 バイト
        test    si, E_INTE
        jz      .status
        mov     dx, WAIT_NORMAL         ; 時間切れ: 約 10 秒、FORMAT DRIVE は約 10 分（仕様書 2.4 の 4、4.6。0.12 版）
        mov     ax, si
        cmp     al, 04h
        jne     .w65
        mov     dx, WAIT_FORMAT
.w65:
        xor     cx, cx
.wait:                                  ; 割り込みハンドラが 055Fh の bit0 を立てるのを待つ。ここでは何も積まない
        test    byte [DISK_DONE], 01h
        jnz     .waited
        loop    .wait
        or      si, E_SLOW              ; 約 65536 回を過ぎた（0h のシークの AH = 10h）
        dec     dx
        jnz     .wait
        call    dma_mask                ; DMA のチャネル c をマスクして時間切れ（仕様書 2.3、2.4 の 4、7）
        jmp     .timeout
.waited:
        and     byte [DISK_DONE], 0FEh
.status:                                ; REQ、BSY、CXD、IXO が 1、MSG が 0（Q29）
        mov     bx, (S_REQ | S_BSY | S_MSG | S_CXD | S_IXO) << 8 | (S_REQ | S_BSY | S_CXD | S_IXO)
        call    wait_bus
        jc      .timeout
        in      al, P_DATA
        mov     [DISK_STATUS], al       ; 最後に読んだステータス（Q7）
        mov     bx, (S_REQ | S_BSY | S_MSG | S_CXD | S_IXO) << 8 | (S_REQ | S_BSY | S_MSG | S_CXD | S_IXO)
        call    wait_bus
        jc      .timeout
        in      al, P_DATA
        mov     bx, S_BSY << 8 | 0      ; バスフリー（Q4）
        call    wait_bus
        jc      .timeout
        mov     al, C_NRDSW
        out     P_CTRL, al
        mov     ch, 0FFh
        mov     al, [DISK_STATUS]
        test    al, 02h                 ; bit1 を先に見る（Q1）
        jnz     .check
        test    al, 01h
        jnz     .e40
        xor     ah, ah
        mov     bx, si
        cmp     bl, 03h
        jne     .ret
        mov     al, [DISK_SENSE]        ; REQUEST SENSE の結果を読み替える（仕様書 4.5、付録 C）
        and     al, 7Fh
        cmp     al, 70h
        je      .ext
        cmp     al, 71h
        je      .ext
        and     al, 3Fh
        mov     ch, al
        mov     bx, sense_map
.find:
        mov     ah, [cs:bx]
        cmp     ah, 0FFh
        je      .e40
        cmp     ah, al
        je      .hit
        add     bx, 2
        jmp     .find
.hit:
        mov     ah, [cs:bx + 1]
        jmp     .ret
.ext:                                   ; 拡張センス: センスキーの表（Q39）
        mov     al, [DISK_SENSE + 2]
        and     al, 0Fh
        mov     ch, al
        or      ch, 80h
        mov     ah, 40h
        cmp     al, 7
        ja      .ret
        mov     bl, al
        xor     bh, bh
        mov     ah, [cs:ext_map + bx]
        jmp     .ret
.check:
        mov     bx, si
        cmp     bl, 03h
        je      .e40                    ; REQUEST SENSE 自身のエラー（Q2）
        mov     si, E_IN | 03h          ; REQUEST SENSE を続けて出す
        jmp     .start
.e40:
        mov     ah, 40h
        jmp     .ret
.timeout:
        mov     al, C_NRDSW             ; 仕様書 2.4 の 7（Q21）
        out     P_CTRL, al
        mov     ah, 90h
        mov     ch, 0FFh
        xor     dl, dl
.ret:
        xor     al, al
        test    si, E_SLOW
        jz      .fast
        inc     al
.fast:
        ret

; ---------------------------------------------------------------- 表

; 仕様書 4.5（センスのコード、AH）。載っていないコードは 40h
sense_map:
        db      04h, 60h,  10h, 0A0h,  11h, 0B0h,  12h, 0E0h,  13h, 0F0h,  14h, 0C0h
        db      15h, 0C8h, 16h, 40h,   17h, 70h,   18h, 08h,   19h, 0D0h,  1Ch, 0B8h
        db      1Eh, 88h,  1Fh, 50h,   21h, 38h,   23h, 30h
        db      0FFh

; 拡張センスのセンスキー 0h〜7h → AH（付録 C。RECOVERED ERROR は成功）
ext_map:
        db      40h, 00h, 60h, 0A0h, 40h, 38h, 60h, 70h

; 容量コード → パラメータ表（仕様書 3.1、3.2）。111b は 001b の表
thick_tab:
        dw      tbl_0, tbl_1, tbl_2, tbl_3, tbl_4k, tbl_5, tbl_6, tbl_1
thin_tab:
        dw      tbl_0, tbl_1, tbl_2, tbl_3, tbl_4n, tbl_5, tbl_6, tbl_1

; 1 個 30 バイト: +00h 組 A（8）、+0Ah 組 B（8）、+14h センスの AH、+18h CHS のヘッド数、+1Ch 新センスの AH
%macro ptab 19
        db      %1, %2, %3, %4, %5, %6, %7, %8, 0, 0
        db      %9, %10, %11, %12, %13, %14, %15, %16, 0, 0
        db      %17, 0, 0, 0, %18, 0, 0, 0, %19, 0
%endmacro

tbl_0:  ptab    01h,01h,00h,03h,00h,98h,40h,00h,  01h,01h,00h,03h,01h,54h,40h,00h,  00h, 04h, 00h
tbl_1:  ptab    01h,01h,00h,03h,01h,35h,80h,00h,  01h,01h,00h,03h,01h,54h,80h,00h,  01h, 04h, 01h
tbl_1x: ptab    01h,01h,00h,03h,01h,35h,80h,00h,  01h,01h,00h,03h,01h,4Ch,80h,00h,  01h, 04h, 01h
tbl_2:  ptab    01h,01h,00h,05h,01h,35h,80h,00h,  01h,01h,00h,05h,01h,54h,80h,00h,  02h, 06h, 02h
tbl_3:  ptab    01h,01h,00h,07h,01h,35h,80h,00h,  01h,01h,00h,07h,01h,54h,80h,00h,  03h, 08h, 03h
tbl_4k: ptab    01h,01h,00h,03h,02h,66h,00h,00h,  01h,01h,00h,03h,02h,0A0h,00h,00h, 03h, 08h, 03h
tbl_4n: ptab    01h,01h,00h,03h,02h,66h,00h,00h,  01h,01h,00h,03h,02h,0A0h,00h,00h, 03h, 04h, 03h
tbl_5:  ptab    01h,01h,00h,05h,02h,66h,00h,00h,  01h,01h,00h,05h,02h,0A0h,00h,00h, 04h, 06h, 05h
tbl_6:  ptab    01h,01h,00h,07h,02h,66h,00h,00h,  01h,01h,00h,07h,02h,0A0h,00h,00h, 05h, 08h, 07h

; 10 バイト型の記述（付録 C の「10 バイト型のジオメトリ」）。3.2 の並びで、+03h ヘッド数 − 1、+04h 頭打ちのシリンダ数 − 1、
; +08h 印、+09h セクタ数、+14h センスの AH、+18h ヘッド数、+1Ch 新センスの AH。前の 6 個は ID 0、後の 6 個は 81h を ID 1 の LUN 0 で受けるもの
%macro xd 6                             ; ヘッド数、セクタ数、頭打ちのシリンダ数、04h の AH、84h の AH、印
        db      0, 0, 0, %1 - 1, (%3 - 1) >> 8, (%3 - 1) & 0FFh, 0, 0, %6, %2
        times 10 db 0
        db      %4, 0, 0, 0, %1, 0, 0, 0, %5, 0
%endmacro
%macro xdset 1
        xd      8, 17, 65535, 0Fh, 0Fh, XM_X | XM_512 | %1     ; 0: 512 バイト、8 × 17
        xd      16, 63, 65535, 0Fh, 0Fh, XM_X | XM_512 | %1    ; 1: 512 バイト、16 × 63
        xd      8, 33, 615, 05h, 07h, XM_X | %1                ; 2: 256 バイト、40 MB の型
        xd      8, 33, 310, 03h, 03h, XM_X | %1                ; 3: 20 MB の型
        xd      4, 33, 310, 01h, 01h, XM_X | %1                ; 4: 10 MB の型
        xd      4, 33, 153, 00h, 00h, XM_X | %1                ; 5: 5 MB の型
%endmacro
xdesc:
        xdset   0
        xdset   XM_ID1
XDESC_SIZE      equ     30
XDESC_COUNT     equ     6

        times   0FFEh - ($ - $$) db 0FFh
        db      00h                     ; 0FFEh
        db      00h                     ; 0FFFh: mkrom.py が全バイトの和を 00h に合わせる（Q23）
