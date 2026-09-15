"""Unit tests for the assembler (run with pytest)."""
import pytest

from bitloom_asm import AsmError, assemble


def enc(line, side_set=0):
    hdr = f".side_set {side_set}\n" if side_set else ""
    return assemble(hdr + line).words[0]


def test_opcode_fields():
    assert enc("jmp 5") == 0x0005
    assert enc("jmp x-- 9") == (2 << 6) | 9
    assert enc("wait 1 gpio 3") == (1 << 13) | (1 << 8) | 3
    assert enc("wait time") == (1 << 13) | (3 << 6)
    assert enc("in pins, 16") == (2 << 13) | 16
    assert enc("in t, 16") == (2 << 13) | (4 << 6) | 16
    assert enc("out pindirs, 1") == (3 << 13) | (4 << 6) | 1
    assert enc("push") == (4 << 13) | (1 << 6)
    assert enc("pull noblock") == (4 << 13) | (1 << 8)
    assert enc("pull ifempty block") == (4 << 13) | (1 << 8) | (1 << 7) | (1 << 6)
    assert enc("mov osr, ~osr") == (5 << 13) | (7 << 6) | (1 << 4) | 7
    assert enc("mov x, ::pins") == (5 << 13) | (1 << 6) | (2 << 4)
    assert enc("set pins, 1") == 0xC001
    assert enc("set flag 3") == (6 << 13) | (4 << 6) | 3
    assert enc("time t+10") == (7 << 13) | 10
    assert enc("time dl+x") == (7 << 13) | (3 << 7)
    assert enc("nop") == (5 << 13) | (2 << 6) | 2


def test_delay_and_sideset():
    assert enc("nop [7]") == enc("nop") | (7 << 9)
    assert enc("nop side 1 [3]", side_set=1) == enc("nop") | (1 << 12) | (3 << 9)
    assert enc("nop side 2 [1]", side_set=2) == enc("nop") | (2 << 11) | (1 << 9)
    with pytest.raises(AsmError):
        enc("nop [8]", side_set=1)          # only 3 delay bits left
    with pytest.raises(AsmError):
        enc("nop", side_set=1)              # side value required
    with pytest.raises(AsmError):
        enc("nop side 1")                   # side without .side_set


def test_labels_wrap_and_relocation():
    p = assemble("""
        .program t
        start:
            set x, 3
        .wrap_target
        loop:
            jmp x-- loop
            jmp start
        .wrap
    """)
    assert p.wrap_target == 1 and p.wrap == 2
    assert p.words[1] & 0x3F == 1 and p.words[2] & 0x3F == 0
    r = p.words_at(20)
    assert r[0] == p.words[0]                      # non-jump untouched
    assert r[1] & 0x3F == 21 and r[2] & 0x3F == 20


def test_errors():
    with pytest.raises(AsmError):
        assemble("bogus x, 1")
    with pytest.raises(AsmError):
        assemble("in pins, 17")
    with pytest.raises(AsmError):
        assemble("set x, 64")
    with pytest.raises(AsmError):
        assemble("\n".join(["nop"] * 65))
