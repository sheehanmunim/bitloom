#!/usr/bin/env python3
"""BitLoom assembler.

Assembles a PIO-flavoured assembly dialect into 16-bit BitLoom instruction
words.  Usable as a library (``assemble(text)``) or a CLI that emits JSON.

Instruction word layout:  [15:13] opcode | [12:9] delay/side-set | [8:0] operand

Syntax summary (one instruction per line, ``;`` starts a comment)::

    .program name
    .side_set N [pindirs]      ; N in 0..3 side-set bits taken from the delay field
    .wrap_target / .wrap       ; loop bounds (default: whole program)
    label:
    jmp [cond] label           ; cond: (none) !x x-- !y y-- x!=y pin !osre
    wait 0|1 gpio N | pin N | flag N
    wait time                  ; block until T >= DL
    in  src, N                 ; src: pins x y null t status isr osr
    out dst, N                 ; dst: pins x y null pindirs pc isr dl
    push [iffull] [block|noblock]
    pull [ifempty] [block|noblock]
    mov dst, [!|~|::]src       ; dst: pins x y dl pindirs pc isr osr
    set dst, N                 ; dst: pins x y pindirs flag(set) clrflag t
    time t+N | dl+N | t+x | dl+x
    nop                        ; encoded as mov y, y

    Any instruction may be followed by ``side N`` and/or ``[N]`` (delay).
"""
from __future__ import annotations

import json
import re
import sys
from dataclasses import dataclass, field

OP = {"jmp": 0, "wait": 1, "in": 2, "out": 3, "push": 4, "pull": 4,
      "mov": 5, "set": 6, "time": 7}
JMP_COND = {"": 0, "!x": 1, "x--": 2, "!y": 3, "y--": 4, "x!=y": 5, "pin": 6, "!osre": 7}
WAIT_SRC = {"gpio": 0, "pin": 1, "flag": 2, "time": 3}
SRC = {"pins": 0, "x": 1, "y": 2, "null": 3, "t": 4, "status": 5, "isr": 6, "osr": 7}
OUT_DST = {"pins": 0, "x": 1, "y": 2, "null": 3, "pindirs": 4, "pc": 5, "isr": 6, "dl": 7}
MOV_DST = {"pins": 0, "x": 1, "y": 2, "dl": 3, "pindirs": 4, "pc": 5, "isr": 6, "osr": 7}
SET_DST = {"pins": 0, "x": 1, "y": 2, "pindirs": 3, "flag": 4, "clrflag": 5, "t": 6}
MOV_OP = {"": 0, "!": 1, "~": 1, "::": 2}


class AsmError(Exception):
    pass


@dataclass
class Program:
    name: str
    words: list[int]
    wrap_target: int
    wrap: int
    side_set: int = 0
    side_pindirs: bool = False
    labels: dict[str, int] = field(default_factory=dict)
    origin: int | None = None

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "words": [f"0x{w:04x}" for w in self.words],
            "wrap_target": self.wrap_target,
            "wrap": self.wrap,
            "side_set": self.side_set,
            "side_pindirs": self.side_pindirs,
            "labels": self.labels,
            "origin": self.origin,
        }

    def words_at(self, offset: int) -> list[int]:
        """Instruction words relocated to load address ``offset``.

        JMP targets are absolute, so they are rebased; nothing else changes.
        """
        if offset + len(self.words) > 64:
            raise AsmError("program does not fit at that offset")
        out = []
        for w in self.words:
            if (w >> 13) == OP["jmp"]:
                w = (w & 0xFFC0) | (((w & 0x3F) + offset) & 0x3F)
            out.append(w)
        return out

    def dump(self) -> str:
        out = [f"; program {self.name}: {len(self.words)} words, "
               f"wrap_target={self.wrap_target} wrap={self.wrap} side_set={self.side_set}"]
        for i, w in enumerate(self.words):
            out.append(f"  {i:2d}: 0x{w:04x}")
        return "\n".join(out)


def _int(tok: str) -> int:
    tok = tok.strip()
    try:
        return int(tok, 0)
    except ValueError as e:
        raise AsmError(f"bad number {tok!r}") from e


def assemble(text: str, name: str | None = None) -> Program:
    lines = text.splitlines()
    prog_name = name or "program"
    side_set = 0
    side_pindirs = False
    origin = None
    wrap_target = None
    wrap = None
    labels: dict[str, int] = {}
    items: list[tuple[int, str, str]] = []   # (lineno, mnemonic+args, side/delay suffix)

    # ---- pass 1: directives, labels, collect instructions -------------
    for ln, raw in enumerate(lines, 1):
        line = raw.split(";", 1)[0].strip()
        if not line:
            continue
        if line.startswith("."):
            parts = line.split()
            d = parts[0]
            if d == ".program":
                prog_name = parts[1]
            elif d == ".side_set":
                side_set = _int(parts[1])
                if not 0 <= side_set <= 3:
                    raise AsmError(f"line {ln}: .side_set must be 0..3")
                side_pindirs = "pindirs" in parts[2:]
                if "opt" in parts[2:]:
                    raise AsmError(f"line {ln}: optional side-set is not supported")
            elif d == ".wrap_target":
                wrap_target = len(items)
            elif d == ".wrap":
                wrap = len(items) - 1
            elif d == ".origin":
                origin = _int(parts[1])
            else:
                raise AsmError(f"line {ln}: unknown directive {d}")
            continue
        m = re.match(r"^([A-Za-z_][A-Za-z0-9_]*):\s*(.*)$", line)
        if m:
            labels[m.group(1)] = len(items)
            line = m.group(2).strip()
            if not line:
                continue
        items.append((ln, line, ""))

    if not items:
        raise AsmError("no instructions")
    if wrap_target is None:
        wrap_target = 0
    if wrap is None:
        wrap = len(items) - 1
    if len(items) > 64:
        raise AsmError(f"program has {len(items)} instructions, memory holds 64")

    delay_bits = 4 - side_set
    words: list[int] = []

    # ---- pass 2: encode ------------------------------------------------
    for ln, line in ((ln, l) for ln, l, _ in items):
        side = None
        delay = 0
        m = re.search(r"\[\s*(\d+)\s*\]\s*$", line)
        if m:
            delay = int(m.group(1))
            line = line[: m.start()].strip()
        m = re.search(r"\bside\s+(\S+)\s*$", line)
        if m:
            side = _int(m.group(1))
            line = line[: m.start()].strip()
        m2 = re.search(r"\[\s*(\d+)\s*\]\s*$", line)   # allow "side N [D]" either order
        if m2:
            delay = int(m2.group(1))
            line = line[: m2.start()].strip()

        if side_set == 0 and side is not None:
            raise AsmError(f"line {ln}: 'side' used without .side_set")
        if side_set and side is None:
            raise AsmError(f"line {ln}: .side_set {side_set} requires 'side N' on every instruction")
        if side is not None and not 0 <= side < (1 << side_set):
            raise AsmError(f"line {ln}: side value {side} does not fit in {side_set} bits")
        if not 0 <= delay < (1 << delay_bits):
            raise AsmError(f"line {ln}: delay {delay} does not fit in {delay_bits} bits")
        ds = ((side or 0) << delay_bits) | delay

        toks = line.replace(",", " ").split()
        mn = toks[0].lower()
        args = [t.lower() for t in toks[1:]]

        try:
            if mn == "nop":
                op, operand = OP["mov"], (MOV_DST["y"] << 6) | SRC["y"]
            elif mn == "jmp":
                if len(args) == 1:
                    cond, target = "", args[0]
                elif len(args) == 2:
                    cond, target = args
                else:
                    raise AsmError("jmp takes [cond] label")
                if cond not in JMP_COND:
                    raise AsmError(f"unknown jmp condition {cond!r}")
                if target in labels:
                    addr = labels[target]
                else:
                    addr = _int(target)
                op, operand = OP["jmp"], (JMP_COND[cond] << 6) | (addr & 0x3F)
            elif mn == "wait":
                if args == ["time"] or args == ["1", "time"]:
                    op, operand = OP["wait"], (WAIT_SRC["time"] << 6)
                else:
                    pol, src, idx = args
                    op = OP["wait"]
                    operand = (_int(pol) << 8) | (WAIT_SRC[src] << 6) | (_int(idx) & 0x3F)
            elif mn == "in":
                src, n = args
                n = _int(n)
                if not 1 <= n <= 16:
                    raise AsmError("in count must be 1..16")
                op, operand = OP["in"], (SRC[src] << 6) | (n & 0x1F)
            elif mn == "out":
                dst, n = args
                n = _int(n)
                if not 1 <= n <= 16:
                    raise AsmError("out count must be 1..16")
                op, operand = OP["out"], (OUT_DST[dst] << 6) | (n & 0x1F)
            elif mn in ("push", "pull"):
                is_pull = mn == "pull"
                cond = 0
                block = 1
                for a in args:
                    if a in ("iffull", "ifempty"):
                        cond = 1
                    elif a == "block":
                        block = 1
                    elif a == "noblock":
                        block = 0
                    else:
                        raise AsmError(f"bad {mn} modifier {a!r}")
                op, operand = OP["push"], (is_pull << 8) | (cond << 7) | (block << 6)
            elif mn == "mov":
                dst, src = args
                mop = ""
                for prefix in ("::", "!", "~"):
                    if src.startswith(prefix):
                        mop, src = prefix, src[len(prefix):]
                        break
                op, operand = OP["mov"], (MOV_DST[dst] << 6) | (MOV_OP[mop] << 4) | SRC[src]
            elif mn == "set":
                dst, n = args
                n = _int(n)
                if not 0 <= n <= 63:
                    raise AsmError("set value must be 0..63")
                op, operand = OP["set"], (SET_DST[dst] << 6) | n
            elif mn == "time":
                (expr,) = args
                m = re.match(r"^(t|dl)\+(x|\d+|0x[0-9a-f]+)$", expr)
                if not m:
                    raise AsmError("time expects t+N, dl+N, t+x or dl+x")
                base = 1 if m.group(1) == "dl" else 0
                if m.group(2) == "x":
                    mode = 2 | base
                    imm = 0
                else:
                    mode = base
                    imm = _int(m.group(2))
                    if not 0 <= imm <= 127:
                        raise AsmError("time immediate must be 0..127")
                op, operand = OP["time"], (mode << 7) | imm
            else:
                raise AsmError(f"unknown mnemonic {mn!r}")
        except KeyError as e:
            raise AsmError(f"line {ln}: bad operand {e}") from e
        except AsmError as e:
            raise AsmError(f"line {ln}: {e}") from e

        words.append((op << 13) | (ds << 9) | operand)

    return Program(prog_name, words, wrap_target, wrap, side_set, side_pindirs, labels, origin)


def assemble_file(path: str) -> Program:
    with open(path) as f:
        return assemble(f.read())


def main(argv: list[str]) -> int:
    if len(argv) < 2:
        print(__doc__)
        return 2
    prog = assemble_file(argv[1])
    if "--json" in argv:
        print(json.dumps(prog.to_dict(), indent=2))
    else:
        print(prog.dump())
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
