# Formal checks

`check.ys` runs a bounded model check of the FIFO invariants (occupancy bound,
pointer/count agreement, no pointer movement on empty pop / full push, count
changes by at most one per cycle) using Yosys' built-in SAT engine, so it needs
nothing beyond `yosys`:

```sh
yosys -q -s formal/check.ys
```

The assertions live under `` `ifdef FORMAL `` in `src/bitloom_fifo.v`. The
state-machine core is checked by the constrained-random cocotb suite in `test/`
against independent Python protocol models; see the project README.
