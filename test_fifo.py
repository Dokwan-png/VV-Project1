"""STUDENT STARTER -- Project 1 FIFO testbench.

Build a driver / monitor / scoreboard testbench around the FWFT FIFO. Use the
golden model (models.fifo_model.FifoModel) as your scoreboard and check count,
full, empty, and rd_data *every* cycle. Hit the corners: fill to full, drain to
empty, write-while-full (dropped), read-while-empty (ignored), simultaneous
read+write, and pointer wrap.
"""""
###############################################################################################################################
#Use of AI was used to help with python syntax for the testbench and troubleshooting all test stimuli was made personally
###############################################################################################################################
import random
from collections import namedtuple
from dataclasses import dataclass
from typing import Optional
 
import cocotb
import vsc
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer
from models.fifo_model import FifoModel
 
WIDTH = 8
DEPTH = 8      # must match the RTL parameter
SEED = 1234
SEEDS = [SEED + 7 * i for i in range(10)]   # T16 sweep
STRICT_COVERAGE = True                       # T16 fails if any bin is unhit
 
 
def seed():
    random.seed(SEED)
 
 
# ---------------------------------------------------------------- transaction
KIND_IDLE, KIND_RD, KIND_WR, KIND_BOTH = 0, 1, 2, 3     # {wr_en, rd_en}
MAX_BURST = DEPTH + 4          # long enough to run past full / past empty
 
 
@vsc.randobj
class FifoTxn:
 
    def __init__(self):
        self.kind = vsc.rand_bit_t(2)
        self.kind_empty = vsc.rand_bit_t(2)
        self.kind_full = vsc.rand_bit_t(2)
        self.kind_mid = vsc.rand_bit_t(2)
        self.wr_en = vsc.rand_bit_t(1)
        self.rd_en = vsc.rand_bit_t(1)
        self.len_sel = vsc.rand_bit_t(2)         # 0 single, 1 short burst, 2 long burst
        self.burst_len = vsc.rand_bit_t(MAX_BURST.bit_length())
        self.wr_data = vsc.rand_bit_t(WIDTH)
        self.occ = vsc.uint8_t()
 
    @vsc.constraint
    def legal_c(self):
        self.wr_en == self.kind[1]
        self.rd_en == self.kind[0]
        self.burst_len in vsc.rangelist(vsc.rng(1, MAX_BURST))
        self.occ <= DEPTH
        with vsc.if_then(self.kind == KIND_IDLE):             # no long idle stretches
            self.len_sel != 2
 
    # pyvsc note: a dist placed *inside* an if_then/else branch ignores its
    # weights, so each zone's weights sit on a helper variable with an
    # unconditional dist, and the occupancy branch only selects which helper
    # `kind` equals.
    @vsc.constraint
    def zone_weights_c(self):
        vsc.dist(self.kind_empty, [vsc.weight(KIND_IDLE, 1), vsc.weight(KIND_RD, 2),
                                   vsc.weight(KIND_WR, 6), vsc.weight(KIND_BOTH, 3)])
        vsc.dist(self.kind_full, [vsc.weight(KIND_IDLE, 1), vsc.weight(KIND_RD, 6),
                                  vsc.weight(KIND_WR, 2), vsc.weight(KIND_BOTH, 3)])
        vsc.dist(self.kind_mid, [vsc.weight(KIND_IDLE, 1), vsc.weight(KIND_RD, 2),
                                 vsc.weight(KIND_WR, 2), vsc.weight(KIND_BOTH, 5)])
 
    @vsc.constraint
    def occupancy_c(self):
        with vsc.if_then(self.occ <= 1):                      # near empty
            self.kind == self.kind_empty
        with vsc.else_if(self.occ >= DEPTH - 1):              # near full
            self.kind == self.kind_full
        with vsc.else_then:                                   # mid-range
            self.kind == self.kind_mid
 
    # Burst length: the weighted choice sits on `len_sel` (a dist on burst_len
    # itself is distorted by the other constraints), then burst_len follows it.
    @vsc.constraint
    def burst_c(self):
        vsc.dist(self.len_sel, [vsc.weight(0, 6), vsc.weight(1, 3), vsc.weight(2, 1)])
        with vsc.if_then(self.len_sel == 0):
            self.burst_len == 1
        with vsc.else_if(self.len_sel == 1):
            self.burst_len in vsc.rangelist(vsc.rng(2, 4))
        with vsc.else_then:
            self.burst_len in vsc.rangelist(vsc.rng(DEPTH, MAX_BURST))
 
    @vsc.constraint
    def data_c(self):
        vsc.dist(self.wr_data, [vsc.weight(0x00, 2), vsc.weight(0xFF, 2),
                                vsc.weight(0x55, 2), vsc.weight(0xAA, 2),
                                vsc.weight(vsc.rng(0x01, 0x54), 1),
                                vsc.weight(vsc.rng(0x56, 0xA9), 1),
                                vsc.weight(vsc.rng(0xAB, 0xFE), 1)])
 
 
# Directed tests need no solver: any object with wr_en/rd_en/wr_data drives the pins.
Pins = namedtuple("Pins", "wr_en rd_en wr_data")
 
 
def check_reset_state(dut):
    """F1: the state the spec guarantees after a reset edge."""
    assert int(dut.count.value) == 0, f"after reset: count {int(dut.count.value)} != 0"
    assert bool(dut.empty.value), "after reset: empty != 1"
    assert not bool(dut.full.value), "after reset: full != 0"
 
 
class FifoDriver:
    """Turns transactions into pin wiggles. Owns the clock and the reset."""
 
    def __init__(self, dut, period_ns=10):
        self.dut = dut
        cocotb.start_soon(Clock(dut.clk, period_ns, units="ns").start())
 
    async def reset(self, check_first_edge=True):
       
        d = self.dut
        d.wr_en.value = 0
        d.rd_en.value = 0
        d.wr_data.value = 0
        d.rst_n.value = 0
        await RisingEdge(d.clk)
        await Timer(1, units="ns")
        if check_first_edge:
            check_reset_state(d)
        for _ in range(2):
            await RisingEdge(d.clk)
        d.rst_n.value = 1
        await FallingEdge(d.clk)
 
    async def drive(self, txn):
        d = self.dut
        d.wr_en.value = int(txn.wr_en)
        d.rd_en.value = int(txn.rd_en)
        d.wr_data.value = int(txn.wr_data)
        await RisingEdge(d.clk)
        await Timer(1, units="ns")
 
 
@dataclass
class Observed:
    cycle: int
    wr_en: int
    rd_en: int
    wr_data: int
    count: int
    full: bool
    empty: bool
    rd_data: Optional[int]     # FWFT head; None while the DUT claims empty
 
 
class FifoMonitor:
 
    def __init__(self, dut):
        self.dut = dut
        self.cycle = 0
 
    def capture(self):
        d = self.dut
        self.cycle += 1
        empty = bool(d.empty.value)
        return Observed(self.cycle,
                        int(d.wr_en.value), int(d.rd_en.value),
                        int(d.wr_data.value), int(d.count.value),
                        bool(d.full.value), empty,
                        None if empty else int(d.rd_data.value))
 
 
# ---------------------------------------------------------------- coverage
class Coverage:
    """Coverpoints C1-C11 and crosses X1-X3 from the coverpoint table.
    Never sampled while rst_n==0 (ignore bins): the driver's reset path and the
    drive/capture path are separate, so reset cycles never reach sample().
    Shared across tests (module-level COV) so T16 can check merged closure."""
 
    def __init__(self):
        self.bins = {}
        for i in range(DEPTH + 1):
            self._decl(f"C1_count_{i}")
        for wr in (0, 1):
            for rd in (0, 1):
                self._decl(f"C2_req_{wr}{rd}")
        for n in ("C3_wr_accepted", "C3_wr_dropped", "C3_wr_none",
                  "C4_rd_accepted", "C4_rd_ignored", "C4_rd_none",
                  "C5_reset_at_empty", "C5_reset_at_partial", "C5_reset_at_full",
                  "C6_empty_1to0", "C6_empty_0to1", "C6_full_0to1", "C6_full_1to0",
                  "C7_wr_wrap", "C7_rd_wrap",
                  "C8_pass_1", "C8_pass_2plus",
                  "C9_drop_run_1", "C9_drop_run_2plus",
                  "C10_head_after_first_write", "C10_head_after_pop",
                  "C11_data_00", "C11_data_FF", "C11_data_55", "C11_data_AA",
                  "C11_data_other",
                  "X2_wrwrap_10", "X2_wrwrap_11", "X2_rdwrap_01", "X2_rdwrap_11",
                  "X3_empty_exit_10", "X3_empty_exit_11",
                  "X3_full_exit_01", "X3_full_exit_11"):
            self._decl(n)
        for b in ("e0", "e1", "mid", "dm1", "full"):
            for wr in (0, 1):
                for rd in (0, 1):
                    self._decl(f"X1_{b}_{wr}{rd}")
        self.passes = 0
        self.on_reset()
 
    def _decl(self, name):
        self.bins[name] = 0
 
    def hit(self, name):
        assert name in self.bins, f"undeclared coverage bin {name}"
        self.bins[name] += 1
 
    def on_reset(self):
        """Pointer / run tracking restarts with the DUT."""
        self.wptr = 0
        self.rptr = 0
        self.seen_full = False
        self.drop_run = 0
 
    def sample_reset(self, pre_count):
        if pre_count == 0:
            self.hit("C5_reset_at_empty")
        elif pre_count == DEPTH:
            self.hit("C5_reset_at_full")
        else:
            self.hit("C5_reset_at_partial")
 
    @staticmethod
    def _bucket(c):
        if c == 0:
            return "e0"
        if c == 1:
            return "e1"
        if c == DEPTH:
            return "full"
        if c == DEPTH - 1:
            return "dm1"
        return "mid"
 
    def sample(self, pre, obs, post):
        """pre = (count, full, empty) of the model before the step; post = model after."""
        pre_count, pre_full, pre_empty = pre
        wr, rd = obs.wr_en, obs.rd_en
        req = f"{wr}{rd}"
        wr_ok = bool(wr) and not pre_full
        rd_ok = bool(rd) and not pre_empty
 
        self.hit(f"C1_count_{post.count}")
        self.hit(f"C2_req_{req}")
        self.hit("C3_wr_accepted" if wr_ok else
                 "C3_wr_dropped" if wr else "C3_wr_none")
        self.hit("C4_rd_accepted" if rd_ok else
                 "C4_rd_ignored" if rd else "C4_rd_none")
 
        # C6 flag transitions
        if pre_empty and not post.empty:
            self.hit("C6_empty_1to0")
            if req in ("10", "11"):
                self.hit(f"X3_empty_exit_{req}")
        if not pre_empty and post.empty:
            self.hit("C6_empty_0to1")
        if not pre_full and post.full:
            self.hit("C6_full_0to1")
        if pre_full and not post.full:
            self.hit("C6_full_1to0")
            if req in ("01", "11"):
                self.hit(f"X3_full_exit_{req}")
 
        # C7 wrap events (accepted operations only) and X2
        if wr_ok and self.wptr == DEPTH - 1:
            self.hit("C7_wr_wrap")
            if req in ("10", "11"):
                self.hit(f"X2_wrwrap_{req}")
        if rd_ok and self.rptr == DEPTH - 1:
            self.hit("C7_rd_wrap")
            if req in ("01", "11"):
                self.hit(f"X2_rdwrap_{req}")
        if wr_ok:
            self.wptr = (self.wptr + 1) % DEPTH
        if rd_ok:
            self.rptr = (self.rptr + 1) % DEPTH
 
        # C8 full-then-drained passes
        if post.full:
            self.seen_full = True
        if self.seen_full and post.empty:
            self.seen_full = False
            self.passes += 1
        if self.passes >= 1:
            self.bins["C8_pass_1"] = max(self.bins["C8_pass_1"], 1)
        if self.passes >= 2:
            self.bins["C8_pass_2plus"] = max(self.bins["C8_pass_2plus"], 1)
 
        # C9 consecutive dropped writes
        if wr and pre_full:
            self.drop_run += 1
            self.hit("C9_drop_run_1" if self.drop_run == 1 else "C9_drop_run_2plus")
        else:
            self.drop_run = 0
 
        # C10 FWFT head visibility (rd_data itself was checked by the scoreboard)
        if pre_empty and wr_ok and not rd and not post.empty \
                and obs.rd_data == obs.wr_data:
            self.hit("C10_head_after_first_write")
        if rd_ok and not post.empty:
            self.hit("C10_head_after_pop")
 
        # C11 data pattern on accepted writes
        if wr_ok:
            names = {0x00: "00", 0xFF: "FF", 0x55: "55", 0xAA: "AA"}
            self.hit(f"C11_data_{names.get(obs.wr_data, 'other')}")
 
        # X1 occupancy (before the op) x request
        self.hit(f"X1_{self._bucket(pre_count)}_{req}")
 
    def snapshot(self):
        return dict(self.bins)
 
    def holes(self):
        return [n for n, v in self.bins.items() if v == 0]
 
    def report(self):
        holes = self.holes()
        total = len(self.bins)
        cocotb.log.info(f"coverage: {total - len(holes)}/{total} bins hit")
        if holes:
            cocotb.log.warning(f"coverage holes: {holes}")
        return holes
 
 
COV = Coverage()
 
 
# ---------------------------------------------------------------- scoreboard
class FifoScoreboard:
 
    def __init__(self, cov=None):
        self.model = FifoModel(WIDTH, DEPTH)
        self.cov = cov
        self.checked = 0
 
    def reset(self):
        self.model = FifoModel(WIDTH, DEPTH)
 
    def update(self, obs):
        m = self.model
        pre = (m.count, m.full, m.empty)
        m.step(obs.wr_en, obs.wr_data, obs.rd_en)
 
        assert 0 <= obs.count <= DEPTH, \
            f"cycle {obs.cycle}: illegal count {obs.count} (must be 0..{DEPTH})"
        assert not (obs.full and obs.empty), \
            f"cycle {obs.cycle}: full and empty both high"
 
        ok = (obs.count == m.count and obs.full == m.full
              and obs.empty == m.empty
              and (m.empty or obs.rd_data == m.head))
        if not ok:
            def hx(v):
                return "--" if v is None else f"0x{v:02X}"
            cocotb.log.error(
                f"first divergence: cycle {obs.cycle} -- "
                f"count {obs.count} vs model {m.count}, "
                f"full {int(obs.full)} vs {int(m.full)}, "
                f"empty {int(obs.empty)} vs {int(m.empty)}, "
                f"rd_data {hx(obs.rd_data)} vs head {hx(m.head)} | "
                f"pins that cycle: wr_en={obs.wr_en} rd_en={obs.rd_en} "
                f"wr_data=0x{obs.wr_data:02X}")
        assert obs.count == m.count, \
            f"cycle {obs.cycle}: count {obs.count} != model {m.count}"
        assert obs.full == m.full, \
            f"cycle {obs.cycle}: full {obs.full} != model {m.full}"
        assert obs.empty == m.empty, \
            f"cycle {obs.cycle}: empty {obs.empty} != model {m.empty}"
        if not m.empty:  # FWFT: head valid only when not empty
            assert obs.rd_data == m.head, \
                f"cycle {obs.cycle}: rd_data {obs.rd_data:#04x} != head {m.head:#04x}"
        self.checked += 1
        if self.cov is not None:
            self.cov.sample(pre, obs, m)
 
 
# ---------------------------------------------------------------- environment
class Env:
 
    def __init__(self, dut):
        self.dut = dut
        self.drv = FifoDriver(dut)
        self.mon = FifoMonitor(dut)
        self.sb = FifoScoreboard(COV)
        self.applied = []
 
    async def start(self):
        await self.drv.reset(check_first_edge=False)
        COV.on_reset()
 
    async def midrun_reset(self):
        """Reset with the FIFO in whatever state it is in (C5)."""
        COV.sample_reset(self.sb.model.count)
        await self.drv.reset(check_first_edge=True)
        self.sb.reset()
        COV.on_reset()
        check_reset_state(self.dut)
 
    async def send(self, txn):
        """Drive one transaction (a pyvsc FifoTxn or a Pins tuple) and judge it."""
        await self.drv.drive(txn)
        obs = self.mon.capture()
        self.sb.update(obs)
        self.applied.append((int(txn.wr_en), int(txn.rd_en), int(txn.wr_data)))
        return obs
 
    async def step(self, wr, rd, data=0):
        return await self.send(Pins(wr, rd, data))
 
    async def run(self, name, seq):
        cocotb.log.info(f"sequence {name}: {len(seq)} cycles")
        for wr, rd, data in seq:
            await self.step(wr, rd, data)
 
    def head(self):
        return int(self.dut.rd_data.value)
 
    async def goto_level(self, n, base=0x80):
        """Move occupancy to n using plain writes or reads (model-tracked)."""
        while self.sb.model.count < n:
            await self.step(1, 0, (base + self.sb.model.count) & 0xFF)
        while self.sb.model.count > n:
            await self.step(0, 1, 0)
 
    async def drain_check(self, expected):
        """Pop every entry, comparing the head to the expected order first."""
        for v in expected:
            assert not bool(self.dut.empty.value), "FIFO empty before all data came out"
            assert self.head() == v, f"head {self.head():#04x} != expected {v:#04x}"
            await self.step(0, 1, 0)
        assert bool(self.dut.empty.value), "FIFO not empty after draining"
 
    def report(self):
        cocotb.log.info(f"{len(self.applied)} cycles applied, "
                        f"{self.sb.checked} judged, 0 mismatches")
 
 
# ---------------------------------------------------------------- stimulus
# Each step is (wr_en, rd_en, wr_data).
def writes(n, base):
    return [(1, 0, (base + i) & 0xFF) for i in range(n)]
 
 
def reads(n):
    return [(0, 1, 0)] * n
 
 
def both(n, base):
    return [(1, 1, (base + i) & 0xFF) for i in range(n)]
 
 
def idle(n):
    return [(0, 0, 0)] * n
 
 
def vals(n, base):
    return [(base + i) & 0xFF for i in range(n)]
 
 
async def run_random(env, cycles, rand_seed, label):
    """Randomize one burst at a time, with the live occupancy as context, and
    expand it into `burst_len` cycles (write data counts up from wr_data)."""
    cocotb.log.info(f"random {label}: {cycles} cycles")
    txn = FifoTxn()
    txn.set_randstate(vsc.RandState.mkFromSeed(rand_seed, "fifo"))
    done = 0
    while done < cycles:
        txn.occ = env.sb.model.count
        txn.randomize()
        for i in range(int(txn.burst_len)):
            if done == cycles:
                break
            await env.step(int(txn.wr_en), int(txn.rd_en),
                           (int(txn.wr_data) + i) & 0xFF)
            done += 1
 
 
GUARDS = (["C2_req_11", "C3_wr_dropped", "C4_rd_ignored"]
          + [f"C1_count_{i}" for i in range(DEPTH + 1)])
 
 
def check_guards(before, label):
    """Coverage guards judged on the delta of this random run alone."""
    delta = {n: COV.bins[n] - before[n] for n in GUARDS}
    missing = [n for n, v in delta.items() if v == 0]
    assert not missing, f"{label}: coverage guards not reached: {missing}"
 
 
####################################################################################################################
# directed tests
####################################################################################################################
@cocotb.test()
async def test_reset_basic(dut):
    seed()
    env = Env(dut)
    await env.start()
    check_reset_state(dut)
    await env.run("idle_after_reset", idle(2))
    await env.run("fill_3", writes(3, 0x11))
    await env.midrun_reset()          # first reset edge checked inside
    await env.midrun_reset()          # reset again while already empty (C5 empty)
    env.report()
 
 
@cocotb.test()
async def test_reset_midrun(dut):
    seed()
    env = Env(dut)
    await env.start()
    for level in (3, DEPTH):
        await env.run(f"fill_{level}", writes(level, 0x20))
        await env.midrun_reset()
        await env.run("post_reset_writes", writes(2, 0xC1))
        assert env.head() == 0xC1, f"stale head {env.head():#04x} after reset"
        await env.drain_check([0xC1, 0xC2])
    env.report()
 
 
@cocotb.test()
async def test_fill_to_full(dut):
    seed()
    env = Env(dut)
    await env.start()
    for i, v in enumerate(vals(DEPTH, 0x10), start=1):
        obs = await env.step(1, 0, v)
        assert obs.count == i, f"write {i}: count {obs.count}"
        assert not obs.empty, f"write {i}: empty still high"
        assert obs.full == (i == DEPTH), f"write {i}: full {obs.full}"
        if i == DEPTH - 1:
            await env.step(0, 0, 0)                   # idle at DEPTH-1 (X1 dm1_00)
    await env.step(0, 0, 0)                           # idle at full (X1 full_00)
    env.report()
 
 
@cocotb.test()
async def test_overwrite_while_full(dut):
    seed()
    env = Env(dut)
    await env.start()
    original = vals(DEPTH, 0x10)
    await env.run("fill", [(1, 0, v) for v in original])
    obs = await env.step(1, 0, 0xA0)                  # one dropped write
    assert obs.count == DEPTH and obs.full
    await env.step(0, 0, 0)                           # break the run
    for v in (0xB0, 0xB1, 0xB2):                      # 3 consecutive dropped writes
        obs = await env.step(1, 0, v)
        assert obs.count == DEPTH and obs.full
    await env.drain_check(original)
    env.report()
 
 
####################################################################################################################
# random tests
####################################################################################################################
 
 
@cocotb.test()
async def test_crv_seed_sweep(dut):
    env = Env(dut)
    await env.start()
    results = {}
    for s in SEEDS:
        await env.midrun_reset()
        before = COV.snapshot()
        try:
            await run_random(env, 2000, s, f"seed={s}")
            check_guards(before, f"seed {s}")
            results[s] = "PASS"
        except AssertionError as e:
            results[s] = f"FAIL: {e}"
            cocotb.log.error(f"seed {s} failed -- rerun with SEED={s} to reproduce")
    for s, r in results.items():
        cocotb.log.info(f"seed {s}: {r}")
    failed = {s: r for s, r in results.items() if r != "PASS"}
    assert not failed, f"seeds failed: {failed}"
 
    holes = COV.report()
    if STRICT_COVERAGE:
        assert not holes, f"merged coverage not closed: {holes}"
    env.report()