# Logic Reference — `tigers_red_right_auto`

Plain-text trace of what [src/main.py](src/main.py) actually does, written so logic errors can be
diagnosed by comparing *intent* against *as-written* behavior. Sections marked **AS-WRITTEN**
describe the code as it currently stands, which in several places differs from the stated intent.

**Scope:** this project only. Other folders in the repo (e.g. the teleop project) are **not** a source
of truth — they may be out of date. Do not read from them or cross-reference them when reviewing.

---

## 1. Conventions

### 1.1 Coordinate frame

- Field-relative frame with the origin at wherever the robot starts.
- **+Y = "forward"** (the direction the robot faces at heading 0). **+X = right.**
- Units are intended to be **inches**.

### 1.2 Angle convention (compass, not standard math)

- Angle 0 points along **+Y**. Angles increase **clockwise** (toward +X).
- This is why `calculateTargetAngle` calls `atan2(x, y)` with the arguments **swapped** relative to
  the usual `atan2(y, x)`. That swap is deliberate and correct for this frame.
- `position()` integrates with `x += sin(angle)·d`, `y += cos(angle)·d` — same convention. Consistent.
- `inertial_1.heading(DEGREES)` is also clockwise-positive. Consistent.

### 1.3 Angle ranges — the source of several bugs

| Quantity | Range | Notes |
|---|---|---|
| `inertial_1.heading(DEGREES)` | `[0, 360)` | **never negative** |
| `calculateTargetAngle(...)` | `(-180, 180]` | from `atan2` |
| `getMTD(...)` | within `±180` of `robotAngle` | re-wrapped by `moveAngleWithinRange` |

Comparing a `[0, 360)` value directly against a `(-180, 180]` value without wrapping is invalid.
This is exactly what the turn-loop exit condition does (see §5.3, Defect **L4**).

### 1.4 Units at every boundary

| Boundary | Unit / range | Enforced? |
|---|---|---|
| `inputCurve` **input** | percent `[-100, 100]` — divided by 100 on entry | no — contract is implicit |
| `inputCurveRaw` **input** | normalized `[-1, 1]` | no |
| `inputCurve` **output** | normalized `[-1, 1]` | tuned so `a+b+c = 100` ⇒ `f(100) = 1.0` |
| `drivetrain(l, r)` → `Motor.spin(..., PERCENT)` | percent `[-100, 100]` | clamped by firmware |
| `Rotation.angle()` | degrees `[0, 360)` — **wraps** | — |
| `Rotation.position()` | degrees — **accumulates** | — |
| `Inertial.gyro_rate(axis, DPS)` | degrees/sec | — |
| `x`, `y` globals | intended inches | **AS-WRITTEN: degrees** (no wheel conversion) |

The normalized-vs-percent gap between `inputCurve`'s output and `drivetrain`'s input is Defect **L2**.

---

## 2. Hardware map

```
Smart ports (auto project)
  PORT1  motorFR   (fwd)      PORT2  motorMR   (fwd)      PORT3  motorBR  (fwd)
  PORT10 motorFL   (rev)      PORT8  motorML   (rev)      PORT9  motorBL  (rev)
  PORT5  elevationL (rev)     PORT6  elevationR (fwd)     [declared, unused in auto]
  PORT4  trackingWheelVertL — Rotation           [TODO says this is a PLACEHOLDER port]
  PORT21 inertial_1                              [unverified — see Defect L9]
3-wire
  A  digital_out_a   B  digital_out_b            [declared, unused in auto]
```

Left-side motors are constructed reversed, so a **positive** speed on both sides drives forward and
`drivetrain(+s, -s)` turns **clockwise** (heading increases). The sign convention in the turn loop
matches this.

---

## 3. Program startup order

Module level runs top to bottom, then:

1. Motor/sensor objects constructed.
2. `x = 0`, `y = 0` set at module scope.
3. `wait(30, MSEC)` → `wait(200, MSEC)` → clear console.
4. All `def`s bound.
5. `comp = Competition(user_control, autonomous)` — **registers** callbacks, runs nothing.
6. `pre_autonomous()` — blocks 1 second.
7. `autonomous()` — **called directly**, unconditionally, ignoring field state (Defect **L7**).
8. Later, when the field enables autonomous, `autonomous()` runs **a second time**.

`position()` is documented as "a threaded function" but **`Thread(position)` is never created**, so
it never runs. `x` and `y` therefore stay `0` for the whole program (Defect **L5**).

---

## 4. Function contracts

### `drivetrain(leftSpeed, rightSpeed)`
Fire-and-forget. Spins all six motors `FORWARD` at the given `PERCENT`. Sign carries the direction.
Never blocks, never waits for completion. `drivetrain(0, 0)` sets velocity 0 (does not brake — no
`set_stopping()` is configured anywhere in this file).

### `position(startingX, startingY, startingAngle, trackingwheelDiamiter)`
**Intent:** a background odometry thread integrating the vertical tracking wheel into field `x`/`y`.

**AS-WRITTEN:**
```
x, y   = startingX, startingY
angle  = startingAngle            # set once, NEVER updated
trackingWheelVertL.set_position(0, DEGREES)
previousTrackingAngle = 0
loop forever:                      # no wait() in the loop body
    delta = trackingWheelVertL.angle() - previousTrackingAngle
    x += sin(radians(angle)) * delta
    y += cos(radians(angle)) * delta
    previousTrackingAngle = trackingWheelVertL.angle()
```
Four independent problems here — Defects **L5a–L5d**.

### `calculateTargetAngle(x, y, directionBool) -> degrees (-180, 180]`
- `directionBool == False` (reverse) ⇒ negate both `x` and `y`, i.e. aim the **back** of the robot at
  the point. Correct.
- `targetAngle = atan2(x, y)`, then converted to degrees via `360·θ/(2π)`.
- **AS-WRITTEN:** treats the arguments as a vector **from the origin**, not from the robot's current
  position. There is no `- x` / `- y`. Defect **L3**.
- **AS-WRITTEN:** uses the bare name `math`, which this file never imports (it imports `math as m`).
  Defect **L1**.

### `moveAngleWithinRange(referanceAngle, secondAngle) -> degrees`
`secondAngle - 360·round((secondAngle - referanceAngle)/360)`

Returns the representation of `secondAngle` nearest `referanceAngle`, i.e. within `±180`. **This
function is correct** — verified numerically:

| ref | second | result |
|---|---|---|
| 0 | 350 | −10 |
| 0 | 170 | 170 |
| 350 | 10 | 370 |
| 300 | −100 | 260 |

Only edge case: an exact 180° difference resolves by banker's rounding. Harmless.

### `getMTD(targetAngle, endAngle, robotAngle) -> degrees`
"Modified Target Direction" — a boomerang/carrot heading that curves the path so the robot arrives at
`endAngle`:
```
MTD = 2·targetAngle − endAngle          # overshoot the direct angle by (direct − desired final)
MTD = moveAngleWithinRange(robotAngle, MTD)   # take the short way around
```
The formula is sound. Note the doubling makes MTD **twice as sensitive** to error in `targetAngle`,
so `targetAngle` being wrong (Defect **L3**) is amplified here. When `endAngle == targetAngle`, MTD
collapses to `targetAngle` — a straight approach.

### `inputCurve(input, a, b, c, d, p)` and `inputCurveRaw(input, a, b, p)`
Odd-symmetric response curve. Contract: **percent `[-100, 100]` in, percent `[-100, 100]` out.**
Normalizes on entry, rescales on exit — the round trip is now consistent.
```
inputCurveRaw(i) = (a/100)·i^p + (b/100)·i          # i normalized [-1, 1]

inputCurve(i):
   i /= 100                                # percent → normalized
   i ≥  d/100 :  y = +inputCurveRaw((1 + d/100)·(i − d/100)) + c/100
   i ≤ −d/100 :  y = mirrored negative branch
   otherwise  :  y = 0                     # DEADZONE: |input| < d percent-units
   return y * 100                          # normalized → percent
```
- `a` = cubic weight, `b` = linear weight, `c` = static-friction kick, `d` = deadzone %, `p` = power.
- `a + b + c = 100` is the invariant that makes `f(100) = 100`. Auto: `8+90+2 = 100` ✓.
- Nit: the rescale factor is `(1 + d/100)`; exact would be `1/(1 − d/100)`. At `d = 1` the error is
  ~0.01%. Not worth fixing.
- Verified against the current code: `f(5) = 5.64`, `f(45) = 42.70`, `f(90) = 88.71`,
  `f(100) = 99.99`, `f(0.9) = 0` (deadzone).
- **The `c` kick means output jumps discontinuously from 0 to 2% at the deadzone edge.** That step,
  combined with the turn loop's ±0.5° exit window, creates a dead band the controller cannot escape —
  Defect **L10**.

### `linearize(leftMS, rightMS) -> (float, float)`
Applies `inputCurve` to both sides with module-level `a, b, c, d, p`. Percent in, percent out —
matches its docstring. ✓

### `moveTo(targX, targY, endAngle, direction)`
Top-level motion primitive. `direction` is a string; only the exact literal `"reverse"` selects
reverse, anything else silently means forward.

Declared gains (all `1`, untuned): `pTurningComponent`, `dTurningComponent`, `pMoveComponent`,
`dMoveComponent`.

**Phase 1 — turn in place** (live):
```
loop forever:
    robotAngle           = inertial_1.heading(DEGREES)                  # [0, 360)
    robotAngleChangeRate = inertial_1.gyro_rate(AxisType.ZAXIS, DPS)    # yaw rate ✓
    targetAngle          = calculateTargetAngle(targX, targY, dirBool)  # loop-invariant
    MTD                  = getMTD(targetAngle, endAngle, robotAngle)    # re-wrapped to robotAngle

    leftSpeedRaw  = pTurningComponent·(MTD − robotAngle)   # P term
    leftSpeedRaw += dTurningComponent·robotAngleChangeRate # D term — WRONG SIGN. Defect L0
    rightSpeedRaw = −leftSpeedRaw                          # opposite sides ⇒ spin in place

    drivetrain(*linearize(leftSpeedRaw, rightSpeedRaw))    # percent in/out ✓
    print(...)                                             # 2 prints/iteration

    if MTD − 0.5 ≤ robotAngle ≤ MTD + 0.5:                 # wrapping correct ✓
        print("Done Turning"); break
    wait(10, MSEC)                                         # ✓
drivetrain(0, 0)
```
Because `getMTD` re-wraps `MTD` to within ±180 of the *current* `robotAngle` on every iteration, the
exit comparison stays valid across the 0/360 seam. Verified: at `robotAngle = 350` with a true target
of 10°, `MTD` evaluates to `370`, giving a correct 20° error rather than −340°.

Still no **timeout** and no **settle counter**, so the loop can run forever if it never lands in the
window (Defects **L4**, **L10**).

**Phase 2 — drive to the point:** written but **entirely commented out** (wrapped in a `"""` string
literal starting at line 287). So `moveTo` currently only ever turns; it never translates. The dead
code still carries the original P/D overwrite bug (`leftSpeedRaw = dMoveComponent` clobbers the P
term with the literal `1`) and still reads `AxisType.XAXIS`. Fix both before un-commenting.

### `pre_autonomous()` / `autonomous()` / `user_control()`
`autonomous()` starts the odometry thread — `Thread(position, (0, 0, 0, 2.75))`, positional tuple,
correct for the VEX API — then runs `moveTo(10, 10, 45, "forward")`. From `(0,0)` that is a 45°
target with a 45° end angle, so `MTD == targetAngle == 45`: a straight approach.
`user_control()` is an empty 20 ms idle loop.

Module scope now ends at `pre_autonomous()`; the bare `autonomous()` call is gone, so **autonomous
only runs under field/competition-switch control.** Bench testing needs a competition switch or a
temporary direct call.

---

## 5. Defect register

Status as of the latest edit pass. Ordered by how badly each one breaks the run.

| ID | Defect | Status |
|---|---|---|
| L0 | D term sign | ✅ fixed |
| L1 | `math` not imported | ✅ fixed |
| L2 | curve input/output scaling | ✅ fixed |
| L3 | target angle ignores robot position | ✅ fixed |
| L3b | reverse negates before subtracting | ✅ fixed |
| L4 | turn-loop exit condition | ⚠️ wrapping ✓, `wait()` ✓ — **no timeout/settle** |
| L5 | odometry thread | ⚠️ started ✓, inches ✓, `wait()` ✓ — **L5b/L5c open** |
| L6 | gyro axis | ✅ fixed (live loop) |
| L7 | `autonomous()` called directly | ✅ fixed |
| L8 | inertial never calibrated | ❌ **open** |
| L9 | ports unverified | ⚠️ tracking wheel confirmed; **inertial PORT21 unverified** |
| L10 | deadzone ⊃ exit window | ✅ fixed |
| L11 | `from concurrent.futures import thread` | ✅ fixed |

### L11 — ✅ Fixed
`from concurrent.futures import thread` deleted. It was an IDE auto-import that fired when `Thread`
was typed; `concurrent.futures` is CPython-only and does not exist in VEX V5 MicroPython, so it
raised `ImportError` before any robot code ran. `Thread` comes from `from vex import *`.

### L10 — ✅ Fixed: deadzone was wider than the exit window
The deadzone (`d = 1`) zeroes the motors whenever `|leftSpeedRaw| < 1`. With `pTurningComponent = 1`,
`leftSpeedRaw` **is** the angle error in degrees when the robot is at rest. The exit test needs
`|error| ≤ 0.5`. So for `0.5 < |error| < 1.0`:

| error | motor output | can exit? |
|---|---|---|
| 0.4° | 0 % | ✅ yes |
| **0.6°** | **0 %** | **❌ no** |
| **0.9°** | **0 %** | **❌ no** |
| 1.0° | 2 % | ✅ (moves again) |

The robot was commanded to stop, so the error never changed, so the loop never exited. This only
became reachable once L2 corrected the output scale.

**Invariant to preserve:** `d / pTurningComponent < turnExitWindow`. Raising `pTurningComponent` to
`4` satisfies it (`1/4 = 0.25 < 0.5`) and is what L0 wanted anyway. The window is now the named
`turnExitWindow` variable rather than a hardcoded `0.5`, so the two can't drift apart silently.
Verified post-fix: a 0.51° error produces 2.95% output, so the robot can always move until it is
genuinely inside the window.

### L3b — ✅ Fixed: reverse direction negated the wrong quantity
`calculateTargetAngle` now subtracts first and negates the **relative vector**:
```python
deltaX = targX - x
deltaY = targY - y
if directionBool == False:
    deltaX *= -1
    deltaY *= -1
targetAngle = m.atan2(deltaX, deltaY)
```
The old form negated the absolute target *before* subtracting robot position, which differs by
`2·(x, y)`. Verified: robot at `(−3, 8)` targeting `(10, 10)` in reverse gave **−158.75°** instead of
**−98.75°**. It agreed at `(0,0)` and along the `x = y` diagonal, so bench-testing from the origin
would never have caught it.

### L0 — ✅ Fixed: D term sign
```python
leftSpeedRaw  = pTurningComponent*(MTD-robotAngle)      # P term
leftSpeedRaw -= dTurningComponent*robotAngleChangeRate  # D term, now subtracted
```
With error `e = MTD − θ` and `MTD` roughly constant, `ė = −θ̇`, so damping is `−Kd·θ̇`. The old `+=`
fed rotation back positively: the faster the robot spun, the harder it was told to spin.

Gains are now `Kp = 4`, `Kd = 0.4`. `Kd/Kp = 0.1 s` is a "lookahead time" — the controller aims at
where the robot will be 0.1 s from now. **These are starting points, not tuned values.** A rough
second-order sim (arbitrary inertia/damping constants, so treat the numbers as directional only)
gives: old `Kp=1, Kd=1` with `+=` → runaway past 282°; same gains with `-=` → creeps and never
settles; `Kp=4, Kd=0.4` → settles in ~1.2 s with ~5° overshoot. Expect to raise `Kd` on the real
robot if overshoot is bad.

The commented Phase 2 at [src/main.py:309-310](src/main.py#L309-L310) still has the original
overwrite (`leftSpeedRaw = dMoveComponent`, clobbering P with the literal `1`) and still reads
`AxisType.XAXIS`. Dead code today; fix before un-commenting.

### L1 — ✅ Fixed
[src/main.py:126](src/main.py#L126) and [src/main.py:128](src/main.py#L128) now use `m.atan2` and
`m.pi`, matching the `import math as m` on line 28. No dependency on `from vex import *` re-exporting
a `math` binding.

### L2 — ✅ Fixed
[src/main.py:203](src/main.py#L203) normalizes on entry (`input /= 100`) and
[src/main.py:216](src/main.py#L216) rescales on exit (`return y*100`). The round trip is consistent
and `linearize` now matches its docstring. Verified: `f(45) = 42.70 %`, `f(90) = 88.71 %`,
`f(100) = 99.99 %`. Values above 100 clamp in firmware, which is the right shape.

### L3 — ✅ Fixed for forward travel
[src/main.py:129](src/main.py#L129) now computes `m.atan2(targX-x, targY-y)` — the vector from the
robot to the target. Parameters were also renamed `targX`/`targY`, removing the shadowing of the
`x`/`y` globals. Correct in the forward case. **The reverse case regressed — see L3b.**

### L4 — ⚠️ Wrapping and pacing fixed; no timeout or settle check
[src/main.py:278](src/main.py#L278) tests `MTD` instead of `targetAngle`, which resolved the
range-mismatch hang: `getMTD` re-wraps `MTD` to within ±180 of the current `robotAngle` every
iteration, so the comparison is valid across the 0/360 seam. Verified at
`robotAngle ∈ {350, 359, 0, 5}`. [src/main.py:282](src/main.py#L282) adds `wait(10, MSEC)`, giving
the D term a consistent timestep, and `print("Done Turning")` now sits before the `break` ✓.

Still missing:
- **No timeout.** The loop can still run forever — via L10's dead band, or simply by never converging
  while L0 stands. This is the single cheapest guard against losing a match to a hang.
- **No settle requirement.** A ±0.5° window can be crossed between 10 ms samples. Require the robot
  to hold the window for N consecutive iterations before breaking.

### L5 — ⚠️ Thread started and units fixed; two integration bugs remain
- **L5a** ✅ [src/main.py:320](src/main.py#L320) `Thread(position, (0, 0, 0, 2.75))` — positional
  tuple, 4 elements matching the 4 parameters. Correct for the VEX API (the kwarg is `arg`, not
  `args`, and native constructors may reject kwargs entirely).
- **L5d** ✅ [src/main.py:99-100](src/main.py#L99-L100) now applies `/360*trackingwheelDiamiter*m.pi`.
  Left-to-right evaluation gives `sin(angle) · (Δ/360) · D · π`, which is the correct arc length.
  `x`/`y` are finally in inches.
- ✅ `wait(10, MSEC)` added at [src/main.py:104](src/main.py#L104).
- **L5b** ❌ `angle` is still captured from `startingAngle` at
  [src/main.py:92](src/main.py#L92) and **never updated**. Every displacement is projected onto the
  starting heading, so the moment the robot turns, `x`/`y` diverge from reality. Needs
  `angle = inertial_1.heading(DEGREES)` at the top of the loop.
- **L5c** ❌ `.angle()` returns `[0, 360)` and **wraps**; the delta jumps by ∓360 every revolution,
  injecting a ~8.6 inch phantom step per wrap at `D = 2.75`. Also `set_position(0, DEGREES)` resets
  what `.position()` reports, **not** what `.angle()` reports — verified in the V5 SDK stub — so
  `previousTrackingAngle = 0` doesn't match the first reading either. Use `.position(DEGREES)`
  (accumulating) throughout.
- Nit: each iteration calls `.angle()` three times, sampling the sensor at three slightly different
  instants. Read once into a local and reuse it.

### L6 — ✅ Fixed (live loop)
[src/main.py:259](src/main.py#L259) now reads `AxisType.ZAXIS`. Confirmed against the V5 SDK: `XAXIS`
is **roll**, `ZAXIS` is **yaw** — correct for turn rate on a flat-mounted sensor. The commented
Phase 2 block still has `XAXIS` at [src/main.py:287](src/main.py#L287).

### L7 — ✅ Fixed
The bare `autonomous()` call is gone; module scope ends at `pre_autonomous()`. Autonomous now runs
only under `Competition` control, so it can no longer fire twice or spawn two odometry threads.
Note the side effect: **bench testing now requires a competition switch** (or a temporary direct
call you remember to remove).

### L8 — ❌ Inertial sensor is never calibrated
`inertial_1.calibrate()` / `while inertial_1.is_calibrating(): wait(...)` are never called (both exist
in the SDK). Every heading reading is therefore untrustworthy, and the whole turn loop depends on it.
Calibration must finish in `pre_autonomous()` before any motion — and note `pre_autonomous` already
burns `wait(1, SECONDS)` doing nothing, which is most of the budget calibration needs.

### L9 — ⚠️ Inertial port still unverified
`Rotation(Ports.PORT4)` is now confirmed real (commit-log entry, 7/26/26). `Inertial(Ports.PORT21)`
has not been confirmed. A wrong port yields a heading that never changes — which, with no timeout,
means the turn loop spins the robot forever. Bench check, not a code check.

### Minor
- The turn loop prints twice per 10 ms iteration. That is a lot of USB traffic; consider a telemetry
  counter that prints every Nth pass.
- `moveTo` still declares `global x, y` without using them (only the commented Phase 2 does).
- `inputCurve`'s parameter `input` shadows the builtin.
- `linearize` declares `global a,b,c,d,p` but only reads them.
- Direction is a magic string; any typo silently means "forward".
- `elevationL/R` and `digital_out_a/b` are declared but unused in the auto project.
- `import time` (line 26) is unused.

---

## 6. Remaining repair order

The turn loop should now converge. What's left, in order:

1. **L8** — calibrate the inertial in `pre_autonomous()`, inside the `wait(1, SECONDS)` it already
   burns doing nothing. Every heading reading is untrustworthy until this exists, and the entire turn
   loop is built on `heading()`.
2. **L9** — confirm `Inertial(Ports.PORT21)` on the robot. A wrong port gives a heading that never
   changes, and with no timeout the robot would spin until the match ends.
3. **L4** — add a timeout and a settle counter. The timeout is the backstop that turns any future
   convergence bug into a lost point instead of a lost match.
4. **L5b** / **L5c** — update `angle` from the inertial each pass, and switch `.angle()` →
   `.position()`. Until then `x`/`y` are wrong the moment the robot turns, which silently corrupts
   `calculateTargetAngle`.
5. Tune `pTurningComponent` / `dTurningComponent` on the robot. The current `4` / `0.4` are
   analytically safe, not empirically tuned.
6. Un-comment Phase 2, carrying its own L0/L6 fixes.

**Bench-test gate:** 1 and 2 are the ones to do before the first powered run — without them a wrong
heading looks exactly like a tuning problem and will waste a lot of time.
