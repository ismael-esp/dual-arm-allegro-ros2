# Allegro Driver Changes — Thumb (j12) Overheating Mitigation

**Date:** 2026-07-02
**Package:** `allegro_hand_cust/allegro_hand`
**Context:** The thumb-roll motor (`j12` / `j30`) loses torque after sustained time holding
the hand fully open, will not return to full-open even when commanded, and only recovers
after a hand power-cycle + driver restart. Diagnosis: the motor overheats because it draws
sustained near-max current holding the thumb up, and the firmware raises a latching
**high-temperature fault**. See the `thumb_gravity_test` node in
`my_dual_robot_cell_control/scripts/` (which ruled out gravity comp as the cause — only ~1°).

This file records exactly what changed so the driver can be reverted to its prior state.

---

## Change 1 — APPLIED: enforce the per-joint `PWM_MAX` clamp

**File:** `src/CAN_AH.cpp`, function `CAN_AH::Version4Handler::writeDevices()`
(originally around line 889).

**Why:** `PWM_MAX[]` is loaded from the `PWM_LIMIT_*` constants in `startCAN()`
(`CAN_AH.cpp:1046-1064`, e.g. `PWM_LIMIT_THUMB_ROLL = 350*1.5 = 525`) but was **never used
as a clamp**. The only torque limit was the controller's `torque_max = 0.7 Nm`
(→ `0.7 * tau_cov_const(1200) = 840` PWM). So the thumb-roll motor could be driven to 840
PWM continuously — well above the author-intended 525 ceiling — causing the overheating.
This change applies the intended per-motor ceiling.

**Trade-off:** `j12` (thumb roll) peak torque is now capped at `525/1200 ≈ 0.44 Nm`
(was `0.7 Nm`). This reduces peak thumb-roll holding torque but prevents the sustained
overcurrent that triggers the thermal fault. Other joints’ limits: roll 375, near 675,
middle 450, far 285; thumb near 405, thumb middle/far 270 (`PWM_LIMIT_* × 1.5`).

**Version 3 (`Version3Handler::writeDevices`) was NOT changed** — the deployed hands are V4.

### Original code (to restore for rollback)

```cpp
	for (int i=0; i<4;i++)
	{
		pwm[i*4+0] = (short)(cur_des[i*4+0]*tau_cov_const);
		pwm[i*4+1] = (short)(cur_des[i*4+1]*tau_cov_const);
		pwm[i*4+2] = (short)(cur_des[i*4+2]*tau_cov_const);
		pwm[i*4+3] = (short)(cur_des[i*4+3]*tau_cov_const);
		command_set_torque(bus, i, &pwm[4*i]);
	}
```

### New code (currently in place)

```cpp
	for (int i=0; i<4;i++)
	{
		// --- PWM_MAX per-joint clamp (added) ---
		for (int k=0; k<4; ++k)
		{
			int j = i*4 + k;
			short p = (short)(cur_des[j]*tau_cov_const);
			short lim = (short)PWM_MAX[j];
			if (lim > 0)
			{
				if (p >  lim) p =  lim;
				else if (p < -lim) p = -lim;
			}
			pwm[j] = p;
		}
		// --- end PWM_MAX clamp ---
		command_set_torque(bus, i, &pwm[4*i]);
	}
```

**To tune instead of revert:** raise/lower the `PWM_LIMIT_*` macros in
`include/allegro_hand/CAN_AH.h:27-35` (they feed `PWM_MAX[]`). Lower = cooler but weaker;
higher = stronger but hotter. To effectively disable the clamp without editing the loop,
set the `PWM_LIMIT_*` values very high.

---

## Change 2 — NOT APPLIED: runtime temperature / high-temp-fault telemetry

**Status: intentionally deferred — requires on-hardware validation. No code changed for this.**

**Why it was not dropped in:** The runtime read path
(`getCurrentStateComm → CAN_AH::readDataTimeStamp (CAN_AH.cpp:702) → readDevices`) reads
**exactly 4 CAN frames per cycle and requires all 4 to be finger-pose messages**;
`readDevices()` sets `emergencyStop = true` if any parsed frame is not a finger device
(`CAN_AH.cpp:817-822`). Enabling periodic temperature frames (`comm_period` temperature > 0,
set at `CAN_AH.cpp:269`, currently `{3,0,0}`) — or injecting a `HAND_INFO` query — puts
non-finger frames into that loop and **halts the hand**. Also, no parser exists for the
periodic `ID_RTR_TEMPERATURE` frame layout, so the per-motor temperature byte format is
unverified. Shipping this untested risks the Bus-Off/halt failures described in the README
(Troubleshooting: CAN bus-off).

**What already exists (safe, read-only):** at startup the driver already requests
`ID_RTR_HAND_INFO` and prints (`CAN_AH.cpp:242-246`):
- `temperature: N (celsius)` (`data[5]`)
- `high temperature fault: ON/OFF` (`data[6] & 0x02`)
- `servo status` (`data[6] & 0x01`), `internal communication fault` (`data[6] & 0x04`)

Check the driver console output at launch for the initial temperature and fault state.

**Safe design for a future, hardware-tested implementation:**
1. Make `CAN_AH::readDataTimeStamp(TPCANTimestamp&)` robust: read in a loop and only accept
   finger-pose frames into the 4 slots; for any other frame, **route it** — if it is a
   `ID_RTR_HAND_INFO` response, store `data[5]` (temp) and `data[6]` (status/fault) into new
   `VersionHandler` members; otherwise discard. (This is a strict robustness improvement.)
2. Every N control cycles, send an RTR for `ID_RTR_HAND_INFO` (single-threaded with the read,
   so no CAN race), then read the routed response.
3. Expose `getHandStatus(int& tempC, bool& highTempFault, ...)` through
   `AllegroHandDriver` and publish/log it; auto-relax the thumb (or drop to
   `GravityCompensator`) when `highTempFault` or `tempC` exceeds a threshold.
4. Validate on hardware that position streaming is unaffected before relying on it.

---

## Rollback

**Preferred (git):**
```bash
cd /mnt/ssd2/ws/Dualarm_ws
git diff -- ros2_ws_jazzy/src/allegro_hand_cust/allegro_hand/src/CAN_AH.cpp   # review
git checkout -- ros2_ws_jazzy/src/allegro_hand_cust/allegro_hand/src/CAN_AH.cpp
```

**Manual:** replace the "New code" block above with the "Original code" block in
`src/CAN_AH.cpp` (`Version4Handler::writeDevices`).

**Rebuild after any change or rollback (inside the ROS2/docker env):**
```bash
cd /home/robot/Dualarm_ws/ros2_ws_jazzy
colcon build --symlink-install --packages-select allegro_hand
source install/setup.bash
```
Then restart the hand driver (and, for a full reset of a latched thermal fault,
power-cycle the hand).

---

## Files touched by this session

- `src/CAN_AH.cpp` — Change 1 (PWM_MAX clamp) applied.
- `DRIVER_CHANGES.md` — this file (new).
- No other driver files modified. Change 2 (temperature telemetry) is documented only.
