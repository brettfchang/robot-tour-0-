# Encoder-Based Odometry with PID Control

## Overview

Goal: Precise 90° turns and 500mm straight-line movements using encoder feedback and PID control on Arduino.

---

## 1. Encoder Setup

### Hardware
- Use Arduino hardware interrupt pins (2 and 3 on Uno)
- Count on CHANGE (both rising and falling edges) to double resolution

### Software
- Declare encoder counts as `volatile long` for ISR safety
- **Atomic reads required:** Disable interrupts when reading 32-bit encoder values from main loop (AVR is 8-bit, reads are not atomic)

```cpp
noInterrupts();
long left = enc_left;
long right = enc_right;
interrupts();
```

### Distance Calculation
```
mm_per_count = (PI * wheel_diameter) / counts_per_rev
distance_mm = encoder_counts * mm_per_count
```

---

## 2. Timing Architecture

**Critical:** PID requires consistent loop timing.

- **Target loop rate:** 50Hz (20ms period)
- Use `micros()` for timing, not `millis()` (better precision)
- Calculate actual `dt` each loop (don't assume fixed)
- Non-blocking loop structure

```cpp
unsigned long last_update = 0;
const unsigned long LOOP_PERIOD_US = 20000; // 50Hz

void loop() {
  unsigned long now = micros();
  if (now - last_update >= LOOP_PERIOD_US) {
    float dt = (now - last_update) / 1000000.0; // seconds
    last_update = now;
    // ... PID update here
  }
}
```

---

## 3. Odometry

### For Straight Driving
Simple distance tracking:
```
total_distance = (left_dist + right_dist) / 2
```

### For Turning (in place)
Heading from encoder difference:
```
heading_change = (right_dist - left_dist) / wheel_base
```

**Note:** For 90° turns, break into small increments or use arc kinematics to avoid linearization error. Since we're turning in place (tank turn), encoder difference directly gives rotation.

### Full Position Tracking (if needed later)
Use mid-point heading for arc integration:
```
theta_mid = theta + delta_theta / 2
x += distance * cos(theta_mid)
y += distance * sin(theta_mid)
theta += delta_theta
```

---

## 4. PID Controller Design

### 4.1 Cascaded Control Architecture

Uses inner velocity loops for consistent motor response regardless of battery/load.

```
┌─────────────────────────────────────────────────────────────────┐
│  OUTER LOOP (Position)                                          │
│  ┌─────────────┐      ┌─────────────┐                          │
│  │ Position    │      │ Heading     │                          │
│  │ Error       │─────▶│ Error       │                          │
│  └─────────────┘      └─────────────┘                          │
│         │                    │                                  │
│         ▼                    ▼                                  │
│  target_velocity      velocity_differential                     │
│         │                    │                                  │
│         └────────┬───────────┘                                  │
│                  ▼                                              │
│         ┌───────────────┐                                       │
│         │ left_target_v │ = target_velocity - differential      │
│         │ right_target_v│ = target_velocity + differential      │
│         └───────────────┘                                       │
└─────────────────────────────────────────────────────────────────┘
                           │
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│  INNER LOOP (Velocity) - one per wheel                          │
│  ┌─────────────┐      ┌─────────────┐                          │
│  │ Left Vel    │      │ Right Vel   │                          │
│  │ PID         │      │ PID         │                          │
│  └─────────────┘      └─────────────┘                          │
│         │                    │                                  │
│         ▼                    ▼                                  │
│     left_PWM            right_PWM                               │
└─────────────────────────────────────────────────────────────────┘
```

**Inner Loop (Velocity):** Runs every update, fast response
- One PID per wheel
- Input: actual wheel velocity (from encoder delta / dt)
- Setpoint: target velocity from outer loop
- Output: PWM to motor
- Handles: battery voltage, friction, motor differences

**Outer Loop (Position/Heading):** Runs every update
- Position PID → target forward velocity
- Heading PID → velocity differential (left vs right)
- Combines into per-wheel target velocities

### 4.2 Velocity Calculation

Each update, calculate actual wheel velocities:
```cpp
// Encoder deltas since last update
long delta_left = enc_left - enc_left_prev;
long delta_right = enc_right - enc_right_prev;

// Velocity in mm/s
float vel_left = (delta_left * mm_per_count) / dt;
float vel_right = (delta_right * mm_per_count) / dt;

// Optional: low-pass filter to reduce noise
vel_left_filtered = alpha * vel_left + (1 - alpha) * vel_left_filtered;
vel_right_filtered = alpha * vel_right + (1 - alpha) * vel_right_filtered;
```

### 4.3 Control Flow (Each Update)

```cpp
// 1. Update odometry
updateOdometry();

// 2. Outer loop: position/heading -> target velocities
float pos_error = calculatePositionError();
float heading_error = calculateHeadingError();

float target_velocity = positionPID(pos_error);
float velocity_diff = headingPID(heading_error);

float target_vel_left = target_velocity - velocity_diff;
float target_vel_right = target_velocity + velocity_diff;

// 3. Inner loop: target velocity -> PWM
float vel_error_left = target_vel_left - actual_vel_left;
float vel_error_right = target_vel_right - actual_vel_right;

int pwm_left = velocityPID_left(vel_error_left);
int pwm_right = velocityPID_right(vel_error_right);

// 4. Apply to motors
drive(pwm_left, pwm_right);
```

### 4.4 PID Formula
```
error = setpoint - measurement
P = Kp * error
I = I + Ki * error * dt         (with anti-windup clamping)
D = Kd * (derivative_filtered)  (filtered, on measurement not error)
output = P + I + D
```

### 4.5 Critical Implementation Details

#### Derivative Filtering
Raw derivative is noisy. Apply low-pass filter:
```cpp
float alpha = 0.7; // tuning parameter
derivative_filtered = alpha * derivative_new + (1 - alpha) * derivative_prev;
```

#### Derivative on Measurement (not error)
Prevents "derivative kick" when setpoint changes:
```cpp
// BAD: derivative = (error - last_error) / dt
// GOOD: derivative = -(measurement - last_measurement) / dt
```

#### Anti-Windup (Integral Clamping)
Prevents integral from growing unbounded when motor is saturated:
```cpp
integral += Ki * error * dt;
integral = constrain(integral, -MAX_INTEGRAL, MAX_INTEGRAL);
```

#### Output Limiting
```cpp
output = constrain(output, -255, 255);
```

#### Motor Deadband
Motors don't move below a certain PWM. Compensate:
```cpp
if (output > 0) output = map(output, 0, 255, DEADBAND, 255);
if (output < 0) output = map(output, -255, 0, -255, -DEADBAND);
```

### 4.6 Tuning Strategy (Cascaded)

**Tune inner loops (velocity) FIRST, then outer loops (position).**

#### Step 1: Velocity PIDs (inner loop)
- Disable outer loop (set target velocity manually, e.g., 100 mm/s)
- Tune left and right velocity PIDs independently
- Start with P only, add D if needed, I rarely needed
- Goal: wheels track target velocity accurately

#### Step 2: Position PID (outer loop)
- Enable outer loop with tuned velocity PIDs
- Tune position P first
- Add D for faster settling
- Add I only if steady-state error persists

#### Step 3: Heading PID (outer loop)
- Tune separately during turns
- Similar process: P → D → I

**Starting values to try:**

Velocity PID (inner):
- Kp: 0.5 - 2.0
- Ki: 0.0 - 0.05
- Kd: 0.0 - 0.1

Position PID (outer):
- Kp: 1.0 - 5.0
- Ki: 0.0 - 0.1
- Kd: 0.0 - 0.5

Heading PID (outer):
- Kp: 2.0 - 10.0
- Ki: 0.0 - 0.1
- Kd: 0.0 - 1.0

Velocity filter alpha: 0.7

---

## 5. Adaptive Velocity (Load Detection)

Automatically detect load from motor response and adjust max velocity.

### Concept

When carrying a load (bottle), motors need more PWM to achieve the same velocity. We can detect this and limit speed accordingly.

```
load_factor = PWM_effort / actual_velocity

Low load_factor  → light/empty  → allow faster speeds
High load_factor → heavy/loaded → limit speed for stability
```

### Implementation

```cpp
// Configuration
const float LOAD_FACTOR_EMPTY = 1.0;    // calibrate: PWM/velocity when empty
const float LOAD_FACTOR_LOADED = 2.0;   // calibrate: PWM/velocity when carrying bottle
const float MAX_VEL_EMPTY = 200.0;      // mm/s
const float MAX_VEL_LOADED = 80.0;      // mm/s

// State
float load_factor_filtered = LOAD_FACTOR_EMPTY;
const float LOAD_FILTER_ALPHA = 0.1;    // slow filter for stability

void updateLoadEstimate() {
  // Only estimate when moving (avoid division by zero)
  float avg_velocity = (abs(vel_left) + abs(vel_right)) / 2.0;
  float avg_pwm = (abs(pwm_left) + abs(pwm_right)) / 2.0;

  if (avg_velocity > 20.0) {  // minimum velocity threshold
    float load_factor = avg_pwm / avg_velocity;
    // Low-pass filter for stability
    load_factor_filtered = LOAD_FILTER_ALPHA * load_factor
                         + (1 - LOAD_FILTER_ALPHA) * load_factor_filtered;
  }
}

float getAdaptiveMaxVelocity() {
  // Linear interpolation between empty and loaded speeds
  float t = (load_factor_filtered - LOAD_FACTOR_EMPTY)
          / (LOAD_FACTOR_LOADED - LOAD_FACTOR_EMPTY);
  t = constrain(t, 0.0, 1.0);
  return MAX_VEL_EMPTY + t * (MAX_VEL_LOADED - MAX_VEL_EMPTY);
}
```

### Integration with Position PID

```cpp
// In control loop:
updateLoadEstimate();
float max_vel = getAdaptiveMaxVelocity();

// Clamp position PID output
target_velocity = positionPID(pos_error);
target_velocity = constrain(target_velocity, -max_vel, max_vel);
```

### Calibration

1. Run robot empty, log average `pwm / velocity` → `LOAD_FACTOR_EMPTY`
2. Run robot with bottle, log average `pwm / velocity` → `LOAD_FACTOR_LOADED`
3. Tune `MAX_VEL_LOADED` for stability when carrying

### Notes

- Use slow filter (low alpha) to avoid rapid speed changes
- Only measure when velocity > threshold (steady motion)
- Battery voltage changes also affect load factor - recalibrate if needed
- Can add manual override: `setCarrying(true)` to force slow mode

---

## 6. Movement Functions (Absolute Positioning)

Uses odometry (x, y, theta) for error calculation. Accumulated errors from previous moves get corrected automatically.

### goToPosition(target_x, target_y)

Drive to an absolute position while maintaining current heading.

```
1. Loop at 50Hz:
   a. Update odometry (read encoders, update x, y, theta)
   b. Calculate distance error:
      - dx = target_x - x
      - dy = target_y - y
      - distance_error = dx * cos(theta) + dy * sin(theta)  // error along heading
      - lateral_error = -dx * sin(theta) + dy * cos(theta)  // error perpendicular to heading
   c. Distance PID -> base_speed
   d. Heading correction from lateral_error -> differential adjustment
   e. left_motor = base_speed - heading_correction
   f. right_motor = base_speed + heading_correction
   g. Check if within tolerance for N iterations -> done
   h. Check timeout -> abort
2. Brake motors
```

### turnToHeading(target_theta)

Turn to an absolute heading (in degrees or radians).

```
1. Loop at 50Hz:
   a. Update odometry (read encoders, update x, y, theta)
   b. Calculate heading error:
      - error = target_theta - theta
      - Normalize to [-180, 180] or [-PI, PI]
   c. Heading PID -> turn_speed
   d. left_motor = -turn_speed
   e. right_motor = +turn_speed
   f. Check if within tolerance for N iterations -> done
   g. Check timeout -> abort
2. Brake motors
```

### Helper: Relative Wrappers (optional convenience)

```cpp
void driveStraight(float distance_mm) {
  float target_x = x + distance_mm * cos(theta);
  float target_y = y + distance_mm * sin(theta);
  goToPosition(target_x, target_y);
}

void turn(float degrees) {
  turnToHeading(theta + degrees);
}
```

---

## 7. Completion Criteria

### Tolerances
- **Distance:** ±5mm (adjustable)
- **Heading:** ±2° (adjustable)

### Settling Detection
Don't declare "done" on first reading in tolerance. Require N consecutive readings:
```cpp
if (abs(error) < tolerance) {
  settled_count++;
  if (settled_count >= SETTLED_THRESHOLD) done = true;
} else {
  settled_count = 0;
}
```

### Safety Timeout
Abort if movement takes too long (motor stall, stuck, etc.):
```cpp
if (millis() - start_time > TIMEOUT_MS) {
  brake();
  return ERROR_TIMEOUT;
}
```

---

## 7. Code Structure

```
robot-tour.ino
├── Configuration (pins, measurements, PID gains)
├── Global state (encoder counts, odometry, PID state)
├── setup() - pins, interrupts, serial
├── loop() - button check, timing loop
├── Encoder ISRs - leftISR(), rightISR()
├── Odometry - updateOdometry()
├── PID - computePID()
├── Movement - driveStraight(), turn()
├── Motor control - drive(), brake(), enableMotors()
└── Utilities - readEncodersAtomic()
```

---

## 8. Measurements Needed

- [ ] Wheel diameter (mm)
- [ ] Wheel base / distance between wheel centers (mm)
- [ ] Encoder counts per wheel revolution
- [ ] Encoder type: single-channel or quadrature?
- [ ] Motor deadband (minimum PWM to start moving)

---

## 9. Testing & Tuning Order

### Phase 1: Verify Hardware
1. **Verify encoders** - print raw counts, ensure both work and count correctly
2. **Verify distance calc** - push robot 100mm by hand, check reported distance
3. **Verify velocity calc** - push robot, check velocity readings are reasonable
4. **Verify heading calc** - rotate robot 90° by hand, check reported angle

### Phase 2: Tune Velocity PIDs (Inner Loop)
5. **Set fixed target velocity** - e.g., 100 mm/s, disable outer loop
6. **Tune left velocity PID** - P first, goal: tracks target smoothly
7. **Tune right velocity PID** - same process
8. **Verify both wheels match** - set same target, both should run at same speed

### Phase 3: Tune Position PID (Outer Loop)
9. **Enable position PID** - drive straight 500mm
10. **Tune position P** - adjust until minimal overshoot
11. **Add position D** - if oscillation or slow settling
12. **Verify straight line** - heading correction should keep robot straight

### Phase 4: Tune Heading PID (Outer Loop)
13. **Test 90° turns** - tune heading P
14. **Add heading D** - for clean stops without oscillation
15. **Verify turn accuracy** - both directions

### Phase 5: Integration
16. **Test sequences** - multiple moves, verify error correction
17. **Fine-tune tolerances** - balance precision vs. completion time
18. **Add I terms** - only if persistent steady-state error remains

---

## 10. Potential Improvements (Future)

- **Trapezoidal velocity profile** - ramp up/down speed for smoother motion
- **Feedforward control** - estimate base PWM from known motor characteristics
- **IMU fusion** - combine gyroscope with encoder heading for drift correction
- **Position tracking** - full x,y,theta odometry for complex paths
