#!/usr/bin/env python3
"""
Sequence Move Visualizer
Shows per-move progress (distance/heading) plus velocity and PWM graphs
"""

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from collections import deque
import re
import sys

BAUD_RATE = 115200
HISTORY_LENGTH = 200

def find_arduino_port():
    """Auto-detect Arduino serial port"""
    ports = serial.tools.list_ports.comports()
    arduino_keywords = ['arduino', 'usbmodem', 'usbserial', 'ch340', 'cp210', 'ftdi', 'acm']

    for port in ports:
        port_lower = (port.device + ' ' + (port.description or '')).lower()
        for keyword in arduino_keywords:
            if keyword in port_lower:
                return port.device

    if ports:
        print("Available ports:")
        for port in ports:
            print(f"  {port.device} - {port.description}")
        return ports[0].device

    return None

# Current move state
current_move = "Idle"
move_target = 0
move_progress = 0
current_heading = 0
target_heading = 0

# Velocity/PWM history
time_history = deque(maxlen=HISTORY_LENGTH)
vel_left_history = deque(maxlen=HISTORY_LENGTH)
vel_right_history = deque(maxlen=HISTORY_LENGTH)
target_vel_left_history = deque(maxlen=HISTORY_LENGTH)
target_vel_right_history = deque(maxlen=HISTORY_LENGTH)
pwm_left_history = deque(maxlen=HISTORY_LENGTH)
pwm_right_history = deque(maxlen=HISTORY_LENGTH)
ff_left_history = deque(maxlen=HISTORY_LENGTH)
ff_right_history = deque(maxlen=HISTORY_LENGTH)
time_counter = 0

# Move history for the sequence view
move_history = []  # List of (move_type, target, achieved)
current_move_start_time = 0

# Current values
current_vel_left = 0
current_vel_right = 0
current_target_vel_left = 0
current_target_vel_right = 0
current_pwm_left = 0
current_pwm_right = 0
current_ff_left = 0
current_ff_right = 0

def parse_move_start(line):
    """Parse move start messages"""
    global current_move, move_target, move_progress, current_move_start_time, time_counter
    global target_heading

    # Forward/Backward
    match = re.search(r'(Forward|Backward)\s+([\d.]+)\s*mm', line)
    if match:
        current_move = match.group(1)
        move_target = float(match.group(2))
        move_progress = 0
        current_move_start_time = time_counter
        return True

    # Turn
    match = re.search(r'Turn (left|right) 90', line)
    if match:
        current_move = f"Turn {match.group(1)}"
        move_target = 90
        move_progress = 0
        current_move_start_time = time_counter
        return True

    return False

def parse_move_done(line):
    """Parse move completion"""
    global current_move, move_history

    if line.strip() == "Done" or "Done." in line:
        if current_move != "Idle":
            move_history.append((current_move, move_target, move_progress))
            if len(move_history) > 10:
                move_history.pop(0)
        current_move = "Idle"
        return True
    return False

def parse_velocity(line):
    """Parse velocity data"""
    global current_vel_left, current_vel_right, current_target_vel_left, current_target_vel_right
    global current_pwm_left, current_pwm_right, current_ff_left, current_ff_right, time_counter
    global move_progress

    # Match: tgtVelL:X tgtVelR:Y velL:Z velR:W pwmL:A pwmR:B ffL:C ffR:D
    match = re.search(r'tgtVelL:\s*([-\d.]+)\s*tgtVelR:\s*([-\d.]+)\s*velL:\s*([-\d.]+)\s*velR:\s*([-\d.]+)\s*pwmL:\s*([-\d.]+)\s*pwmR:\s*([-\d.]+)\s*ffL:\s*([-\d.]+)\s*ffR:\s*([-\d.]+)', line)
    if match:
        current_target_vel_left = float(match.group(1))
        current_target_vel_right = float(match.group(2))
        current_vel_left = float(match.group(3))
        current_vel_right = float(match.group(4))
        current_pwm_left = float(match.group(5))
        current_pwm_right = float(match.group(6))
        current_ff_left = float(match.group(7))
        current_ff_right = float(match.group(8))

        time_history.append(time_counter)
        target_vel_left_history.append(current_target_vel_left)
        target_vel_right_history.append(current_target_vel_right)
        vel_left_history.append(current_vel_left)
        vel_right_history.append(current_vel_right)
        pwm_left_history.append(current_pwm_left)
        pwm_right_history.append(current_pwm_right)
        ff_left_history.append(current_ff_left)
        ff_right_history.append(current_ff_right)

        # Estimate progress from velocity (rough integration)
        avg_vel = (current_vel_left + current_vel_right) / 2
        move_progress += abs(avg_vel) * 0.01  # 10ms sample

        time_counter += 1
        return True

    # Match turn format: turnVel:X velL:Y velR:Z pwmL:A pwmR:B ffL:C ffR:D
    match = re.search(r'turnVel:\s*([-\d.]+)\s*velL:\s*([-\d.]+)\s*velR:\s*([-\d.]+)\s*pwmL:\s*([-\d.]+)\s*pwmR:\s*([-\d.]+)\s*ffL:\s*([-\d.]+)\s*ffR:\s*([-\d.]+)', line)
    if match:
        turn_vel = float(match.group(1))
        current_vel_left = float(match.group(2))
        current_vel_right = float(match.group(3))
        current_pwm_left = float(match.group(4))
        current_pwm_right = float(match.group(5))
        current_ff_left = float(match.group(6))
        current_ff_right = float(match.group(7))
        current_target_vel_left = turn_vel
        current_target_vel_right = -turn_vel

        time_history.append(time_counter)
        target_vel_left_history.append(current_target_vel_left)
        target_vel_right_history.append(current_target_vel_right)
        vel_left_history.append(current_vel_left)
        vel_right_history.append(current_vel_right)
        pwm_left_history.append(current_pwm_left)
        pwm_right_history.append(current_pwm_right)
        ff_left_history.append(current_ff_left)
        ff_right_history.append(current_ff_right)
        time_counter += 1
        return True

    return False

def parse_kff(line):
    """Parse adaptive coefficient updates"""
    match = re.search(r'kffL=([\d.]+)\s*kffR=([\d.]+)', line)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None

# Set up the plot (2 rows x 3 cols)
fig = plt.figure(figsize=(16, 8))

# Move status (top left)
ax_move = fig.add_subplot(2, 3, 1)
ax_move.set_xlim(0, 1)
ax_move.set_ylim(0, 1)
ax_move.axis('off')
ax_move.set_title('Current Move')

# Move history (bottom left)
ax_history = fig.add_subplot(2, 3, 4)
ax_history.axis('off')
ax_history.set_title('Move History')

# Velocity plots (middle column)
ax_vel_left = fig.add_subplot(2, 3, 2)
ax_vel_left.grid(True, alpha=0.3)
ax_vel_left.set_ylabel('Velocity (mm/s)')
ax_vel_left.set_title('Left Wheel Velocity')

ax_vel_right = fig.add_subplot(2, 3, 5)
ax_vel_right.grid(True, alpha=0.3)
ax_vel_right.set_xlabel('Sample')
ax_vel_right.set_ylabel('Velocity (mm/s)')
ax_vel_right.set_title('Right Wheel Velocity')

# PWM plots (right column)
ax_pwm_left = fig.add_subplot(2, 3, 3)
ax_pwm_left.grid(True, alpha=0.3)
ax_pwm_left.set_ylabel('PWM')
ax_pwm_left.set_title('Left Wheel PWM')

ax_pwm_right = fig.add_subplot(2, 3, 6)
ax_pwm_right.grid(True, alpha=0.3)
ax_pwm_right.set_xlabel('Sample')
ax_pwm_right.set_ylabel('PWM')
ax_pwm_right.set_title('Right Wheel PWM')

# Velocity plot elements
vel_left_line, = ax_vel_left.plot([], [], 'b-', linewidth=1, label='Actual')
target_left_line, = ax_vel_left.plot([], [], 'r--', linewidth=1, label='Target')
ax_vel_left.legend(loc='upper right')

vel_right_line, = ax_vel_right.plot([], [], 'b-', linewidth=1, label='Actual')
target_right_line, = ax_vel_right.plot([], [], 'r--', linewidth=1, label='Target')
ax_vel_right.legend(loc='upper right')

# PWM plot elements
pwm_left_line, = ax_pwm_left.plot([], [], 'g-', linewidth=1, label='PWM')
ff_left_line, = ax_pwm_left.plot([], [], 'm-', linewidth=1, alpha=0.7, label='Feedforward')
ax_pwm_left.legend(loc='upper right')

pwm_right_line, = ax_pwm_right.plot([], [], 'g-', linewidth=1, label='PWM')
ff_right_line, = ax_pwm_right.plot([], [], 'm-', linewidth=1, alpha=0.7, label='Feedforward')
ax_pwm_right.legend(loc='upper right')

# Text elements for move display
move_text = None
progress_bar = None
history_text = None
kff_left = 0.8
kff_right = 0.8

# Serial connection
ser = None

def init_serial(port=None):
    global ser

    if port is None:
        port = find_arduino_port()

    if port is None:
        print("No serial ports found!")
        return False

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=0.1)
        print(f"Connected to {port}")
        return True
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        return False

def update(frame):
    global move_text, progress_bar, history_text, kff_left, kff_right

    # Read serial data
    if ser and ser.in_waiting:
        try:
            lines = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                line = line.strip()
                if line:
                    print(line)  # Echo to console
                    parse_move_start(line)
                    parse_move_done(line)
                    parse_velocity(line)
                    kff_result = parse_kff(line)
                    if kff_result:
                        kff_left, kff_right = kff_result
        except Exception as e:
            print(f"Serial error: {e}")

    # Clear and redraw move status
    ax_move.clear()
    ax_move.set_xlim(0, 1)
    ax_move.set_ylim(0, 1)
    ax_move.axis('off')
    ax_move.set_title('Current Move')

    # Move type and progress
    if current_move == "Idle":
        status = "Idle - Waiting for button press"
        progress_pct = 0
        color = 'gray'
    elif "Forward" in current_move or "Backward" in current_move:
        progress_pct = min(100, (move_progress / move_target) * 100) if move_target > 0 else 0
        status = f"{current_move}\n{move_progress:.0f} / {move_target:.0f} mm ({progress_pct:.0f}%)"
        color = 'blue' if "Forward" in current_move else 'orange'
    else:  # Turn
        progress_pct = min(100, (move_progress / move_target) * 100) if move_target > 0 else 0
        status = f"{current_move}\nProgress: {progress_pct:.0f}%"
        color = 'green' if "left" in current_move else 'red'

    ax_move.text(0.5, 0.7, status, ha='center', va='center', fontsize=14,
                 fontweight='bold', transform=ax_move.transAxes)

    # Progress bar
    ax_move.add_patch(plt.Rectangle((0.1, 0.35), 0.8, 0.15, fill=False, edgecolor='black', linewidth=2))
    ax_move.add_patch(plt.Rectangle((0.1, 0.35), 0.8 * (progress_pct / 100), 0.15,
                                     fill=True, facecolor=color, alpha=0.7))

    # Adaptive coefficient display
    ax_move.text(0.5, 0.15, f"kffL: {kff_left:.3f}  |  kffR: {kff_right:.3f}",
                 ha='center', va='center', fontsize=10, fontfamily='monospace',
                 transform=ax_move.transAxes)

    # Move history
    ax_history.clear()
    ax_history.set_xlim(0, 1)
    ax_history.set_ylim(0, 1)
    ax_history.axis('off')
    ax_history.set_title('Move History')

    if move_history:
        history_str = ""
        for i, (move_type, target, achieved) in enumerate(reversed(move_history[-5:])):
            if "Forward" in move_type or "Backward" in move_type:
                history_str += f"{move_type}: {achieved:.0f}/{target:.0f} mm\n"
            else:
                history_str += f"{move_type}: done\n"
        ax_history.text(0.1, 0.9, history_str, ha='left', va='top', fontsize=10,
                       fontfamily='monospace', transform=ax_history.transAxes)
    else:
        ax_history.text(0.5, 0.5, "No moves yet", ha='center', va='center',
                       fontsize=10, color='gray', transform=ax_history.transAxes)

    # Update velocity plots
    if len(time_history) > 0:
        times = list(time_history)

        vel_left_line.set_data(times, list(vel_left_history))
        target_left_line.set_data(times, list(target_vel_left_history))
        vel_right_line.set_data(times, list(vel_right_history))
        target_right_line.set_data(times, list(target_vel_right_history))

        pwm_left_line.set_data(times, list(pwm_left_history))
        ff_left_line.set_data(times, list(ff_left_history))
        pwm_right_line.set_data(times, list(pwm_right_history))
        ff_right_line.set_data(times, list(ff_right_history))

        # Auto-scale velocity
        vel_vals = list(vel_left_history) + list(vel_right_history) + \
                   list(target_vel_left_history) + list(target_vel_right_history)
        if vel_vals:
            min_vel = min(vel_vals) - 20
            max_vel = max(vel_vals) + 20
            ax_vel_left.set_xlim(times[0], times[-1] + 1)
            ax_vel_left.set_ylim(min_vel, max_vel)
            ax_vel_right.set_xlim(times[0], times[-1] + 1)
            ax_vel_right.set_ylim(min_vel, max_vel)

        # Auto-scale PWM
        pwm_vals = list(pwm_left_history) + list(pwm_right_history) + \
                   list(ff_left_history) + list(ff_right_history)
        if pwm_vals:
            min_pwm = min(pwm_vals) - 20
            max_pwm = max(pwm_vals) + 20
            ax_pwm_left.set_xlim(times[0], times[-1] + 1)
            ax_pwm_left.set_ylim(min_pwm, max_pwm)
            ax_pwm_right.set_xlim(times[0], times[-1] + 1)
            ax_pwm_right.set_ylim(min_pwm, max_pwm)

    return (vel_left_line, vel_right_line, target_left_line, target_right_line,
            pwm_left_line, pwm_right_line, ff_left_line, ff_right_line)

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None

    print("Sequence Move Visualizer")
    print("Usage: python visualizer_sequence.py [serial_port]")
    print()

    if port:
        print(f"Using specified port: {port}")
    else:
        print("Auto-detecting Arduino port...")

    if not init_serial(port):
        print("\nRunning in demo mode (no serial connection)")

    plt.tight_layout()
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    if ser:
        ser.close()

if __name__ == '__main__':
    main()
