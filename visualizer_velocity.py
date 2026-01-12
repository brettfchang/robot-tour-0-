#!/usr/bin/env python3
"""
Robot Odometry + Velocity Visualizer
Reads serial data from Arduino and plots robot position and wheel velocities in real-time
"""

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from collections import deque
import re
import sys
import math

BAUD_RATE = 115200
HISTORY_LENGTH = 200  # Number of velocity samples to keep

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

# Robot dimensions for visualization (mm)
ROBOT_LENGTH = 150
ROBOT_WIDTH = 120

# Track history
history_x = [0]
history_y = [0]
current_x = 0
current_y = 0
current_heading = 0

# Target point (if any)
target_x = None
target_y = None

# Velocity history
time_history = deque(maxlen=HISTORY_LENGTH)
vel_left_history = deque(maxlen=HISTORY_LENGTH)
vel_right_history = deque(maxlen=HISTORY_LENGTH)
target_vel_history = deque(maxlen=HISTORY_LENGTH)
time_counter = 0

# Current velocities
current_vel_left = 0
current_vel_right = 0
current_target_vel = 0

def parse_odometry(line):
    """Parse odometry from 'X: 123.45 Y: 67.89 Heading: 45.00 deg' format"""
    global current_x, current_y, current_heading

    match = re.search(r'X:\s*([-\d.]+)\s*Y:\s*([-\d.]+)\s*(?:Heading|Hdg\(gyro\)):\s*([-\d.]+)', line)
    if match:
        current_x = float(match.group(1))
        current_y = float(match.group(2))
        current_heading = float(match.group(3)) * math.pi / 180
        history_x.append(current_x)
        history_y.append(current_y)
        return True
    return False

def parse_target(line):
    """Parse target from 'Driving to: 123.45, 67.89' format"""
    global target_x, target_y

    match = re.search(r'Driving to:\s*([-\d.]+),\s*([-\d.]+)', line)
    if match:
        target_x = float(match.group(1))
        target_y = float(match.group(2))
        return True
    return False

def parse_velocity(line):
    """Parse velocity data from 'tgtVel: 100.0 velL: 95.0 velR: 98.0' format"""
    global current_vel_left, current_vel_right, current_target_vel, time_counter

    # Match straight line format: tgtVel:X velL:Y velR:Z
    match = re.search(r'tgtVel:\s*([-\d.]+)\s*velL:\s*([-\d.]+)\s*velR:\s*([-\d.]+)', line)
    if match:
        current_target_vel = float(match.group(1))
        current_vel_left = float(match.group(2))
        current_vel_right = float(match.group(3))

        time_history.append(time_counter)
        target_vel_history.append(current_target_vel)
        vel_left_history.append(current_vel_left)
        vel_right_history.append(current_vel_right)
        time_counter += 1
        return True

    # Match turn format: turnVel:X velL:Y velR:Z
    match = re.search(r'turnVel:\s*([-\d.]+)\s*velL:\s*([-\d.]+)\s*velR:\s*([-\d.]+)', line)
    if match:
        turn_vel = float(match.group(1))
        current_vel_left = float(match.group(2))
        current_vel_right = float(match.group(3))
        # For turns, left target = turnVel, right target = -turnVel
        current_target_vel = turn_vel

        time_history.append(time_counter)
        target_vel_history.append(turn_vel)  # Left wheel target
        vel_left_history.append(current_vel_left)
        vel_right_history.append(current_vel_right)
        time_counter += 1
        return True

    return False

# Set up the plot with subplots
fig = plt.figure(figsize=(14, 8))

# Position plot (left side, larger)
ax_pos = fig.add_subplot(1, 2, 1)
ax_pos.set_aspect('equal')
ax_pos.grid(True, alpha=0.3)
ax_pos.set_xlabel('X (mm)')
ax_pos.set_ylabel('Y (mm)')
ax_pos.set_title('Robot Position')

# Velocity plots (right side, stacked)
ax_vel_left = fig.add_subplot(2, 2, 2)
ax_vel_left.grid(True, alpha=0.3)
ax_vel_left.set_ylabel('Velocity (mm/s)')
ax_vel_left.set_title('Left Wheel Velocity')

ax_vel_right = fig.add_subplot(2, 2, 4)
ax_vel_right.grid(True, alpha=0.3)
ax_vel_right.set_xlabel('Sample')
ax_vel_right.set_ylabel('Velocity (mm/s)')
ax_vel_right.set_title('Right Wheel Velocity')

# Position plot elements
path_line, = ax_pos.plot([], [], 'b-', linewidth=1, alpha=0.5, label='Path')
robot_body = patches.Rectangle((0, 0), ROBOT_LENGTH, ROBOT_WIDTH,
                                 fill=True, color='blue', alpha=0.5)
robot_direction = ax_pos.arrow(0, 0, 0, 0, head_width=20, head_length=15, fc='red', ec='red')
target_marker, = ax_pos.plot([], [], 'gx', markersize=15, markeredgewidth=3, label='Target')
start_marker, = ax_pos.plot([0], [0], 'ko', markersize=8, label='Start')

ax_pos.add_patch(robot_body)
ax_pos.legend(loc='upper left')

# Status text
status_text = ax_pos.text(0.02, 0.98, '', transform=ax_pos.transAxes,
                          verticalalignment='top', fontfamily='monospace',
                          fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

# Velocity plot elements
vel_left_line, = ax_vel_left.plot([], [], 'b-', linewidth=1, label='Actual')
target_left_line, = ax_vel_left.plot([], [], 'r--', linewidth=1, label='Target')
ax_vel_left.legend(loc='upper right')

vel_right_line, = ax_vel_right.plot([], [], 'b-', linewidth=1, label='Actual')
target_right_line, = ax_vel_right.plot([], [], 'r--', linewidth=1, label='Target')
ax_vel_right.legend(loc='upper right')

# Serial connection
ser = None
serial_port = None

def init_serial(port=None):
    global ser, serial_port

    if port is None:
        port = find_arduino_port()

    if port is None:
        print("No serial ports found!")
        return False

    serial_port = port

    try:
        ser = serial.Serial(serial_port, BAUD_RATE, timeout=0.1)
        print(f"Connected to {serial_port}")
        return True
    except serial.SerialException as e:
        print(f"Error opening serial port {serial_port}: {e}")
        return False

def update(frame):
    global robot_direction

    # Read all available serial data
    if ser and ser.in_waiting:
        try:
            lines = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').split('\n')
            for line in lines:
                line = line.strip()
                if line:
                    print(line)  # Echo to console
                    parse_odometry(line)
                    parse_target(line)
                    parse_velocity(line)
        except Exception as e:
            print(f"Serial error: {e}")

    # Update path
    path_line.set_data(history_x, history_y)

    # Update robot position and orientation
    cos_h = math.cos(current_heading)
    sin_h = math.sin(current_heading)

    cx, cy = current_x, current_y
    hw, hl = ROBOT_WIDTH / 2, ROBOT_LENGTH / 2

    robot_body.set_xy((cx - hl * cos_h + hw * sin_h,
                       cy - hl * sin_h - hw * cos_h))
    robot_body.set_angle(math.degrees(current_heading))

    # Update direction arrow
    robot_direction.remove()
    arrow_len = ROBOT_LENGTH * 0.8
    robot_direction = ax_pos.arrow(cx, cy,
                               arrow_len * cos_h, arrow_len * sin_h,
                               head_width=20, head_length=15, fc='red', ec='red')

    # Update target marker
    if target_x is not None:
        target_marker.set_data([target_x], [target_y])

    # Update status text
    status = f'Position: ({current_x:.1f}, {current_y:.1f}) mm\n'
    status += f'Heading: {math.degrees(current_heading):.1f}\u00b0\n'
    status += f'Vel L: {current_vel_left:.1f} mm/s\n'
    status += f'Vel R: {current_vel_right:.1f} mm/s'
    if target_x is not None:
        dist = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        status += f'\nTarget: ({target_x:.1f}, {target_y:.1f})\nDist: {dist:.1f} mm'
    status_text.set_text(status)

    # Auto-scale position plot
    if len(history_x) > 1:
        all_x = history_x + ([target_x] if target_x else [])
        all_y = history_y + ([target_y] if target_y else [])
        margin = 200
        ax_pos.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax_pos.set_ylim(min(all_y) - margin, max(all_y) + margin)

    # Update velocity plots
    if len(time_history) > 0:
        times = list(time_history)

        # Left wheel
        vel_left_line.set_data(times, list(vel_left_history))
        target_left_line.set_data(times, list(target_vel_history))

        # Right wheel (target is negative of left for turns, same for straights)
        vel_right_line.set_data(times, list(vel_right_history))
        # For right wheel target, we negate for turns (approximate)
        target_right_line.set_data(times, list(target_vel_history))

        # Auto-scale velocity plots
        all_vels = list(vel_left_history) + list(vel_right_history) + list(target_vel_history)
        if all_vels:
            min_vel = min(all_vels) - 20
            max_vel = max(all_vels) + 20
            ax_vel_left.set_xlim(times[0], times[-1] + 1)
            ax_vel_left.set_ylim(min_vel, max_vel)
            ax_vel_right.set_xlim(times[0], times[-1] + 1)
            ax_vel_right.set_ylim(min_vel, max_vel)

    return path_line, robot_body, target_marker, status_text, vel_left_line, vel_right_line, target_left_line, target_right_line

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None

    print("Robot Odometry + Velocity Visualizer")
    print("Usage: python visualizer_velocity.py [serial_port]")
    print()

    if port:
        print(f"Using specified port: {port}")
    else:
        print("Auto-detecting Arduino port...")

    if not init_serial(port):
        print("\nRunning in demo mode (no serial connection)")
        print("Press 'q' to quit")

    plt.tight_layout()
    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    if ser:
        ser.close()

if __name__ == '__main__':
    main()
