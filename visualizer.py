#!/usr/bin/env python3
"""
Robot Odometry Visualizer
Reads serial data from Arduino and plots robot position in real-time
"""

import serial
import serial.tools.list_ports
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
import re
import sys
import math

BAUD_RATE = 115200

def find_arduino_port():
    """Auto-detect Arduino serial port"""
    ports = serial.tools.list_ports.comports()

    # Common Arduino identifiers
    arduino_keywords = ['arduino', 'usbmodem', 'usbserial', 'ch340', 'cp210', 'ftdi', 'acm']

    for port in ports:
        port_lower = (port.device + ' ' + (port.description or '')).lower()
        for keyword in arduino_keywords:
            if keyword in port_lower:
                return port.device

    # If no Arduino found, list available ports
    if ports:
        print("Available ports:")
        for port in ports:
            print(f"  {port.device} - {port.description}")
        return ports[0].device  # Return first available

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

def parse_odometry(line):
    """Parse odometry from 'X: 123.45 Y: 67.89 Heading: 45.00 deg' format"""
    global current_x, current_y, current_heading

    match = re.search(r'X:\s*([-\d.]+)\s*Y:\s*([-\d.]+)\s*Heading:\s*([-\d.]+)', line)
    if match:
        current_x = float(match.group(1))
        current_y = float(match.group(2))
        current_heading = float(match.group(3)) * math.pi / 180  # Convert to radians
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

def parse_log(line):
    """Parse log line for distance and heading error"""
    match = re.search(r'dist:([-\d.]+)\s*hdgErr:([-\d.]+)', line)
    if match:
        return float(match.group(1)), float(match.group(2))
    return None, None

# Set up the plot
fig, ax = plt.subplots(1, 1, figsize=(10, 10))
ax.set_aspect('equal')
ax.grid(True, alpha=0.3)
ax.set_xlabel('X (mm)')
ax.set_ylabel('Y (mm)')
ax.set_title('Robot Odometry Visualizer')

# Plot elements
path_line, = ax.plot([], [], 'b-', linewidth=1, alpha=0.5, label='Path')
robot_body = patches.Rectangle((0, 0), ROBOT_LENGTH, ROBOT_WIDTH,
                                 fill=True, color='blue', alpha=0.5)
robot_direction = ax.arrow(0, 0, 0, 0, head_width=20, head_length=15, fc='red', ec='red')
target_marker, = ax.plot([], [], 'gx', markersize=15, markeredgewidth=3, label='Target')
start_marker, = ax.plot([0], [0], 'ko', markersize=8, label='Start')

ax.add_patch(robot_body)
ax.legend(loc='upper left')

# Status text
status_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                      verticalalignment='top', fontfamily='monospace',
                      fontsize=9, bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

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
        except Exception as e:
            print(f"Serial error: {e}")

    # Update path
    path_line.set_data(history_x, history_y)

    # Update robot position and orientation
    # Robot rectangle centered on position
    cos_h = math.cos(current_heading)
    sin_h = math.sin(current_heading)

    # Calculate corners of robot rectangle
    cx, cy = current_x, current_y
    hw, hl = ROBOT_WIDTH / 2, ROBOT_LENGTH / 2

    # Remove old robot body and create new one with rotation
    robot_body.set_xy((cx - hl * cos_h + hw * sin_h,
                       cy - hl * sin_h - hw * cos_h))
    robot_body.set_angle(math.degrees(current_heading))

    # Update direction arrow
    robot_direction.remove()
    arrow_len = ROBOT_LENGTH * 0.8
    robot_direction = ax.arrow(cx, cy,
                               arrow_len * cos_h, arrow_len * sin_h,
                               head_width=20, head_length=15, fc='red', ec='red')

    # Update target marker
    if target_x is not None:
        target_marker.set_data([target_x], [target_y])

    # Update status text
    status = f'Position: ({current_x:.1f}, {current_y:.1f}) mm\n'
    status += f'Heading: {math.degrees(current_heading):.1f}°'
    if target_x is not None:
        dist = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        status += f'\nTarget: ({target_x:.1f}, {target_y:.1f})\nDist to target: {dist:.1f} mm'
    status_text.set_text(status)

    # Auto-scale with some padding
    if len(history_x) > 1:
        all_x = history_x + ([target_x] if target_x else [])
        all_y = history_y + ([target_y] if target_y else [])
        margin = 200
        ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
        ax.set_ylim(min(all_y) - margin, max(all_y) + margin)

    return path_line, robot_body, target_marker, status_text

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else None

    print("Robot Odometry Visualizer")
    print("Usage: python visualizer.py [serial_port]")
    print()

    if port:
        print(f"Using specified port: {port}")
    else:
        print("Auto-detecting Arduino port...")

    if not init_serial(port):
        print("\nRunning in demo mode (no serial connection)")
        print("Press 'q' to quit")

    ani = FuncAnimation(fig, update, interval=50, blit=False, cache_frame_data=False)
    plt.show()

    if ser:
        ser.close()

if __name__ == '__main__':
    main()
