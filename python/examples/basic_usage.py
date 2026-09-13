#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RobStride Basic Usage Example
Demonstrates basic motor control operations
"""

import sys
import os
import time
import math

# Add the python directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'python'))

try:
    from src.position_control import PositionControllerMIT
    from src.speed_control import SpeedController
except ImportError:
    print("❌ Cannot import motor control modules")
    sys.exit(1)

def test_position_control(motor_id=11, can_interface='can0'):
    """Test position control with basic movements"""
    print(f"\n🎯 Testing Position Control (Motor {motor_id})")
    print("=" * 50)

    controller = PositionControllerMIT(motor_id, channel=can_interface)

    try:
        # Connect to motor
        if not controller.connect():
            print("❌ Failed to connect to motor")
            return False

        print("✅ Connected successfully!")

        # Test movements
        movements = [
            (0.0, "Home position"),
            (90.0, "90 degrees"),
            (-90.0, "-90 degrees"),
            (180.0, "180 degrees"),
            (0.0, "Back to home")
        ]

        for angle, description in movements:
            print(f"📍 Moving to {description}: {angle}°")
            controller.set_angle(angle)
            time.sleep(2.0)  # Wait for movement to complete

        print("✅ Position control test completed!")
        return True

    except Exception as e:
        print(f"❌ Position control test failed: {e}")
        return False
    finally:
        controller.stop_and_exit()

def test_speed_control(motor_id=11, can_interface='can0'):
    """Test speed control with basic movements"""
    print(f"\n⚡ Testing Speed Control (Motor {motor_id})")
    print("=" * 50)

    controller = SpeedController(motor_id, channel=can_interface)

    try:
        # Connect to motor
        if not controller.connect():
            print("❌ Failed to connect to motor")
            return False

        print("✅ Connected successfully!")

        # Test speeds
        speeds = [
            (2.0, "Forward 2.0 rad/s"),
            (5.0, "Forward 5.0 rad/s"),
            (0.0, "Stop"),
            (-2.0, "Reverse 2.0 rad/s"),
            (-5.0, "Reverse 5.0 rad/s"),
            (0.0, "Stop")
        ]

        for speed, description in speeds:
            print(f"🚀 {description}: {speed} rad/s")
            controller.set_velocity(speed)
            time.sleep(3.0)  # Run for 3 seconds

        print("✅ Speed control test completed!")
        return True

    except Exception as e:
        print(f"❌ Speed control test failed: {e}")
        return False
    finally:
        controller.stop_and_exit()

def interactive_demo(motor_id=11, can_interface='can0'):
    """Interactive demo for user experimentation"""
    print(f"\n🎮 Interactive Demo (Motor {motor_id})")
    print("=" * 50)
    print("Choose control mode:")
    print("1. Position Control")
    print("2. Speed Control")
    print("3. Both (alternating)")

    try:
        choice = input("Enter choice (1-3): ").strip()

        if choice == '1':
            return test_position_control(motor_id, can_interface)
        elif choice == '2':
            return test_speed_control(motor_id, can_interface)
        elif choice == '3':
            result1 = test_position_control(motor_id, can_interface)
            time.sleep(1.0)
            result2 = test_speed_control(motor_id, can_interface)
            return result1 and result2
        else:
            print("❌ Invalid choice")
            return False

    except KeyboardInterrupt:
        print("\n👋 Demo interrupted by user")
        return False

def main():
    """Main function"""
    print("🎯 RobStride Basic Usage Example")
    print("=" * 50)

    # Get motor ID from command line or use default
    motor_id = 11
    if len(sys.argv) > 1:
        try:
            motor_id = int(sys.argv[1])
        except ValueError:
            print("❌ Invalid motor ID. Using default (11)")

    # Get CAN interface from command line or environment
    can_interface = 'can0'
    if len(sys.argv) > 2:
        can_interface = sys.argv[2]
    else:
        can_interface = os.environ.get('CAN_INTERFACE', 'can0')

    print(f"Using Motor ID: {motor_id}")
    print(f"Using CAN interface: {can_interface}")

    # Check and setup CAN interface
    # Handle SLCAN USB devices (like CH340)
    if can_interface.startswith('/dev/tty') or can_interface.startswith('slcan'):
        print(f"🔌 Detected SLCAN/serial device: {can_interface}")
        print(f"   Make sure slcand is running:")
        print(f"   sudo slcand -o -s8 -S 1000000 {can_interface} can0")
        print(f"   sudo ip link set up can0")
        print(f"   Then use 'can0' as CAN_INTERFACE")
        return 1
    
    if not os.path.exists(f'/sys/class/net/{can_interface}'):
        print(f"❌ Error: {can_interface} interface not found")
        print(f"For native CAN (can0):")
        print(f"  sudo ip link set {can_interface} type can bitrate 1000000")
        print(f"  sudo ip link set up {can_interface}")
        print(f"For SLCAN USB (CH340, etc.):")
        print(f"  sudo slcand -o -s8 -S 1000000 /dev/ttyUSB0 can0")
        print(f"  sudo ip link set up can0")
        print(f"\nOr set CAN_INTERFACE environment variable:")
        print(f"  export CAN_INTERFACE=can0")
        return 1
    
    # Check if interface is UP
    import subprocess
    try:
        result = subprocess.run(['ip', 'link', 'show', can_interface], 
                              capture_output=True, text=True)
        if 'UP' not in result.stdout:
            print(f"⚠️ Warning: {can_interface} is not UP")
            print(f"Attempting to bring up {can_interface}...")
            subprocess.run(['sudo', 'ip', 'link', 'set', 'up', can_interface], 
                         capture_output=True)
            print(f"  Run manually if failed: sudo ip link set up {can_interface}")
    except Exception:
        pass
    
    print(f"📡 Using CAN interface: {can_interface}")

    try:
        # Run interactive demo
        success = interactive_demo(motor_id, can_interface)

        if success:
            print("\n🎉 Demo completed successfully!")
            return 0
        else:
            print("\n❌ Demo failed!")
            return 1

    except Exception as e:
        print(f"\n💥 Unexpected error: {e}")
        return 1

if __name__ == "__main__":
    sys.exit(main())