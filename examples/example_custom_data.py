#!/usr/bin/env python
"""Example using custom data arrays instead of MuJoCo objects"""

import numpy as np
import time
from sound_sim import MujocoSoundSystem, step_with_sound


def main():
    print("Sound synthesis with custom data arrays")
    print("=" * 50)
    
    # Initialize sound system without contact sounds
    sound_system = MujocoSoundSystem(include_contact=False)
    sound_system.start()
    
    # Simulate a robot with 6 motors
    num_motors = 6
    timesteps = 500  # 10 seconds at 50Hz
    
    print("\n1. SMOOTH WALKING PATTERN (5 seconds)")
    print("   Simulating smooth sinusoidal motion...")
    
    for t in range(250):
        phase = t * 0.1
        
        # Generate smooth motor velocities (rad/s)
        motor_vel = np.array([
            0.5 * np.sin(phase),
            0.5 * np.cos(phase),
            0.3 * np.sin(phase * 2),
            0.5 * np.sin(phase + np.pi),
            0.5 * np.cos(phase + np.pi),
            0.3 * np.sin(phase * 2 + np.pi)
        ])
        
        # Generate corresponding torques (Nm)
        motor_tau = motor_vel * 10 + np.random.randn(num_motors) * 0.1
        
        # Step the sound system
        sound_system.step(motor_vel=motor_vel, motor_tau=motor_tau)
        
        time.sleep(0.02)  # 50Hz
    
    print("\n2. OSCILLATING/JERKY MOTION (5 seconds)")
    print("   Adding noise to create policy oscillations...")
    
    for t in range(250):
        # Generate oscillating velocities
        base_vel = np.sin(t * 0.1) * 0.3
        noise = np.random.randn(num_motors) * 0.5
        motor_vel = base_vel + noise
        
        # Higher, more variable torques
        motor_tau = motor_vel * 20 + np.random.randn(num_motors) * 5
        
        sound_system.step(motor_vel=motor_vel, motor_tau=motor_tau)
        time.sleep(0.02)
    
    print("\n" + "=" * 50)
    print("Test complete!")
    
    sound_system.stop()


def example_with_contact():
    """Example including contact forces"""
    print("\nExample with contact forces:")
    
    # Initialize with contact sounds enabled
    sound_system = MujocoSoundSystem(include_contact=True)
    sound_system.start()
    
    for t in range(100):
        motor_vel = np.random.randn(6) * 0.5
        motor_tau = motor_vel * 10
        
        # Simulate ground contact (e.g., foot hitting ground)
        if t % 25 == 0:
            contact_force = np.array([100.0])  # Impact force
        else:
            contact_force = np.array([10.0])   # Standing force
        
        sound_system.step(
            motor_vel=motor_vel,
            motor_tau=motor_tau,
            contact_force=contact_force
        )
        
        time.sleep(0.02)
    
    sound_system.stop()


def example_simple_api():
    """Simplest possible usage"""
    print("\nSimplest API usage:")
    
    # Just call step_with_sound directly - it handles everything
    for t in range(100):
        motor_vel = np.sin(t * 0.1) * np.ones(4)
        motor_tau = motor_vel * 5
        
        # One line - that's it!
        step_with_sound(motor_vel=motor_vel, motor_tau=motor_tau)
        
        time.sleep(0.02)


if __name__ == "__main__":
    main()
    # Uncomment to test other examples:
    # example_with_contact()
    # example_simple_api()