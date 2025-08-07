#!/usr/bin/env python
"""Test to clearly hear the effect of torque on sound"""

import numpy as np
import time
from sound_sim import MujocoSoundSystem
from sound_sim import TorqueSynthesizer


def main():
    print("Testing torque effect on sound synthesis")
    print("=" * 50)
    
    # Create sound system with torque synthesizer
    sound_system = MujocoSoundSystem(
        synthesizers=[
            # Comment/uncomment to test different synthesizers
            # JointSynthesizer(),  # Motor sounds (vel + tau)
            # OscillationSynthesizer(),  # Oscillation detection
            TorqueSynthesizer(),  # Torque-specific sounds
        ],
        include_contact=False
    )
    sound_system.start()
    
    num_motors = 6
    
    print("\n1. CONSTANT VELOCITY, NO TORQUE (3 seconds)")
    print("   You should hear: Little to no sound")
    for t in range(150):
        motor_vel = np.ones(num_motors) * 0.5  # Constant velocity
        motor_tau = np.zeros(num_motors)  # Zero torque
        
        sound_system.step(motor_vel=motor_vel, motor_tau=motor_tau)
        time.sleep(0.02)
    
    print("\n2. CONSTANT VELOCITY, LOW TORQUE (3 seconds)")
    print("   You should hear: Low rumble")
    for t in range(150):
        motor_vel = np.ones(num_motors) * 0.5
        motor_tau = np.ones(num_motors) * 15  # Low torque
        
        sound_system.step(motor_vel=motor_vel, motor_tau=motor_tau)
        time.sleep(0.02)
    
    print("\n3. CONSTANT VELOCITY, HIGH TORQUE (3 seconds)")
    print("   You should hear: Louder rumble + strain sounds")
    for t in range(150):
        motor_vel = np.ones(num_motors) * 0.5
        motor_tau = np.ones(num_motors) * 40  # High torque
        
        sound_system.step(motor_vel=motor_vel, motor_tau=motor_tau)
        time.sleep(0.02)
    
    print("\n4. CONSTANT VELOCITY, VARYING TORQUE (3 seconds)")
    print("   You should hear: Changing pitch and intensity")
    for t in range(150):
        motor_vel = np.ones(num_motors) * 0.5
        # Sinusoidal torque variation
        motor_tau = np.ones(num_motors) * (20 + 20 * np.sin(t * 0.1))
        
        sound_system.step(motor_vel=motor_vel, motor_tau=motor_tau)
        time.sleep(0.02)
    
    print("\n5. TORQUE IMPACTS (3 seconds)")
    print("   You should hear: Clicks/thuds on sudden torque changes")
    for t in range(150):
        motor_vel = np.ones(num_motors) * 0.5
        # Sudden torque spikes
        if t % 25 == 0:
            motor_tau = np.ones(num_motors) * 60  # Spike
        else:
            motor_tau = np.ones(num_motors) * 10  # Low baseline
        
        sound_system.step(motor_vel=motor_vel, motor_tau=motor_tau)
        time.sleep(0.02)
    
    print("\n" + "=" * 50)
    print("Test complete!")
    
    sound_system.stop()


if __name__ == "__main__":
    main()