#!/usr/bin/env python
"""Play sound from real robot data"""

import time
import numpy as np
from sound_sim import load_and_prepare, MujocoSoundSystem
from tqdm import tqdm
import time
import matplotlib.pyplot as plt


def main():
    # Load your real data
    print("Loading real robot data...")
    motor_data = load_and_prepare(
        "data/g1_stu_future_real_recordings_20250804_195137.json"
    )

    motor_vel = motor_data["motor_vel"]  # (35705, 29)
    motor_tau = motor_data["motor_tau"]  # (35705, 29)

    print(f"Loaded {motor_vel.shape[0]} timesteps with {motor_vel.shape[1]} motors")
    print(f"Duration: {motor_vel.shape[0] * 0.02:.1f} seconds at 50Hz")

    # Initialize sound system with enhanced torque sounds
    from sound_sim import JointSynthesizer, OscillationSynthesizer, TorqueSynthesizer
    
    sound_system = MujocoSoundSystem(
        synthesizers=[
            JointSynthesizer(),      # Motor sounds (vel + tau effects)
            TorqueSynthesizer(),      # Dedicated torque/strain sounds
            OscillationSynthesizer(), # Oscillation detection
        ],
        include_contact=False
    )
    sound_system.start()

    print("\nPlaying sound from real data...")
    print("Press Ctrl+C to stop")

    try:
        # Play through the data
        for t in tqdm(range(3000, min(6000, len(motor_vel)))):  # First 20 seconds
            t1 = time.time()
            # Get current timestep data
            vel = motor_vel[t]
            tau = motor_tau[t]

            # Generate sound
            sound_system.step(motor_vel=vel, motor_tau=tau)  # Use actual tau now!

            # Print progress every second
            if t % 50 == 0:
                print(
                    f"Time: {t * 0.02:.1f}s - Avg vel: {np.mean(np.abs(vel)):.2f}, Avg tau: {np.mean(np.abs(tau)):.2f}"
                )

            t2 = time.time()
            # Precise sleep to match original data rate (50Hz)
            time.sleep(0.02 - (t2 - t1) - 0.0032)

    except KeyboardInterrupt:
        print("\nStopped by user")

    sound_system.stop()
    print("Done!")


if __name__ == "__main__":
    main()
