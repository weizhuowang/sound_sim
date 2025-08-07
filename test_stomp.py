#!/usr/bin/env python
"""Test foot stomp synthesizer with simulated ground contacts"""

import numpy as np
import time
from sound_sim import MujocoSoundSystem, FootStompSynthesizer

print("Testing foot stomp sounds")
print("=" * 50)

# Create sound system with only stomp synthesizer for clarity
sound = MujocoSoundSystem(
    synthesizers=[FootStompSynthesizer()],
    use_mixer=True,
)
sound.start()

# Simulate walking pattern with two feet
# qfrc represents ground reaction forces for each contact point
num_contacts = 2  # Two feet
duration = 10  # seconds
dt = 0.02  # 50Hz

print("\nSimulating walking pattern:")
print("- Alternating foot stomps")
print("- Varying force intensities")
print("- Playing for 10 seconds...\n")

# Walking cycle parameters
step_duration = 0.5  # 500ms per step
steps = int(duration / step_duration)

for step in range(steps):
    # Determine which foot is stomping
    foot = step % 2
    
    # Create force array (zeros for lifted foot, force for stomping foot)
    qfrc = np.zeros(num_contacts)
    
    # Stomp phase (first 100ms of step)
    stomp_frames = int(0.1 / dt)  # 100ms stomp
    stance_frames = int(0.4 / dt)  # 400ms stance
    
    for frame in range(int(step_duration / dt)):
        if frame < stomp_frames:
            # Impact phase - force ramps up quickly
            force_magnitude = 50 + np.random.uniform(0, 30)  # Vary force 50-80
            qfrc[foot] = force_magnitude * (frame / stomp_frames)
            print(f"Foot {foot}: STOMP! (force={qfrc[foot]:.1f})")
        elif frame < stomp_frames + stance_frames:
            # Stance phase - maintain contact
            qfrc[foot] = 50 + np.random.uniform(0, 20)
        else:
            # Lift phase
            qfrc[foot] = 0
            
        # Add small force on other foot during double support
        if frame < stomp_frames:
            qfrc[1-foot] = 20  # Other foot still has some contact
        
        sound.step(qfrc=qfrc)
        time.sleep(dt)

print("\nTesting rapid stomps (running)...")
print("-" * 30)

# Simulate faster running pattern
for _ in range(100):  # 2 seconds of running
    qfrc = np.zeros(num_contacts)
    
    # Faster alternation with shorter contact time
    foot = int((_ / 10) % 2)
    phase = _ % 10
    
    if phase < 3:  # Short impact
        qfrc[foot] = 60 + np.random.uniform(0, 40)
        if phase == 0:
            print(f"Foot {foot}: Quick stomp!")
    
    sound.step(qfrc=qfrc)
    time.sleep(dt)

print("\nTesting heavy jump landing...")
print("-" * 30)

# Both feet land simultaneously
for i in range(50):  # 1 second
    qfrc = np.zeros(num_contacts)
    
    if i == 10:  # Impact at 200ms
        print("BOTH FEET: Heavy landing!")
    
    if 10 <= i < 15:  # Impact phase
        force = 100 * ((i - 10) / 5)  # Ramp up
        qfrc[0] = force
        qfrc[1] = force * 0.9  # Slightly asymmetric
    elif 15 <= i < 40:  # Recovery
        qfrc[0] = 80 * np.exp(-(i-15) * 0.1)
        qfrc[1] = 75 * np.exp(-(i-15) * 0.1)
    
    sound.step(qfrc=qfrc)
    time.sleep(dt)

sound.stop()
print("\nDone!")