#!/usr/bin/env python
"""Test sound synthesis without viewer - works with regular python"""

import mujoco
import numpy as np
import time
from sound_sim import MujocoSoundSystem

def main():
    # Create a simple humanoid model
    xml_string = """
    <mujoco model="test_humanoid">
        <option timestep="0.02"/>
        
        <worldbody>
            <body name="torso" pos="0 0 1">
                <joint name="slide_x" type="slide" axis="1 0 0"/>
                <joint name="slide_y" type="slide" axis="0 1 0"/>
                <joint name="hinge_z" type="hinge" axis="0 0 1"/>
                <geom type="box" size="0.1 0.1 0.2"/>
                
                <body name="leg" pos="0 0 -0.3">
                    <joint name="hip" type="hinge" axis="0 1 0"/>
                    <geom type="capsule" fromto="0 0 0 0 0 -0.3" size="0.05"/>
                    
                    <body name="foot" pos="0 0 -0.3">
                        <joint name="knee" type="hinge" axis="0 1 0"/>
                        <geom type="sphere" size="0.05"/>
                    </body>
                </body>
            </body>
        </worldbody>
        
        <actuator>
            <motor joint="hip" gear="10"/>
            <motor joint="knee" gear="10"/>
        </actuator>
    </mujoco>
    """
    
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)
    
    # Initialize sound system
    sound_system = MujocoSoundSystem()
    sound_system.start()
    
    print("Sound test started! Testing different movement patterns...")
    print("=" * 50)
    
    # Test 1: Smooth sinusoidal motion
    print("\n1. SMOOTH MOTION (5 seconds)")
    print("   You should hear: Steady motor sounds")
    phase = 0
    for i in range(250):  # 5 seconds at 50Hz
        phase += 0.1
        data.ctrl[0] = 0.5 * np.sin(phase)
        data.ctrl[1] = 0.5 * np.cos(phase)
        
        mujoco.mj_step(model, data)
        sound_system.step(data)
        time.sleep(0.02)  # 50Hz
    
    # Test 2: Oscillating/jerky motion
    print("\n2. OSCILLATING MOTION (5 seconds)")
    print("   You should hear: Clicks and buzzing from oscillations")
    for i in range(250):
        # Add random noise to create oscillations
        data.ctrl[0] = np.random.randn() * 0.5
        data.ctrl[1] = np.random.randn() * 0.5
        
        mujoco.mj_step(model, data)
        sound_system.step(data)
        time.sleep(0.02)
    
    # Test 3: Sudden impacts
    print("\n3. IMPACT TEST (5 seconds)")
    print("   You should hear: Impact sounds when velocity changes rapidly")
    for i in range(250):
        if i % 50 == 0:  # Every second
            data.ctrl[0] = 1.0 if i % 100 == 0 else -1.0
            data.ctrl[1] = -1.0 if i % 100 == 0 else 1.0
        else:
            data.ctrl[0] = 0
            data.ctrl[1] = 0
        
        mujoco.mj_step(model, data)
        sound_system.step(data)
        time.sleep(0.02)
    
    # Test 4: Different frequencies
    print("\n4. FREQUENCY SWEEP (5 seconds)")
    print("   You should hear: Pitch changes with velocity")
    for i in range(250):
        speed = i / 250.0  # Gradually increase speed
        data.ctrl[0] = speed * np.sin(i * 0.1 * (1 + speed * 2))
        data.ctrl[1] = speed * np.cos(i * 0.1 * (1 + speed * 2))
        
        mujoco.mj_step(model, data)
        sound_system.step(data)
        time.sleep(0.02)
    
    print("\n" + "=" * 50)
    print("Test complete!")
    
    sound_system.stop()

if __name__ == "__main__":
    main()