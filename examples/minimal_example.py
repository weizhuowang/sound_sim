#!/usr/bin/env python
"""Minimal example showing how to add sound to any MuJoCo simulation"""

import mujoco
import time
from sound_sim import step_with_sound

# Your existing MuJoCo setup
xml = """<mujoco>
    <worldbody>
        <body>
            <joint type="hinge"/>
            <geom size="0.1"/>
        </body>
    </worldbody>
    <actuator><motor joint="0" gear="1"/></actuator>
</mujoco>"""

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

# Your existing simulation loop
for i in range(500):  # 10 seconds at 50Hz
    # Your control logic
    data.ctrl[0] = 0.5
    
    # Normal MuJoCo step
    mujoco.mj_step(model, data)
    
    # Just add this one line for sound!
    step_with_sound(data)
    
    time.sleep(0.02)

print("Done! That's all you need - just one line: step_with_sound(data)")