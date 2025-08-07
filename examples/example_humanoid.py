import mujoco
import mujoco.viewer
import numpy as np
from sound_sim import MujocoSoundSystem, step_with_sound


def main():
    xml_string = """
    <mujoco model="humanoid">
        <option timestep="0.02" gravity="0 0 -9.81"/>
        
        <default>
            <joint armature="0.01" damping="0.1"/>
            <geom contype="1" conaffinity="1" friction="0.7 0.1 0.1"/>
        </default>
        
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom name="floor" type="plane" size="10 10 0.1" rgba=".9 .9 .9 1"/>
            
            <body name="torso" pos="0 0 1.5">
                <joint name="root" type="free"/>
                <geom name="torso_geom" type="box" size="0.15 0.1 0.3" rgba="0.5 0.5 1 1" mass="10"/>
                
                <body name="pelvis" pos="0 0 -0.3">
                    <geom name="pelvis_geom" type="box" size="0.15 0.1 0.1" rgba="0.5 0.5 1 1" mass="5"/>
                    
                    <body name="right_thigh" pos="0.1 0 -0.1">
                        <joint name="right_hip" type="hinge" axis="0 1 0" range="-45 45"/>
                        <geom name="right_thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.06" rgba="0.7 0.3 0.3 1" mass="3"/>
                        
                        <body name="right_shin" pos="0 0 -0.4">
                            <joint name="right_knee" type="hinge" axis="0 1 0" range="-90 0"/>
                            <geom name="right_shin_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.05" rgba="0.7 0.3 0.3 1" mass="2"/>
                            
                            <body name="right_foot" pos="0 0 -0.4">
                                <joint name="right_ankle" type="hinge" axis="0 1 0" range="-30 30"/>
                                <geom name="right_foot_geom" type="box" size="0.08 0.04 0.02" pos="0.04 0 -0.02" rgba="0.3 0.3 0.3 1" mass="0.5"/>
                            </body>
                        </body>
                    </body>
                    
                    <body name="left_thigh" pos="-0.1 0 -0.1">
                        <joint name="left_hip" type="hinge" axis="0 1 0" range="-45 45"/>
                        <geom name="left_thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.06" rgba="0.3 0.7 0.3 1" mass="3"/>
                        
                        <body name="left_shin" pos="0 0 -0.4">
                            <joint name="left_knee" type="hinge" axis="0 1 0" range="-90 0"/>
                            <geom name="left_shin_geom" type="capsule" fromto="0 0 0 0 0 -0.4" size="0.05" rgba="0.3 0.7 0.3 1" mass="2"/>
                            
                            <body name="left_foot" pos="0 0 -0.4">
                                <joint name="left_ankle" type="hinge" axis="0 1 0" range="-30 30"/>
                                <geom name="left_foot_geom" type="box" size="0.08 0.04 0.02" pos="0.04 0 -0.02" rgba="0.3 0.3 0.3 1" mass="0.5"/>
                            </body>
                        </body>
                    </body>
                </body>
            </body>
        </worldbody>
        
        <actuator>
            <motor name="right_hip_motor" joint="right_hip" gear="50" ctrlrange="-1 1"/>
            <motor name="right_knee_motor" joint="right_knee" gear="30" ctrlrange="-1 1"/>
            <motor name="right_ankle_motor" joint="right_ankle" gear="20" ctrlrange="-1 1"/>
            <motor name="left_hip_motor" joint="left_hip" gear="50" ctrlrange="-1 1"/>
            <motor name="left_knee_motor" joint="left_knee" gear="30" ctrlrange="-1 1"/>
            <motor name="left_ankle_motor" joint="left_ankle" gear="20" ctrlrange="-1 1"/>
        </actuator>
    </mujoco>
    """
    
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)
    
    sound_system = MujocoSoundSystem(sample_rate=44100, buffer_size=882)
    sound_system.start()
    print("Sound system started! You should hear:")
    print("- Motor sounds based on joint velocities")
    print("- Clicks and buzzes when the policy oscillates")
    print("- Impact sounds on ground contact")
    print("\nControls:")
    print("- Press 'v' to adjust volume")
    print("- Press 's' to toggle sound on/off")
    
    class Controller:
        def __init__(self):
            self.time = 0
            self.phase = 0
            
        def compute_control(self, data):
            self.time += model.opt.timestep
            self.phase += 0.1
            
            data.ctrl[0] = 0.3 * np.sin(self.phase)
            data.ctrl[1] = 0.5 * np.sin(self.phase * 1.5)
            data.ctrl[2] = 0.2 * np.sin(self.phase * 2)
            
            data.ctrl[3] = 0.3 * np.sin(self.phase + np.pi)
            data.ctrl[4] = 0.5 * np.sin(self.phase * 1.5 + np.pi)
            data.ctrl[5] = 0.2 * np.sin(self.phase * 2 + np.pi)
            
            if self.time > 5 and self.time < 10:
                noise = np.random.randn(6) * 0.3
                data.ctrl[:] += noise
    
    controller = Controller()
    
    def key_callback(keycode):
        if keycode == ord('v'):
            current = sound_system.volume
            new_volume = (current + 0.25) % 1.25
            if new_volume > 1.0:
                new_volume = 0.25
            sound_system.set_volume(new_volume)
            print(f"Volume: {new_volume:.2f}")
        elif keycode == ord('s'):
            if sound_system.enabled:
                sound_system.disable()
                print("Sound disabled")
            else:
                sound_system.enable()
                print("Sound enabled")
    
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            mujoco.mj_step(model, data)
            
            controller.compute_control(data)
            
            sound_system.step(data)
            
            viewer.sync()
    
    sound_system.stop()
    print("Sound system stopped")


if __name__ == "__main__":
    main()