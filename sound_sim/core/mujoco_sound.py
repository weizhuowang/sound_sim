import numpy as np
from typing import Optional, List, Dict, Any
from sound_sim.core.sound_engine import SoundEngine
from sound_sim.synthesizers.base import Synthesizer, JointSynthesizer, OscillationSynthesizer, ContactSynthesizer


class MujocoSoundSystem:
    def __init__(
        self,
        sample_rate: int = 44100,
        buffer_size: int = 882,
        synthesizers: Optional[List[Synthesizer]] = None,
        include_contact: bool = True
    ):
        self.engine = SoundEngine(sample_rate, buffer_size)
        self.sample_rate = sample_rate
        self.buffer_size = buffer_size
        self.include_contact = include_contact
        
        if synthesizers is None:
            self.synthesizers = [
                JointSynthesizer(sample_rate, buffer_size),
                OscillationSynthesizer(sample_rate, buffer_size)
            ]
            if include_contact:
                self.synthesizers.append(ContactSynthesizer(sample_rate, buffer_size))
        else:
            self.synthesizers = synthesizers
        
        self.enabled = True
        self.volume = 0.5
        
    def start(self):
        self.engine.start()
    
    def stop(self):
        self.engine.stop()
    
    def extract_state(self, data) -> Dict[str, Any]:
        """Extract state from MuJoCo data object.
        Expects: data.qvel (joint velocities) and data.qfrc_actuator (actuator forces)
        Optional: data.cfrc_ext (contact forces)
        """
        state = {}
        
        if hasattr(data, 'qvel'):
            state['motor_vel'] = data.qvel.copy()
        
        if hasattr(data, 'qfrc_actuator'):
            state['motor_tau'] = data.qfrc_actuator.copy()
        
        if self.include_contact and hasattr(data, 'cfrc_ext'):
            contact_forces = np.linalg.norm(data.cfrc_ext, axis=1)
            state['contact_force'] = contact_forces
        
        return state
    
    def step(self, data=None, motor_vel=None, motor_tau=None, contact_force=None):
        """Step the sound system with state data.
        
        Option 1: Pass MuJoCo data object
            step(data)
        
        Option 2: Pass arrays directly
            step(motor_vel=vel_array, motor_tau=tau_array, contact_force=force_array)
        """
        if not self.enabled:
            return
        
        if data is not None:
            state = self.extract_state(data)
        else:
            state = {}
            if motor_vel is not None:
                state['motor_vel'] = motor_vel
            if motor_tau is not None:
                state['motor_tau'] = motor_tau
            if contact_force is not None and self.include_contact:
                state['contact_force'] = contact_force
        
        mixed_audio = np.zeros(self.buffer_size)
        for synth in self.synthesizers:
            if isinstance(synth, ContactSynthesizer) and 'contact_force' not in state:
                continue
            audio = synth.synthesize(state)
            mixed_audio += audio
        
        mixed_audio = np.clip(mixed_audio * self.volume, -1.0, 1.0)
        
        self.engine.play(mixed_audio)
    
    def set_volume(self, volume: float):
        self.volume = np.clip(volume, 0.0, 1.0)
    
    def enable(self):
        self.enabled = True
    
    def disable(self):
        self.enabled = False
    
    def add_synthesizer(self, synth: Synthesizer):
        self.synthesizers.append(synth)
    
    def remove_synthesizer(self, synth: Synthesizer):
        if synth in self.synthesizers:
            self.synthesizers.remove(synth)
    
    def clear_synthesizers(self):
        self.synthesizers = []


def step_with_sound(data=None, motor_vel=None, motor_tau=None, contact_force=None, 
                    sound_system: Optional[MujocoSoundSystem] = None):
    """Simple interface to add sound to any simulation.
    
    Usage with MuJoCo:
        step_with_sound(data)
    
    Usage with arrays:
        step_with_sound(motor_vel=velocities, motor_tau=torques)
    """
    global _default_sound_system
    
    if sound_system is None:
        if '_default_sound_system' not in globals():
            include_contact = contact_force is not None or data is not None
            _default_sound_system = MujocoSoundSystem(include_contact=include_contact)
            _default_sound_system.start()
        sound_system = _default_sound_system
    
    sound_system.step(data, motor_vel, motor_tau, contact_force)