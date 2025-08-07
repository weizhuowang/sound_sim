import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Any, Optional
from scipy import signal


class Synthesizer(ABC):
    def __init__(self, sample_rate: int = 44100, buffer_size: int = 882):
        self.sample_rate = sample_rate
        self.buffer_size = buffer_size
        self.time = np.linspace(0, buffer_size/sample_rate, buffer_size)
        
    @abstractmethod
    def synthesize(self, state: Dict[str, Any]) -> np.ndarray:
        pass


class JointSynthesizer(Synthesizer):
    def __init__(self, sample_rate: int = 44100, buffer_size: int = 882):
        super().__init__(sample_rate, buffer_size)
        self.base_freq = 100.0
        self.freq_scale = 50.0
        self.torque_freq_scale = 20.0  # Torque affects frequency
        self.torque_amp_scale = 0.5   # Torque affects amplitude
        
    def synthesize(self, state: Dict[str, Any]) -> np.ndarray:
        motor_vel = state.get('motor_vel', np.array([]))
        motor_tau = state.get('motor_tau', np.array([]))
        
        if len(motor_vel) == 0:
            return np.zeros(self.buffer_size)
        
        avg_vel = np.mean(np.abs(motor_vel))
        avg_torque = np.mean(np.abs(motor_tau)) if len(motor_tau) > 0 else 0
        
        # Frequency based on BOTH velocity and torque
        frequency = self.base_freq + avg_vel * self.freq_scale + avg_torque * self.torque_freq_scale
        frequency = np.clip(frequency, 50, 2000)
        
        # Main motor sound
        wave = np.sin(2 * np.pi * frequency * self.time)
        
        # Add harmonics when torque is high (motor strain)
        if avg_torque > 5:
            torque_norm = np.tanh(avg_torque / 20.0)
            # Add second and third harmonics for richer sound
            wave += 0.3 * torque_norm * np.sin(4 * np.pi * frequency * self.time)
            wave += 0.15 * torque_norm * np.sin(6 * np.pi * frequency * self.time)
            # Add some roughness with subharmonic
            wave += 0.1 * torque_norm * np.sin(np.pi * frequency * self.time)
        
        # Amplitude based on BOTH velocity and torque
        vel_amplitude = np.tanh(avg_vel * 2) * 0.2
        torque_amplitude = np.tanh(avg_torque / 30.0) * self.torque_amp_scale * 0.2
        amplitude = vel_amplitude + torque_amplitude
        
        return wave * amplitude


class OscillationSynthesizer(Synthesizer):
    def __init__(self, sample_rate: int = 44100, buffer_size: int = 882):
        super().__init__(sample_rate, buffer_size)
        self.prev_qvel = None
        self.oscillation_history = []
        self.history_size = 10
        self.click_sound = self._generate_click()
        self.buzz_sound = self._generate_buzz()
        
    def _generate_click(self) -> np.ndarray:
        t = np.linspace(0, 0.001, int(0.001 * self.sample_rate))
        click = np.sin(2 * np.pi * 1000 * t) * np.exp(-t * 1000)
        if len(click) < self.buffer_size:
            return np.pad(click, (0, self.buffer_size - len(click)))
        else:
            return click[:self.buffer_size]
    
    def _generate_buzz(self) -> np.ndarray:
        freq = np.random.uniform(150, 250)
        buzz = np.sin(2 * np.pi * freq * self.time)
        buzz += 0.3 * np.sin(2 * np.pi * freq * 2 * self.time)
        envelope = signal.windows.tukey(self.buffer_size, alpha=0.5)
        return buzz * envelope * 0.2
    
    def synthesize(self, state: Dict[str, Any]) -> np.ndarray:
        motor_vel = state.get('motor_vel', np.array([]))
        
        if len(motor_vel) == 0:
            return np.zeros(self.buffer_size)
        
        if self.prev_qvel is None:
            self.prev_qvel = motor_vel.copy()
            return np.zeros(self.buffer_size)
        
        accel = motor_vel - self.prev_qvel
        jerk_metric = np.std(accel)
        
        self.oscillation_history.append(jerk_metric)
        if len(self.oscillation_history) > self.history_size:
            self.oscillation_history.pop(0)
        
        avg_jerk = np.mean(self.oscillation_history)
        
        direction_changes = np.sum(np.diff(np.sign(motor_vel)) != 0)
        
        output = np.zeros(self.buffer_size)
        
        if direction_changes > len(motor_vel) * 0.3:
            output += self.click_sound * min(direction_changes / len(motor_vel), 1.0)
        
        if avg_jerk > 0.5:
            jerk_intensity = np.tanh((avg_jerk - 0.5) * 2)
            output += self.buzz_sound * jerk_intensity
        
        self.prev_qvel = motor_vel.copy()
        return output


class ContactSynthesizer(Synthesizer):
    def __init__(self, sample_rate: int = 44100, buffer_size: int = 882):
        super().__init__(sample_rate, buffer_size)
        self.prev_contact = None
        self.impact_sound = self._generate_impact()
        
    def _generate_impact(self) -> np.ndarray:
        t = np.linspace(0, 0.05, int(0.05 * self.sample_rate))
        impact = np.random.randn(len(t)) * 0.5
        impact = signal.lfilter([1], [1, -0.95], impact)
        impact *= np.exp(-t * 50)
        if len(impact) < self.buffer_size:
            return np.pad(impact, (0, self.buffer_size - len(impact)))
        else:
            return impact[:self.buffer_size]
    
    def synthesize(self, state: Dict[str, Any]) -> np.ndarray:
        contact_force = state.get('contact_force', np.array([]))
        
        if len(contact_force) == 0:
            return np.zeros(self.buffer_size)
        
        total_force = np.sum(np.abs(contact_force))
        
        if self.prev_contact is None:
            self.prev_contact = total_force
            return np.zeros(self.buffer_size)
        
        force_delta = total_force - self.prev_contact
        
        output = np.zeros(self.buffer_size)
        if force_delta > 10.0:
            intensity = np.tanh(force_delta / 50.0)
            output = self.impact_sound * intensity * 0.5
        
        self.prev_contact = total_force
        return output