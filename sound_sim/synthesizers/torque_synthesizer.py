import numpy as np
from sound_sim.synthesizers.base import Synthesizer
from typing import Dict, Any
from scipy import signal


class TorqueSynthesizer(Synthesizer):
    """Synthesizer specifically for motor torque/strain sounds."""
    
    def __init__(self, sample_rate: int = 44100, buffer_size: int = 882):
        super().__init__(sample_rate, buffer_size)
        self.prev_tau = None
        self.strain_threshold = 20.0  # Torque threshold for strain sounds
        
    def synthesize(self, state: Dict[str, Any]) -> np.ndarray:
        motor_tau = state.get('motor_tau', np.array([]))
        
        if len(motor_tau) == 0:
            return np.zeros(self.buffer_size)
        
        avg_torque = np.mean(np.abs(motor_tau))
        max_torque = np.max(np.abs(motor_tau))
        
        output = np.zeros(self.buffer_size)
        
        # Low-frequency rumble for high torque
        if avg_torque > 10:
            rumble_freq = 30 + avg_torque * 0.5  # 30-80 Hz
            rumble = np.sin(2 * np.pi * rumble_freq * self.time)
            rumble *= np.tanh(avg_torque / 30.0) * 0.15
            output += rumble
        
        # Mid-frequency strain sound
        if avg_torque > self.strain_threshold:
            strain_intensity = np.tanh((avg_torque - self.strain_threshold) / 20.0)
            
            # Metallic strain sound (multiple harmonics)
            strain_freq = 200 + avg_torque * 2
            for harmonic in [1, 1.5, 2, 2.5, 3]:
                output += (0.2 / harmonic) * strain_intensity * np.sin(
                    2 * np.pi * strain_freq * harmonic * self.time
                )
            
            # Add some noise for grittiness
            noise = np.random.randn(self.buffer_size) * 0.05 * strain_intensity
            output += signal.lfilter([1], [1, -0.9], noise)
        
        # Sudden torque changes (impacts/jerks)
        if self.prev_tau is not None:
            torque_change = np.mean(np.abs(motor_tau - self.prev_tau))
            if torque_change > 15:
                # Create a click/thud sound
                click_intensity = np.tanh(torque_change / 30.0)
                t_click = np.linspace(0, 0.01, int(0.01 * self.sample_rate))
                click = np.sin(2 * np.pi * 150 * t_click) * np.exp(-t_click * 200)
                if len(click) < self.buffer_size:
                    click_padded = np.zeros(self.buffer_size)
                    click_padded[:len(click)] = click
                    output += click_padded * click_intensity * 0.3
        
        self.prev_tau = motor_tau.copy()
        
        return output