"""Base synthesizer class for sound synthesis."""

import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, Any


class Synthesizer(ABC):
    """Base class for all synthesizers."""
    
    def __init__(self, sample_rate: int = 44100, buffer_size: int = 882):
        self.sample_rate = sample_rate
        self.buffer_size = buffer_size
        self.time = np.linspace(0, buffer_size/sample_rate, buffer_size)
        
    @abstractmethod
    def synthesize(self, state: Dict[str, Any]) -> np.ndarray:
        """Generate audio from state data.
        
        Args:
            state: Dictionary containing motor_vel, motor_tau, etc.
            
        Returns:
            Audio samples (numpy array)
        """
        pass