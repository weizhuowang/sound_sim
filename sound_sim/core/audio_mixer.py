import numpy as np
from collections import deque
from typing import Optional


class AudioMixer:
    """Handles mixing and overlapping of audio buffers across time steps.
    
    This solves the problem of sounds being cut off at buffer boundaries
    by maintaining a rolling buffer that accumulates overlapping sounds.
    """
    
    def __init__(self, sample_rate: int = 44100, buffer_size: int = 882, max_overlap_buffers: int = 10):
        """
        Args:
            sample_rate: Audio sample rate
            buffer_size: Size of each audio buffer (20ms at 44.1kHz = 882 samples)
            max_overlap_buffers: Maximum number of overlapping sounds (limits tail length)
        """
        self.sample_rate = sample_rate
        self.buffer_size = buffer_size
        self.max_overlap_buffers = max_overlap_buffers
        
        # Rolling buffer for accumulating overlapping sounds
        self.mix_buffer = np.zeros(buffer_size * max_overlap_buffers)
        
        # Queue of active sound tails (sounds that extend beyond current buffer)
        self.active_sounds = deque(maxlen=max_overlap_buffers)
        
    def add_sound(self, sound: np.ndarray, extend_duration: bool = False) -> np.ndarray:
        """Add a new sound to the mixer and get the current output buffer.
        
        Args:
            sound: New sound to add (can be any length)
            extend_duration: If True, let sound play out fully even if longer than buffer_size
        
        Returns:
            Mixed audio buffer of size buffer_size for current timestep
        """
        # Shift mix buffer left by buffer_size (advance time)
        self.mix_buffer[:-self.buffer_size] = self.mix_buffer[self.buffer_size:]
        self.mix_buffer[-self.buffer_size:] = 0
        
        # Process active sounds from previous timesteps
        for active_sound in list(self.active_sounds):
            remaining = active_sound['samples']
            position = active_sound['position']
            
            if position < len(remaining):
                # How many samples to add in this buffer
                samples_to_add = min(self.buffer_size, len(remaining) - position)
                
                # Add to mix buffer
                self.mix_buffer[:samples_to_add] += remaining[position:position + samples_to_add]
                
                # Update position
                active_sound['position'] += samples_to_add
                
                # Remove if complete
                if active_sound['position'] >= len(remaining):
                    self.active_sounds.remove(active_sound)
        
        # Add new sound
        if len(sound) > 0:
            if extend_duration and len(sound) > self.buffer_size:
                # Add first part to current buffer
                self.mix_buffer[:self.buffer_size] += sound[:self.buffer_size]
                
                # Save remainder for next timesteps
                self.active_sounds.append({
                    'samples': sound,
                    'position': self.buffer_size
                })
            else:
                # Add entire sound (truncate if needed)
                samples_to_add = min(len(sound), self.buffer_size)
                self.mix_buffer[:samples_to_add] += sound[:samples_to_add]
        
        # Return current buffer (with clipping)
        return np.clip(self.mix_buffer[:self.buffer_size], -1.0, 1.0)
    
    def get_output(self) -> np.ndarray:
        """Get current output buffer without adding new sound."""
        return np.clip(self.mix_buffer[:self.buffer_size], -1.0, 1.0)
    
    def reset(self):
        """Clear all buffers and active sounds."""
        self.mix_buffer.fill(0)
        self.active_sounds.clear()


class OverlapBuffer:
    """Simple overlap-add buffer for smooth transitions between audio frames."""
    
    def __init__(self, buffer_size: int = 882, overlap: int = 64):
        """
        Args:
            buffer_size: Size of each audio buffer
            overlap: Number of samples to crossfade between buffers
        """
        self.buffer_size = buffer_size
        self.overlap = overlap
        self.prev_tail = np.zeros(overlap)
        
    def process(self, audio: np.ndarray) -> np.ndarray:
        """Apply overlap-add to smooth transitions between buffers.
        
        Args:
            audio: Input audio buffer
            
        Returns:
            Processed audio with smooth transitions
        """
        if len(audio) < self.overlap:
            return audio
            
        output = audio.copy()
        
        # Crossfade with previous buffer's tail
        fade_in = np.linspace(0, 1, self.overlap)
        fade_out = np.linspace(1, 0, self.overlap)
        
        output[:self.overlap] = (
            self.prev_tail * fade_out + 
            audio[:self.overlap] * fade_in
        )
        
        # Save tail for next buffer
        self.prev_tail = audio[-self.overlap:].copy()
        
        return output