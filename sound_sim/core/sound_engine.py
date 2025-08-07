import numpy as np
import pyaudio
import threading
from queue import Queue
from typing import Optional


class SoundEngine:
    def __init__(self, sample_rate: int = 44100, buffer_size: int = 882):
        self.sample_rate = sample_rate
        self.buffer_size = buffer_size
        self.audio_queue = Queue(maxsize=10)
        self.is_running = False
        self.stream: Optional[pyaudio.Stream] = None
        self.pa = pyaudio.PyAudio()
        self.thread: Optional[threading.Thread] = None
        
    def _audio_callback(self, in_data, frame_count, time_info, status):
        if not self.audio_queue.empty():
            audio_data = self.audio_queue.get()
            if len(audio_data) < frame_count:
                audio_data = np.pad(audio_data, (0, frame_count - len(audio_data)))
            elif len(audio_data) > frame_count:
                audio_data = audio_data[:frame_count]
            return (audio_data.astype(np.float32).tobytes(), pyaudio.paContinue)
        else:
            return (np.zeros(frame_count, dtype=np.float32).tobytes(), pyaudio.paContinue)
    
    def start(self):
        if not self.is_running:
            self.stream = self.pa.open(
                format=pyaudio.paFloat32,
                channels=1,
                rate=self.sample_rate,
                output=True,
                frames_per_buffer=self.buffer_size,
                stream_callback=self._audio_callback
            )
            self.stream.start_stream()
            self.is_running = True
    
    def stop(self):
        if self.is_running:
            self.is_running = False
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            self.pa.terminate()
    
    def play(self, audio_data: np.ndarray):
        if self.is_running and not self.audio_queue.full():
            normalized = np.clip(audio_data, -1.0, 1.0)
            self.audio_queue.put(normalized)
    
    def __del__(self):
        self.stop()