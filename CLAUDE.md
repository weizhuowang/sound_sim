# Sound Sim Development Notes

## Project Overview
Real-time sound synthesis system for robot simulations that captures policy oscillations and motor dynamics through audio feedback.

## Key Design Decisions

### Audio Parameters
- **Sample rate**: 44.1kHz (CD quality standard)
- **Buffer size**: 882 samples (20ms at 44.1kHz, matches 50Hz simulation rate)
- **Non-blocking**: Uses PyAudio streaming with queue to avoid slowing simulation

### State Interface
Required inputs:
- `motor_vel`: Motor velocities (rad/s or m/s)
- `motor_tau`: Motor torques (Nm)
- `contact_force`: Contact forces (optional)

### Sound Mapping
1. **Velocity → Frequency**: Higher velocity = higher pitch
2. **Torque → Amplitude & Harmonics**: Higher torque = louder + strain sounds
3. **Oscillations → Clicks/Buzzes**: Detected via velocity derivatives
4. **Impacts → Thuds**: Sudden force changes

## Architecture

```
sound_sim/
├── core/
│   ├── sound_engine.py      # PyAudio streaming engine
│   ├── mujoco_sound.py      # Main interface & mixing
│   └── utils.py              # Data loading & processing
├── synthesizers/
│   ├── base.py               # Base class + basic synthesizers
│   └── torque_synthesizer.py # Specialized torque sounds
└── examples/
    └── ...                   # Usage examples
```

## Testing Commands

```bash
# Install package
pip install -e .

# Test torque effects clearly
python examples/test_torque_effect.py

# Play real robot data
python examples/play_real_data.py
```

## Known Issues & Solutions

1. **macOS MuJoCo viewer**: Use `mjpython` instead of `python` for viewer
2. **Buffer underruns**: Increase queue size in SoundEngine if needed
3. **Latency**: Reduce buffer_size (but may cause choppy audio)

## Future Improvements

- [ ] GPU acceleration for synthesis
- [ ] Spatial audio (3D sound positioning)
- [ ] Learning-based synthesis from real motor recordings
- [ ] FFT-based spectral synthesis
- [ ] WebAudio support for browser playback