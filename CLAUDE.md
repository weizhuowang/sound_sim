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
│   ├── audio_mixer.py       # Handles overlapping sounds
│   ├── mujoco_sound.py      # Main interface & mixing
│   └── utils.py              # Data loading & processing
├── synthesizers/
│   ├── base.py               # Base Synthesizer class only
│   └── per_joint.py          # All per-joint synthesizers
└── examples/
    └── ...                   # Usage examples
```

## Design Philosophy (User Preferences)

### Per-Joint Only
- **NO averaging** - Each joint is processed individually
- **Simple components** - Each synthesizer does ONE thing
- **Modular** - Easy to combine different synthesizers
- **Keep it simple** - Avoid complex hybrid synthesizers

### Current Synthesizers
- `VelocitySynthesizer`: Joint velocity → frequency (one tone per joint)
- `DirectionChangeSynthesizer`: Sign changes → clicks
- `TorqueDeltaSynthesizer`: Torque changes → impacts

### User's Coding Practices & Preferences
- **Simplicity first** - Avoid over-engineering, keep components simple
- **Modular design** - Small, composable pieces rather than monolithic systems  
- **No unnecessary files** - Don't create excessive examples or test files
- **Clear separation** - Each component should do ONE thing well
- **Start from scratch when needed** - Build exactly what's needed, not what might be needed
- **Per-joint focus** - Want to hear individual joint behaviors, not averaged signals
- **Avoid hybrids** - Don't mix concepts in a single synthesizer
- **Documentation in code** - Update CLAUDE.md and README.md with design decisions

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