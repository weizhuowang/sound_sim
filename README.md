# Sound Sim

Real-time sound synthesis for robot simulations (MuJoCo, Isaac Gym, etc.)

## Features

- 🎵 **Real-time synthesis** at 44.1kHz without slowing down 50Hz simulations
- 🤖 **Motor sounds** that respond to velocity AND torque
- 📊 **Oscillation detection** - hear when your policy is jerky
- 💥 **Impact sounds** for contacts (optional)
- 🔧 **Modular synthesizers** - easy to extend and customize
- 🎛️ **Simple API** - one line to add sound to any simulation

## Installation

```bash
# Install as editable package
pip install -e .

# With MuJoCo support
pip install -e ".[mujoco]"

# For running examples
pip install -e ".[examples]"
```

## Quick Start

### Simplest Usage

```python
from sound_sim import step_with_sound
import numpy as np

# In your simulation loop
for t in range(steps):
    motor_vel = np.array([...])  # Your motor velocities
    motor_tau = np.array([...])  # Your motor torques
    
    # Just one line!
    step_with_sound(motor_vel=motor_vel, motor_tau=motor_tau)
```

### With MuJoCo

```python
import mujoco
from sound_sim import step_with_sound

model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)

while running:
    mujoco.mj_step(model, data)
    step_with_sound(data)  # Automatically extracts motor data
```

### Advanced Control

```python
from sound_sim import MujocoSoundSystem, VelocitySynthesizer, DirectionChangeSynthesizer

# Per-joint sound system (hear individual motors)
sound = MujocoSoundSystem(
    synthesizers=[
        VelocitySynthesizer(),       # Each joint gets own frequency
        DirectionChangeSynthesizer(), # Clicks on direction changes
    ],
    include_contact=False
)
sound.start()
sound.set_volume(0.7)

# In your loop
sound.step(motor_vel=vel, motor_tau=tau)
```

## Working with Real Data

```python
from sound_sim import load_and_prepare

# Load robot recording
motor_data = load_and_prepare("path/to/recording.json")

# Play through the data
for t in range(len(motor_data['motor_vel'])):
    step_with_sound(
        motor_vel=motor_data['motor_vel'][t],
        motor_tau=motor_data['motor_tau'][t]
    )
```

## Sound Components (Per-Joint Only)

All synthesizers process each joint individually - no averaging!

### Available Synthesizers

- **VelocitySynthesizer**: Each joint gets its own frequency based on velocity
- **DirectionChangeSynthesizer**: Clicks when individual joints change direction  
- **TorqueDeltaSynthesizer**: Impacts when individual joints have torque spikes

### What You'll Hear

- **Multiple tones** - One frequency per joint (not a single averaged tone)
- **Individual clicks** - Can identify which specific joint is reversing
- **Isolated impacts** - Torque spikes on specific joints are audible
- **Rich soundscape** - Complex but informative, helps identify problematic joints

## Examples

```bash
# Test with synthetic data
python examples/test_sound.py

# Clear torque effect demo
python examples/test_torque_effect.py

# Play real robot data
python examples/play_real_data.py

# MuJoCo humanoid (macOS: use mjpython)
python examples/example_humanoid.py
```

## Custom Synthesizers

```python
from sound_sim import Synthesizer
import numpy as np

class MySynthesizer(Synthesizer):
    def synthesize(self, state):
        motor_vel = state.get('motor_vel', np.array([]))
        motor_tau = state.get('motor_tau', np.array([]))
        
        # Generate audio (return buffer_size samples)
        frequency = 440 + np.mean(motor_vel) * 100
        return np.sin(2 * np.pi * frequency * self.time) * 0.5
```

## API Reference

### Core Functions

- `step_with_sound(data=None, motor_vel=None, motor_tau=None, contact_force=None)`
- `MujocoSoundSystem(sample_rate=44100, buffer_size=882, synthesizers=None, include_contact=True)`

### Data Utils

- `load_and_prepare(filepath, dt=0.02)` - Load JSON and prepare motor data
- `compute_velocities(positions, dt=0.02)` - Compute velocities from positions
- `timeseries_to_arrays(data)` - Convert list of dicts to dict of arrays

## Requirements

- Python >= 3.8
- NumPy >= 1.24.0
- PyAudio >= 0.2.11
- SciPy >= 1.10.0
- MuJoCo >= 3.0.0 (optional)

## License

MIT