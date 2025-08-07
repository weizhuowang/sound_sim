"""
Sound Sim - Real-time sound synthesis for robot simulations
"""

from sound_sim.core.mujoco_sound import MujocoSoundSystem, step_with_sound
from sound_sim.core.sound_engine import SoundEngine
from sound_sim.core.utils import (
    load_json_data,
    timeseries_to_arrays,
    compute_velocities,
    prepare_motor_data,
    load_and_prepare,
    inspect_data,
)

from sound_sim.synthesizers.base import (
    Synthesizer,
    JointSynthesizer,
    OscillationSynthesizer,
    ContactSynthesizer,
)
from sound_sim.synthesizers.torque_synthesizer import TorqueSynthesizer

__version__ = "0.1.0"

__all__ = [
    # Core
    "MujocoSoundSystem",
    "step_with_sound",
    "SoundEngine",
    # Utils
    "load_json_data",
    "timeseries_to_arrays",
    "compute_velocities",
    "prepare_motor_data",
    "load_and_prepare",
    "inspect_data",
    # Synthesizers
    "Synthesizer",
    "JointSynthesizer",
    "OscillationSynthesizer",
    "ContactSynthesizer",
    "TorqueSynthesizer",
]