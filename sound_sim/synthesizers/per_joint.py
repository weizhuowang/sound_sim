"""Per-joint synthesizers for individual motor sound synthesis."""

import numpy as np
from typing import Dict, Any, Optional
from sound_sim.synthesizers.base import Synthesizer


class PerJointSynthesizer(Synthesizer):
    """Base class for synthesizers that process each joint individually."""

    def __init__(
        self, sample_rate: int = 44100, buffer_size: int = 882, max_joints: int = 30
    ):
        super().__init__(sample_rate, buffer_size)
        self.max_joints = max_joints
        self.prev_vel = None
        self.prev_tau = None

    def synthesize(self, state: Dict[str, Any]) -> np.ndarray:
        motor_vel = state.get("motor_vel", np.array([]))
        motor_tau = state.get("motor_tau", np.array([]))

        if len(motor_vel) == 0:
            return np.zeros(self.buffer_size)

        num_joints = min(len(motor_vel), self.max_joints)
        output = np.zeros(self.buffer_size)

        # Initialize previous values if needed
        if self.prev_vel is None:
            self.prev_vel = np.zeros_like(motor_vel)
        if self.prev_tau is None:
            self.prev_tau = (
                np.zeros_like(motor_tau)
                if len(motor_tau) > 0
                else np.zeros_like(motor_vel)
            )

        # Process each joint individually
        for i in range(num_joints):
            vel_i = motor_vel[i]
            tau_i = motor_tau[i] if i < len(motor_tau) else 0
            prev_vel_i = self.prev_vel[i] if i < len(self.prev_vel) else 0
            prev_tau_i = self.prev_tau[i] if i < len(self.prev_tau) else 0

            # Generate sound for this joint
            joint_sound = self.synthesize_joint(i, vel_i, tau_i, prev_vel_i, prev_tau_i)

            # Handle sounds that may be longer than buffer_size
            if len(joint_sound) > len(output):
                # Extend output to accommodate longer sound
                output = np.pad(output, (0, len(joint_sound) - len(output)))

            # Mix into output with better scaling
            # Use more aggressive scaling for many joints
            scaling = max(np.sqrt(num_joints), num_joints / 10.0)
            output[: len(joint_sound)] += joint_sound / scaling

        # Update previous values
        self.prev_vel = motor_vel.copy()
        if len(motor_tau) > 0:
            self.prev_tau = motor_tau.copy()

        # Soft clipping to prevent harsh distortion
        output = np.tanh(output * 0.5) * 2.0

        return output

    def synthesize_joint(
        self, joint_idx: int, vel: float, tau: float, prev_vel: float, prev_tau: float
    ) -> np.ndarray:
        """Override this to implement per-joint sound synthesis."""
        return np.zeros(self.buffer_size)


class VelocitySynthesizer(PerJointSynthesizer):
    """Simple velocity-to-frequency mapping for each joint."""

    def __init__(
        self, sample_rate: int = 44100, buffer_size: int = 882, max_joints: int = 30
    ):
        super().__init__(sample_rate, buffer_size, max_joints)
        self.base_freq = 4000.0  # Base frequency per joint
        self.freq_spread = 2000.0  # Frequency spread between joints
        self.vel_scale = 200.0  # How much velocity affects frequency
        self.sample_counter = np.zeros(
            max_joints, dtype=np.int64
        )  # Track samples for continuity
        # Random phase offsets for each joint to prevent constructive interference
        np.random.seed(42)  # Consistent randomness
        self.phase_offsets = np.random.uniform(0, 2 * np.pi, max_joints)

    def synthesize_joint(
        self, joint_idx: int, vel: float, tau: float, prev_vel: float, prev_tau: float
    ) -> np.ndarray:
        # Each joint has slightly different base frequency for richness
        # Logarithmic spacing to avoid beat frequencies
        joint_base_freq = self.base_freq * (
            1.0 + joint_idx * self.freq_spread / (self.max_joints * self.base_freq)
        )

        # Frequency based on velocity magnitude
        frequency = joint_base_freq + np.abs(vel) * self.vel_scale
        frequency = np.clip(frequency, 50, 20000)

        # Amplitude based on velocity magnitude
        amplitude = np.tanh(np.abs(vel) * 2) * 0.04

        # Generate sine wave with phase continuity using time-based approach
        # Calculate phase based on total samples elapsed
        t = (
            self.sample_counter[joint_idx] + np.arange(self.buffer_size)
        ) / self.sample_rate
        # Add phase offset for this joint to prevent constructive interference
        wave = (
            np.sin(2 * np.pi * frequency * t + self.phase_offsets[joint_idx])
            * amplitude
        )

        # Update sample counter for next buffer
        self.sample_counter[joint_idx] += self.buffer_size

        return wave


class DirectionChangeSynthesizer(PerJointSynthesizer):
    """Generates clicks when individual joints change direction."""

    def __init__(
        self, sample_rate: int = 44100, buffer_size: int = 882, max_joints: int = 30
    ):
        super().__init__(sample_rate, buffer_size, max_joints)
        self.click_threshold = 0.3  # Minimum velocity to consider for direction change
        # Random phase offsets for each joint to prevent simultaneous clicks
        np.random.seed(43)  # Different seed from VelocitySynthesizer
        self.phase_offsets = np.random.uniform(0, 2 * np.pi, max_joints)

    def synthesize_joint(
        self, joint_idx: int, vel: float, tau: float, prev_vel: float, prev_tau: float
    ) -> np.ndarray:
        output = np.zeros(self.buffer_size)

        # Detect direction change (sign change with non-zero velocities)
        if (
            np.abs(vel) > self.click_threshold
            and np.abs(prev_vel) > self.click_threshold
            and np.sign(vel) != np.sign(prev_vel)
        ):
            # Impact intensity based on torque change magnitude
            intensity = np.abs(vel - prev_vel) / 5

            # Lower frequency for larger joints (assumed by index)
            impact_freq = 1000 + (1.0 - joint_idx / self.max_joints) * 40

            # Short impact sound (10ms)
            t = np.linspace(0, 0.01, int(0.01 * self.sample_rate))
            impact = np.sin(2 * np.pi * impact_freq * t) * np.exp(-t * 200)

            # Add some noise for texture
            # noise = np.random.randn(len(t)) * 0.1 * intensity
            # impact += noise * np.exp(-t * 300)

            # Add to output
            if len(impact) <= self.buffer_size:
                output[: len(impact)] = impact * intensity

        return output


class TorqueDeltaSynthesizer(PerJointSynthesizer):
    """Generates impacts/thuds based on torque changes per joint."""

    def __init__(
        self, sample_rate: int = 44100, buffer_size: int = 882, max_joints: int = 30
    ):
        super().__init__(sample_rate, buffer_size, max_joints)
        self.delta_threshold = 5  # Minimum torque change to trigger sound

    def synthesize_joint(
        self, joint_idx: int, vel: float, tau: float, prev_vel: float, prev_tau: float
    ) -> np.ndarray:
        output = np.zeros(self.buffer_size)

        # Calculate torque change
        tau_delta = np.abs(tau - prev_tau)

        if tau_delta > self.delta_threshold:
            # Click intensity based on velocity magnitude (capped)
            # print(np.abs(vel - prev_vel))
            intensity = np.clip((tau_delta / 20) ** 3, 0, 1.0) * 0.8

            # Vary frequency slightly per joint for natural variation
            click_freq = 25 + joint_idx * 1

            # Short mechanical click sound (5ms attack, 50ms total)
            t = np.linspace(0, 0.5, int(0.5 * self.sample_rate))

            # Sharp attack with fast decay for plastic/mechanical sound
            envelope = np.exp(-t * 30)  # Much faster decay

            # Mix of frequencies for plastic/mechanical timbre
            click = (
                np.sin(2 * np.pi * click_freq * t) * 0.3  # Fundamental
                + np.sin(2 * np.pi * click_freq * 1.5 * t) * 0.2  # Inharmonic overtone
                + np.sin(2 * np.pi * click_freq * 2.3 * t) * 0.1  # Inharmonic overtone
                + np.sin(2 * np.pi * click_freq * 3.7 * t) * 0.05  # Another inharmonic
            )

            # Add some broadband noise for texture
            noise = np.random.randn(len(t)) * 0.9

            # Combine with envelope
            click = (click + noise) * envelope

            # Return the full click sound - mixer will handle overlapping
            return click * intensity  # Reduce click volume

        return output


class FootStompSynthesizer(PerJointSynthesizer):
    """Generates stomp sounds when feet contact the ground with force."""

    def __init__(
        self, sample_rate: int = 44100, buffer_size: int = 882, max_joints: int = 30
    ):
        super().__init__(sample_rate, buffer_size, max_joints)
        self.force_threshold = 10.0  # Minimum force to trigger stomp
        self.prev_force = None

    def synthesize(self, state: Dict[str, Any]) -> np.ndarray:
        # Look for qfrc in state (ground reaction forces)
        qfrc = state.get("qfrc", np.array([]))

        if len(qfrc) == 0:
            return np.zeros(self.buffer_size)

        # Initialize previous force if needed
        if self.prev_force is None:
            self.prev_force = np.zeros_like(qfrc)

        num_joints = min(len(qfrc), self.max_joints)
        output = np.zeros(self.buffer_size)

        # Process each joint/contact point
        for i in range(num_joints):
            force_i = qfrc[i]
            prev_force_i = self.prev_force[i] if i < len(self.prev_force) else 0

            # Detect new impact (force crosses threshold from below)
            if force_i > self.force_threshold and prev_force_i <= self.force_threshold:
                # Stomp intensity based on force magnitude
                intensity = np.clip(force_i / 100.0, 0, 2.0)

                # Low frequency stomp (50-150 Hz)
                stomp_freq = 20 + (1.0 - i / self.max_joints) * 0

                # Stomp sound (80ms with resonance)
                t = np.linspace(0, 0.6, int(0.6 * self.sample_rate))

                # Deep thud with some harmonics
                stomp = (
                    np.sin(2 * np.pi * stomp_freq * t) * 0.5  # Fundamental
                    + np.sin(2 * np.pi * stomp_freq * 2 * t) * 0.2  # 2nd harmonic
                    + np.sin(2 * np.pi * stomp_freq * 3 * t) * 0.1  # 3rd harmonic
                )

                # Fast attack, slower decay for impact feel
                envelope = np.exp(-t * 20)

                # Add some low-freq noise for texture
                noise = np.random.randn(len(t)) * 0.2
                noise = np.convolve(
                    noise, np.ones(15) / 15, mode="same"
                )  # Low-pass filter

                stomp = (stomp + noise) * envelope * intensity

                # Handle sounds longer than buffer
                if len(stomp) > len(output):
                    output = np.pad(output, (0, len(stomp) - len(output)))

                output[: len(stomp)] += stomp

        # Update previous force
        self.prev_force = qfrc.copy()

        return output
