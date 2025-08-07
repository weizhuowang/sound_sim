import json
import numpy as np
from typing import Dict, Any, Optional, List, Union


def load_json_data(filepath: str) -> Union[Dict[str, Any], List[Dict[str, Any]]]:
    """Load JSON data from file."""
    with open(filepath, 'r') as f:
        data = json.load(f)
    return data


def timeseries_to_arrays(data: List[Dict[str, Any]]) -> Dict[str, np.ndarray]:
    """Convert time series data (list of dicts) to dict of numpy arrays.
    
    Input: [{timestamp: t1, tau: [1,2,3], ...}, {timestamp: t2, tau: [4,5,6], ...}]
    Output: {'timestamp': [t1, t2, ...], 'tau': [[1,2,3], [4,5,6], ...], ...}
    """
    if not data:
        return {}
    
    # Get all keys from first item
    keys = data[0].keys()
    
    # Initialize result dict
    result = {key: [] for key in keys}
    
    # Collect all values
    for item in data:
        for key in keys:
            if key in item:
                result[key].append(item[key])
    
    # Convert to numpy arrays
    for key in keys:
        result[key] = np.array(result[key])
    
    return result


def compute_velocities(positions: np.ndarray, dt: float = 0.02) -> np.ndarray:
    """Compute velocities from position data using finite differences.
    
    Args:
        positions: (timesteps, n_joints) array of positions
        dt: timestep in seconds
    
    Returns:
        velocities: (timesteps, n_joints) array
    """
    velocities = np.zeros_like(positions)
    
    # Use central differences for middle points
    velocities[1:-1] = (positions[2:] - positions[:-2]) / (2 * dt)
    
    # Use forward difference for first point
    velocities[0] = (positions[1] - positions[0]) / dt
    
    # Use backward difference for last point
    velocities[-1] = (positions[-1] - positions[-2]) / dt
    
    return velocities


def inspect_data(data: Union[Dict[str, Any], List[Dict]], max_items: int = 5) -> None:
    """Print summary of data structure and shapes."""
    
    if isinstance(data, list):
        print(f"Time series data: {len(data)} timesteps")
        if data:
            print(f"Keys in each timestep: {list(data[0].keys())}")
            
            # Convert to arrays for better inspection
            print("\nConverting to arrays...")
            data = timeseries_to_arrays(data)
    
    print("\nData structure:")
    print("-" * 60)
    
    for key, value in data.items():
        if isinstance(value, np.ndarray):
            print(f"{key:20s}: array shape {value.shape}, dtype={value.dtype}")
            if len(value.shape) == 1 and len(value) <= max_items:
                print(f"  → values: {value}")
            elif len(value.shape) == 2 and value.shape[0] <= max_items:
                print(f"  → first {min(max_items, value.shape[0])} samples:")
                for i in range(min(max_items, value.shape[0])):
                    sample = value[i]
                    if len(sample) > 10:
                        print(f"    [{i}]: [{sample[0]:.3f}, {sample[1]:.3f}, ... {sample[-1]:.3f}]")
                    else:
                        print(f"    [{i}]: {sample}")
        elif isinstance(value, (list, tuple)):
            print(f"{key:20s}: {type(value).__name__} with {len(value)} items")
        elif isinstance(value, (int, float, str, bool)):
            print(f"{key:20s}: {type(value).__name__} = {value}")
        else:
            print(f"{key:20s}: {type(value).__name__}")
    print("-" * 60)


def prepare_motor_data(data: Union[Dict[str, np.ndarray], List[Dict]], 
                       compute_vel_from_pos: bool = True,
                       dt: float = 0.02) -> Dict[str, np.ndarray]:
    """Prepare motor data for sound synthesis.
    
    Returns dict with 'motor_vel', 'motor_tau' arrays ready for sound synthesis.
    """
    # Convert time series to arrays if needed
    if isinstance(data, list):
        data = timeseries_to_arrays(data)
    
    result = {}
    
    # Get torques (tau)
    if 'tau' in data:
        result['motor_tau'] = data['tau']
    elif 'motor_tau' in data:
        result['motor_tau'] = data['motor_tau']
    elif 'torque' in data:
        result['motor_tau'] = data['torque']
    
    # Get velocities - compute from positions if needed
    if 'dof_vel' in data:
        result['motor_vel'] = data['dof_vel']
    elif 'motor_vel' in data:
        result['motor_vel'] = data['motor_vel']
    elif compute_vel_from_pos:
        # Try to compute from position data
        pos_key = None
        for key in ['body_dof_pos', 'dof_pos', 'joint_pos', 'qpos']:
            if key in data:
                pos_key = key
                break
        
        if pos_key:
            positions = data[pos_key]
            result['motor_vel'] = compute_velocities(positions, dt)
            print(f"Computed velocities from {pos_key} with dt={dt}")
    
    return result


def load_and_prepare(filepath: str, dt: float = 0.02) -> Dict[str, np.ndarray]:
    """Load JSON and prepare for sound synthesis in one step."""
    raw_data = load_json_data(filepath)
    return prepare_motor_data(raw_data, compute_vel_from_pos=True, dt=dt)


# Quick test/example
if __name__ == "__main__":
    import os
    
    # Load the JSON file
    json_path = "data/g1_stu_future_real_recordings_20250805_213834.json"
    
    if os.path.exists(json_path):
        print(f"Loading {json_path}...")
        raw_data = load_json_data(json_path)
        
        print("\nInspecting data structure...")
        inspect_data(raw_data)
        
        print("\nPreparing motor data for sound synthesis...")
        motor_data = prepare_motor_data(raw_data)
        
        if motor_data:
            print("\nPrepared data:")
            for key, val in motor_data.items():
                print(f"  {key}: shape {val.shape}")
                print(f"    min={val.min():.3f}, max={val.max():.3f}, mean={val.mean():.3f}")
    else:
        print(f"File not found: {json_path}")