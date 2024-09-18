import os
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
from scipy.interpolate import interp1d

def read_tum_file(file_path):
    """
    Reads a TUM format file.
    """
    timestamps = []
    poses = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            data = line.strip().split()
            timestamp = float(data[0])
            pose = list(map(float, data[1:]))
            timestamps.append(timestamp)
            poses.append(pose)
    return np.array(timestamps), np.array(poses)

def write_tum_file(file_path, timestamps, poses):
    """
    Writes a TUM format file.
    """
    with open(file_path, 'w+') as file:
        for timestamp, pose in zip(timestamps, poses):
            file.write(f"{timestamp:.6f} {' '.join(map(str, pose))}\n")

def interpolate_poses(timestamps_ref, timestamps, poses):
    """
    Interpolates poses to match the reference timestamps.
    """
    # Interpolate translation (x, y, z)
    interp_funcs = [interp1d(timestamps, poses[:, i], kind='linear', fill_value='extrapolate') for i in range(3)]
    interpolated_translations = np.array([func(timestamps_ref) for func in interp_funcs]).T

    # Interpolate rotation (quaternions) using Slerp
    rotations = R.from_quat(poses[:, 3:7])
    slerp = Slerp(timestamps, rotations)
    interpolated_rotations = slerp(timestamps_ref).as_quat()

    # Combine translations and rotations
    interpolated_poses = np.hstack((interpolated_translations, interpolated_rotations))
    return interpolated_poses

def main(tum_file_1, tum_file_2, output_file):
    # Read TUM files
    timestamps_1, poses_1 = read_tum_file(tum_file_1)
    timestamps_2, poses_2 = read_tum_file(tum_file_2)

    # Interpolate poses from the first file to match the timestamps of the second file
    interpolated_poses = interpolate_poses(timestamps_2, timestamps_1, poses_1)

    # Write the interpolated poses to the output file
    write_tum_file(output_file, timestamps_2, interpolated_poses)

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 4:
        print("Usage: python script.py <tum_file_1> <tum_file_2> <output_file>")
    else:
        tum_file_1 = sys.argv[1]
        tum_file_2 = sys.argv[2]
        output_file = sys.argv[3]
        main(tum_file_1, tum_file_2, output_file)
