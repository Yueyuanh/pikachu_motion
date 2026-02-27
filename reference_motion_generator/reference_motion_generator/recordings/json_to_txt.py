#!/usr/bin/env python3
"""
JSON to TXT Converter for Motion Capture Data

This script converts JSON files containing motion capture data to TXT format.
The script supports both single file conversion and batch processing of directories.
"""

import argparse
import json
import os
import sys
from pathlib import Path


def convert_json_to_txt(input_path, output_path):
    """
    Convert a single JSON file to TXT format with the correct structure.
    
    Args:
        input_path (str): Path to the input JSON file
        output_path (str): Path to the output TXT file
    """
    # Read the JSON file
    with open(input_path, 'r', encoding='utf-8') as f:
        data = json.load(f)
    
    # Check if this is a hardware format (has more fields) or just a basic format
    # The original format from gait generator has additional fields like FPS, Joints, etc.
    # but the target format should only have LoopMode, FrameDuration, etc.
    
    # Get the frame structure info
    if not data['Frames'] or not isinstance(data['Frames'], list):
        raise ValueError("Frames key must contain a list of frame data")
    
    if not data['Frames']:
        raise ValueError("Frames list is empty")
    
    # Process frames according to the required format
    # Original JSON format from gait_generator:
    # [root_position(3) + root_orientation_quat(4) + joints_positions(n) + 
    #  left_toe_pos(3) + right_toe_pos(3) + world_linear_vel(3) + 
    #  world_angular_vel(3) + joints_vel(n) + left_toe_vel(3) + 
    #  right_toe_vel(3) + foot_contacts(2)]
    #
    # Target format:
    # [x, y, z, qx, qy, qz, qw, j1, j2, j3, ..., l_toe_x, l_toe_y, l_toe_z, 
    #  r_toe_x, r_toe_y, r_toe_z, lin_vel_x, lin_vel_y, lin_vel_z, 
    #  ang_vel_x, ang_vel_y, ang_vel_z, j1_vel, j2_vel, ...,
    #  l_toe_vel_x, l_toe_vel_y, l_toe_vel_z, r_toe_vel_x, r_toe_vel_y, r_toe_vel_z]
    
    processed_frames = []
    for frame_idx, frame in enumerate(data['Frames']):
        if not isinstance(frame, list) or len(frame) < 13:  # Minimum expected elements
            raise ValueError(f"Frame {frame_idx} has invalid format or insufficient data")
        
        # Parse the frame data based on the Frame_offset info if available
        # If not, we'll determine it from the first frame structure
        if 'Frame_offset' in data and data['Frame_offset']:
            offsets = data['Frame_offset'][0]
            sizes = data['Frame_size'][0] if 'Frame_size' in data and data['Frame_size'] else None
            
            if sizes:
                joints_count = sizes['joints_pos']
            else:
                # Calculate joints count based on remaining fields after known fixed sizes
                expected_remaining = (sizes['left_toe_pos'] if sizes else 3) + \
                                   (sizes['right_toe_pos'] if sizes else 3) + \
                                   (sizes['world_linear_vel'] if sizes else 3) + \
                                   (sizes['world_angular_vel'] if sizes else 3) + \
                                   (sizes['left_toe_vel'] if sizes else 3) + \
                                   (sizes['right_toe_vel'] if sizes else 3) + \
                                   (sizes['foot_contacts'] if sizes else 2)
                joints_count = len(frame) - 3 - 4 - expected_remaining  # 3 pos + 4 quat
        else:
            # Calculate based on expected structure: left_toe_pos(3) + right_toe_pos(3) + 
            # world_linear_vel(3) + world_angular_vel(3) + joints_vel(? same as joints) + 
            # left_toe_vel(3) + right_toe_vel(3) + foot_contacts(2)
            # So if total frame length is: 3 + 4 + joints + 3 + 3 + 3 + 3 + joints + 3 + 3 + 2
            # = 24 + 2*joints, so joints = (len(frame) - 24) / 2
            expected_remaining = 3 + 3 + 3 + 3  # left_toe_pos + right_toe_pos + world_linear_vel + world_angular_vel
            expected_remaining += 3 + 3  # left_toe_vel + right_toe_vel
            expected_remaining += 2  # foot_contacts
            joints_count = (len(frame) - 3 - 4 - expected_remaining) // 2

        if joints_count < 0:
            raise ValueError(f"Frame {frame_idx} has invalid structure")
        
        # Parse frame components
        idx = 0
        # 1. root_position (3 elements: x, y, z)
        if idx + 3 > len(frame):
            raise ValueError(f"Frame {frame_idx} missing root_position data")
        root_position = frame[idx:idx+3]
        idx += 3
        
        # 2. root_orientation_quat (4 elements: qx, qy, qz, qw in original format)
        if idx + 4 > len(frame):
            raise ValueError(f"Frame {frame_idx} missing root_orientation_quat data")
        qx, qy, qz, qw = frame[idx:idx+4]
        idx += 4
        
        # 3. joints_positions (variable length)
        if idx + joints_count > len(frame):
            raise ValueError(f"Frame {frame_idx} missing joints_positions data")
        joints_positions = frame[idx:idx+joints_count]
        idx += joints_count
        
        # 4. left_toe_pos (3 elements)
        if idx + 3 > len(frame):
            raise ValueError(f"Frame {frame_idx} missing left_toe_pos data")
        left_toe_pos = frame[idx:idx+3]
        idx += 3
        
        # 5. right_toe_pos (3 elements)
        if idx + 3 > len(frame):
            raise ValueError(f"Frame {frame_idx} missing right_toe_pos data")
        right_toe_pos = frame[idx:idx+3]
        idx += 3
        
        # 6. world_linear_vel (3 elements)
        if idx + 3 > len(frame):
            raise ValueError(f"Frame {frame_idx} missing world_linear_vel data")
        world_linear_vel = frame[idx:idx+3]
        idx += 3
        
        # 7. world_angular_vel (3 elements)
        if idx + 3 > len(frame):
            raise ValueError(f"Frame {frame_idx} missing world_angular_vel data")
        world_angular_vel = frame[idx:idx+3]
        idx += 3
        
        # 8. joints_vel (same count as joints_positions)
        if idx + joints_count > len(frame):
            raise ValueError(f"Frame {frame_idx} missing joints_vel data")
        joints_vel = frame[idx:idx+joints_count]
        idx += joints_count
        
        # 9. left_toe_vel (3 elements)
        if idx + 3 > len(frame):
            raise ValueError(f"Frame {frame_idx} missing left_toe_vel data")
        left_toe_vel = frame[idx:idx+3]
        idx += 3
        
        # 10. right_toe_vel (3 elements)
        if idx + 3 > len(frame):
            raise ValueError(f"Frame {frame_idx} missing right_toe_vel data")
        right_toe_vel = frame[idx:idx+3]
        idx += 3
        
        # 11. foot_contacts (2 elements) - may not be present in all formats
        if idx + 2 <= len(frame):
            foot_contacts = frame[idx:idx+2]
        elif idx == len(frame):
            # If we've reached the end and there are no foot contacts, that's ok for some formats
            foot_contacts = [0, 0]  # Default values
        else:
            raise ValueError(f"Frame {frame_idx} has unexpected structure")
        
        # Create the new frame in the target format
        # [x, y, z, qx, qy, qz, qw, j1, j2, j3, ..., l_toe_x, l_toe_y, l_toe_z, 
        #  r_toe_x, r_toe_y, r_toe_z, lin_vel_x, lin_vel_y, lin_vel_z, 
        #  ang_vel_x, ang_vel_y, ang_vel_z, j1_vel, j2_vel, ...,
        #  l_toe_vel_x, l_toe_vel_y, l_toe_vel_z, r_toe_vel_x, r_toe_vel_y, r_toe_vel_z]
        new_frame = (
            root_position +           # 3: x, y, z
            [qx, qy, qz, qw] +       # 4: qx, qy, qz, qw
            joints_positions +        # n: joint positions
            left_toe_pos +           # 3: l_toe_x, l_toe_y, l_toe_z
            right_toe_pos +          # 3: r_toe_x, r_toe_y, r_toe_z
            world_linear_vel +       # 3: lin_vel_x, lin_vel_y, lin_vel_z
            world_angular_vel +      # 3: ang_vel_x, ang_vel_y, ang_vel_z
            joints_vel +             # n: joint velocities
            left_toe_vel +           # 3: l_toe_vel_x, l_toe_vel_y, l_toe_vel_z
            right_toe_vel            # 3: r_toe_vel_x, r_toe_vel_y, r_toe_vel_z
        )
        
        processed_frames.append(new_frame)
    
    # Create the output data structure with only the required fields for the target format
    output_data = {
        "LoopMode": data.get("LoopMode", "Wrap"),
        "FrameDuration": data.get("FrameDuration", 0.021),
        "EnableCycleOffsetPosition": data.get("EnableCycleOffsetPosition", True),
        "EnableCycleOffsetRotation": data.get("EnableCycleOffsetRotation", True),
        "QuaternionOrder": "xyzw",
        "MotionWeight": data.get("MotionWeight", 0.5),
        "Frames": processed_frames
    }
    
    # Write the output file in a line-by-line format
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write('{\n')
        f.write('  "LoopMode": ' + json.dumps(output_data["LoopMode"]) + ',\n')
        f.write('  "FrameDuration": ' + json.dumps(output_data["FrameDuration"]) + ',\n')
        f.write('  "EnableCycleOffsetPosition": ' + json.dumps(output_data["EnableCycleOffsetPosition"]) + ',\n')
        f.write('  "EnableCycleOffsetRotation": ' + json.dumps(output_data["EnableCycleOffsetRotation"]) + ',\n')
        f.write('  "QuaternionOrder": ' + json.dumps(output_data["QuaternionOrder"]) + ',\n')
        f.write('  "MotionWeight": ' + json.dumps(output_data["MotionWeight"]) + ',\n')
        f.write('  "Frames": [\n')
        
        # Write each frame on its own line
        frames = output_data["Frames"]
        for i, frame in enumerate(frames):
            frame_str = json.dumps(frame)
            if i < len(frames) - 1:
                f.write('    ' + frame_str + ',\n')
            else:
                f.write('    ' + frame_str + '\n')
        
        f.write('  ]\n')
        f.write('}\n')


def process_directory(input_dir, output_dir):
    """
    Process all JSON files in a directory and convert them to TXT format.
    
    Args:
        input_dir (str): Path to the input directory
        output_dir (str): Path to the output directory
    """
    input_path = Path(input_dir)
    output_path = Path(output_dir)
    
    # Create output directory if it doesn't exist
    output_path.mkdir(parents=True, exist_ok=True)
    
    # Find all JSON files in the input directory
    json_files = list(input_path.glob("*.json"))
    
    if not json_files:
        print(f"No JSON files found in {input_dir}")
        return
    
    print(f"Found {len(json_files)} JSON files to convert")
    
    for json_file in json_files:
        try:
            # Create output file name (with .txt extension instead of .json)
            output_file = output_path / json_file.with_suffix('.txt').name
            
            print(f"Converting {json_file.name} -> {output_file.name}")
            convert_json_to_txt(str(json_file), str(output_file))
            print(f"Successfully converted {json_file.name}")
        except Exception as e:
            print(f"Error converting {json_file.name}: {str(e)}")
            continue


def main():
    parser = argparse.ArgumentParser(
        description="Convert JSON motion capture files to TXT format"
    )
    parser.add_argument(
        "--input_file", 
        type=str, 
        help="Path to a single input JSON file"
    )
    parser.add_argument(
        "--input_dir", 
        type=str, 
        help="Path to input directory containing JSON files"
    )
    parser.add_argument(
        "--output", 
        type=str, 
        required=True, 
        help="Path to output file (for single file) or output directory (for batch)"
    )
    
    args = parser.parse_args()
    
    # Validate arguments
    if not args.input_file and not args.input_dir:
        parser.error("Either --input_file or --input_dir must be specified")
    
    if args.input_file and args.input_dir:
        parser.error("Only one of --input_file or --input_dir can be specified")
    
    try:
        if args.input_file:
            # Single file conversion
            if not os.path.isfile(args.input_file):
                raise FileNotFoundError(f"Input file does not exist: {args.input_file}")
            
            if os.path.isdir(args.output):
                # If output is a directory, create output file in that directory
                input_filename = Path(args.input_file).stem
                output_path = Path(args.output) / f"{input_filename}.txt"
            else:
                # Output is a file path
                output_path = args.output
            
            convert_json_to_txt(args.input_file, output_path)
            print(f"Successfully converted {args.input_file} to {output_path}")
        
        elif args.input_dir:
            # Directory conversion
            if not os.path.isdir(args.input_dir):
                raise FileNotFoundError(f"Input directory does not exist: {args.input_dir}")
            
            process_directory(args.input_dir, args.output)
            print(f"Batch conversion completed!")
    
    except Exception as e:
        print(f"Error: {str(e)}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
