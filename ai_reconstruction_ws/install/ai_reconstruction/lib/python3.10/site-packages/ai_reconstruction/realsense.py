import pyrealsense2 as rs
import numpy as np
import cv2
import os
import glob
import yaml
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener

def main(args=None):
    print("Initialized succesfully")
    # Create folders to store the frames
    rgb_folder = '/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/rgb'
    depth_folder = '/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/depth'
    groundtruth_file_path = '/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/groundtruth.txt'

    # Check if the file exists and is not empty
    if os.path.exists(groundtruth_file_path) and os.path.getsize(groundtruth_file_path) > 0:
        os.remove(groundtruth_file_path)  # Delete the file

        os.makedirs(rgb_folder, exist_ok=True)
        os.makedirs(depth_folder, exist_ok=True)

    # Delete existing frames in the folders
    for file in glob.glob(os.path.join(rgb_folder, '*.png')):
        os.remove(file)
    for file in glob.glob(os.path.join(depth_folder, '*.png')):
        os.remove(file)

    ####################################################################################################
    #camera calibration
    #modify this part

    pipe = rs.pipeline()
    cfg = rs.config()
    h, w = 720, 1280
    depth_scale = 0 
    cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, 30)

    profile = pipe.start(cfg)
    align_to = rs.stream.color
    align = rs.align(align_to)

    rgb_sensor = profile.get_device().query_sensors()[1]
    rgb_sensor.set_option(rs.option.enable_auto_exposure, False)
    rgb_sensor.set_option(rs.option.enable_auto_white_balance, False)
    rgb_sensor.set_option(rs.option.exposure, 200)
    rgb_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
    rgb_intrinsics = rgb_profile.get_intrinsics()

    fx = rgb_intrinsics.fx
    fy = rgb_intrinsics.fy
    cx = rgb_intrinsics.ppx
    cy = rgb_intrinsics.ppy
    width = rgb_intrinsics.width
    height = rgb_intrinsics.height

    disorted = False

    # Initialize tf of camera
    rclpy.init(args=args)
    node = Node('position_listener')
    tf_buffer = Buffer()
    listener = TransformListener(tf_buffer, node)

    ####################################################################################################
    #frame capturing

    while True:
        # Allow ROS2 to process incoming messages, updating the buffer
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            # Wait for a coherent pair of frames: depth and color
            frames = pipe.wait_for_frames()

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get the aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame()
            aligned_color_frame = aligned_frames.get_color_frame()

            # Get the tf
            transform: TransformStamped = tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time())
            # Extract position and rotation
            position = transform.transform.translation
            rotation = transform.transform.rotation

            if not aligned_depth_frame or not aligned_color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            color_image = np.asanyarray(aligned_color_frame.get_data())

            cv2.imshow('rgb', color_image)
            cv2.imshow('depth', depth_image)

            # Save frames to the respective folders using time stamp
            timestamp = str(int(time.time() * 1000000))  # Microsecond timestamp
            rgb_filename = os.path.join(rgb_folder, f'{timestamp}.png')
            depth_filename = os.path.join(depth_folder, f'{timestamp}.png')
            cv2.imwrite(rgb_filename, color_image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
            cv2.imwrite(depth_filename, depth_image, [cv2.IMWRITE_PNG_COMPRESSION, 0])
        
            # Create groundtruth.txt and write the positions and rotations
            with open('/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/groundtruth.txt', 'a') as file:
                file.write(f'{position.x} {position.y} {position.z} {rotation.x} {rotation.y} {rotation.z} {rotation.w}\n')

            if cv2.waitKey(1) == ord('q'):
                break
        except Exception as e:
            node.get_logger().info('Waiting for transform from base_footprint to map...')

    pipe.stop()

    ####################################################################################################
    # Get the list of PNG files in the depthrealsense folder
    depth_files = [f for f in os.listdir(depth_folder) if f.endswith('.png')]
    depth_files = sorted(depth_files, key=lambda x: int(x[:-4]))

    # Delete existing depthfile.txt if it exists
    if os.path.exists('/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/depth.txt'):
        os.remove('/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/depth.txt')

    # Create depthfile.txt and write the filenames
    with open('/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/depth.txt', 'w') as file:
        for depth_file in depth_files:
            filename = depth_file[:-4]  # Remove the file extension (.png)
            depth_filepath = os.path.join(depth_folder, depth_file)
            file.write(f'{filename} {depth_filepath}\n')

    ####################################################################################################
    # Get the list of PNG files in the RGB realsense folder
    rgb_files = [f for f in os.listdir(rgb_folder) if f.endswith('.png')]
    rgb_files = sorted(rgb_files, key=lambda x: int(x[:-4]))

    # Delete existing rgb.txt if it exists
    if os.path.exists('/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/rgb.txt'):
        os.remove('/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/rgb.txt')

    # Create rgb.txt and write the filenames
    with open('/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset/rgb.txt', 'w') as file:
        for rgb_file in rgb_files:
            rgb_filepath = os.path.join(rgb_folder, rgb_file)
            file.write(f'{rgb_filepath}\n')

    #####################################################################################################
    # Write the calibration parameters to a YAML file

    config_file = os.path.join("/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset", "calibration.yaml")
    config = {
        "inherit_from": "configs/rgbd/tum/base_config.yaml",
        "Dataset": {
            "dataset_path": "/mnt/nova_ssd/workspaces/ai_reconstruction_ws/our_dataset"
        },
        "Calibration": {
            "fx": fx,
            "fy": fy,
            "cx": cx,
            "cy": cy,
            "k1": 0,
            "k2": 0,
            "p1": 0,
            "p2": 0,
            "k3": 0,
            "distorted": disorted,
            "width": w,
            "height": h,
            "depth_scale": depth_scale
        }
    }

    with open(config_file, "w") as f:
        yaml.dump(config, f, default_flow_style=False)

if __name__ == '__main__':
    main()
