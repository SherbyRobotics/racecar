import pandas as pd
import numpy as np
from pathlib import Path

# NEW/CORRECTED IMPORTS
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message # <-- ADDED THIS IMPORT

# --- SCRIPT CONFIGURATION ---
CONFIG = {
    'BAG_PATH': 'calibration_run_01',  # The folder containing your .mcap bag file
    'STORAGE_ID': 'mcap',             # 'mcap' or 'sqlite3'
    'TOPICS_TO_EXPORT': [
        '/imu/data',
        '/odom'
    ],
    'OUTPUT_PREFIX': 'run_01'       # A prefix for all output filenames
}
# ----------------------------


# ===== MESSAGE PARSER FUNCTIONS =====
def parse_imu_message(msg):
    """Parses a sensor_msgs/msg/Imu message."""
    return {
        'accel_x': msg.linear_acceleration.x,
        'accel_y': msg.linear_acceleration.y,
        'accel_z': msg.linear_acceleration.z,
        'gyro_x': msg.angular_velocity.x,
        'gyro_y': msg.angular_velocity.y,
        'gyro_z': msg.angular_velocity.z,
    }

def parse_odom_message(msg):
    """Parses a nav_msgs/msg/Odometry message."""
    return {
        'pos_x': msg.pose.pose.position.x,
        'pos_y': msg.pose.pose.position.y,
        'pos_z': msg.pose.pose.position.z,
        'orient_x': msg.pose.pose.orientation.x,
        'orient_y': msg.pose.pose.orientation.y,
        'orient_z': msg.pose.pose.orientation.z,
        'orient_w': msg.pose.pose.orientation.w,
        'twist_linear_x': msg.twist.twist.linear.x,
        'twist_angular_z': msg.twist.twist.angular.z,
    }

# ===== REGISTER YOUR PARSERS HERE =====
MESSAGE_PARSERS = {
    'sensor_msgs/msg/Imu': parse_imu_message,
    'nav_msgs/msg/Odometry': parse_odom_message,
}
# ======================================


# --- CORRECTED HELPER FUNCTION ---
def deserialize_ros_message(msg_bytes, msg_type_name):
    """Helper function to deserialize a ROS message using the official rclpy method."""
    msg_type_class = get_message(msg_type_name)
    return deserialize_message(msg_bytes, msg_type_class)
# ----------------------------------


def main():
    bag_path = Path(CONFIG['BAG_PATH'])
    if not bag_path.exists():
        print(f"Error: Bag file path not found at '{bag_path}'")
        return

    # --- 1. Read all messages from the bag ---
    print(f"Reading messages from bag: {bag_path}...")
    storage_options = StorageOptions(uri=str(bag_path), storage_id=CONFIG['STORAGE_ID'])
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    topic_type_map = {topic.name: topic.type for topic in topic_types}

    raw_messages = {topic: [] for topic in CONFIG['TOPICS_TO_EXPORT']}
    
    while reader.has_next():
        topic_name, msg_bytes, timestamp = reader.read_next()
        if topic_name in CONFIG['TOPICS_TO_EXPORT']:
            msg_type_name = topic_type_map[topic_name]
            # --- USE THE CORRECTED HELPER FUNCTION ---
            msg = deserialize_ros_message(msg_bytes, msg_type_name)
            raw_messages[topic_name].append({'timestamp': timestamp, 'msg': msg})
    print("Finished reading messages.")

    # --- 2. Process and structure the data ---
    print("Processing and structuring data...")
    processed_data = {}
    for topic_name, messages in raw_messages.items():
        if not messages:
            print(f"Warning: No messages found for topic '{topic_name}'. Skipping.")
            continue
            
        msg_type_name = topic_type_map[topic_name]
        
        if msg_type_name not in MESSAGE_PARSERS:
            print(f"Warning: No parser available for message type '{msg_type_name}'. Skipping topic '{topic_name}'.")
            continue
            
        parser_func = MESSAGE_PARSERS[msg_type_name]
        
        parsed_rows = []
        for item in messages:
            parsed_msg = parser_func(item['msg'])
            parsed_msg['timestamp_ns'] = item['timestamp']
            parsed_rows.append(parsed_msg)
            
        processed_data[topic_name] = pd.DataFrame(parsed_rows)
    print("Finished processing data.")

    # --- 3. Save the data to files ---
    print("Saving data to .csv and .npy files...")
    output_prefix = CONFIG['OUTPUT_PREFIX']
    for topic_name, df in processed_data.items():
        sanitized_topic_name = topic_name.replace('/', '_').strip('_')
        
        csv_filename = f"{output_prefix}_{sanitized_topic_name}.csv"
        df.to_csv(csv_filename, index=False)
        print(f"  -> Saved {csv_filename}")
        
        npy_filename = f"{output_prefix}_{sanitized_topic_name}.npy"
        np.save(npy_filename, df.to_numpy())
        print(f"  -> Saved {npy_filename}")
        
    print("Export complete.")

if __name__ == "__main__":
    main()