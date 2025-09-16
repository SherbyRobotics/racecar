import numpy as np
from rosbag2_py import SequentialReader
from rosbag2_py import StorageOptions, ConverterOptions
from rosidl_runtime_py.utilities import get_message

def deserialize_message(msg_bytes, msg_type):
    """Helper function to deserialize a ROS message."""
    return get_message(msg_type).deserialize(msg_bytes)

def export_imu_to_npy(bag_file_path, topic_name, output_npy_path):
    """
    Reads a ROS2 bag file and exports IMU data from a specific topic to a .npy file.
    """
    # THE ONLY CHANGE IS HERE: storage_id is now 'mcap'
    storage_options = StorageOptions(uri=bag_file_path, storage_id='mcap')
    
    converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
    
    reader = SequentialReader()
    reader.open(storage_options, converter_options)
    
    topic_types = reader.get_all_topics_and_types()
    topic_type_map = {topic.name: topic.type for topic in topic_types}
    
    if topic_name not in topic_type_map:
        print(f"Topic '{topic_name}' not found in the bag file.")
        return
        
    msg_type = topic_type_map[topic_name]

    # Prepare a list to hold the data rows
    data_rows = []

    while reader.has_next():
        topic, msg_bytes, timestamp = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(msg_bytes, msg_type)
            
            # Create a row of the data you want to save
            row = [
                timestamp,
                msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
            ]
            data_rows.append(row)

    # Convert the list of rows to a NumPy array and save to .npy file
    np_array = np.array(data_rows)
    np.save(output_npy_path, np_array)
    print(f"Data from topic '{topic_name}' exported to '{output_npy_path}'")
    print(f"Array shape: {np_array.shape}")

# --- Main execution ---
if __name__ == "__main__":
    BAG_FILE = 'data'  # Your .mcap bag folder
    TOPIC = '/prob_cmd'
    OUTPUT_NPY = 'data.npy'
    
    export_imu_to_npy(BAG_FILE, TOPIC, OUTPUT_NPY)