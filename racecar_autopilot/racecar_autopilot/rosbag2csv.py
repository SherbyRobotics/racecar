"""Convert ROS 2 bag to CSV files, one CSV file per topic."""


# Copyright 2020 Open Source Robotics Foundation, Inc.
# Copyright 2023, 2024 Michal Sojka <michal.sojka@cvut.cz>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import os.path
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

if os.environ.get('ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL', None) is not None:
    # This is needed on Linux when compiling with clang/libc++.
    # TL;DR This makes class_loader work when using a python extension compiled with libc++.
    #
    # For the fun RTTI ABI details, see https://whatofhow.wordpress.com/2015/03/17/odr-rtti-dso/.
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)

import rosbag2_py  # noqa


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='mcap')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def _gen_msg_values(msg, prefix=""):
    if isinstance(msg, list):
        for i, val in enumerate(msg):
            yield from _gen_msg_values(val, f"{prefix}[{i}]")
    elif hasattr(msg, "get_fields_and_field_types"):
        for field, type_ in msg.get_fields_and_field_types().items():
            val = getattr(msg, field)
            full_field_name = prefix + "." + field if prefix else field
            if type_.startswith("sequence<"):
                for i, aval in enumerate(val):
                    yield from _gen_msg_values(aval, f"{full_field_name}[{i}]")
            else:
                yield from _gen_msg_values(val, full_field_name)
    else:
        yield prefix, msg


def dump_bag(bag_path):
    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type
                for i in range(len(topic_types))}

    file_map = {}

    start_time = None
    msg_cnt = 0
    while reader.has_next():
        (topic, data, ts) = reader.read_next()
        if topic in ["/rosout", "/parameter_events"]:
            continue
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        if topic not in file_map:
            file = open("{}/{}.csv".format(
                bag_path,
                topic.lstrip("/") .replace("/", "_")),
                "w")
            fields = [field for field, val in _gen_msg_values(msg)
                      if not field.startswith("header.")]
            print("time," + ','.join(fields), file=file)
            file_map[topic] = file

        file = file_map[topic]
        if hasattr(msg, "header"):
            t = msg.header.stamp.sec + 1e-9*msg.header.stamp.nanosec
        else:
            t = ts
        if start_time is None:
            start_time = t
        print(','.join([str(t - start_time)] +
                       [str(val) for field, val in _gen_msg_values(msg)
                        if not field.startswith("header.")]),
              file=file)
        if msg_cnt % 1000 == 0:
            print("{:5.3f}".format(t - start_time))
        msg_cnt += 1


if len(sys.argv) == 2:
    dump_bag(sys.argv[1])
else:
    print("Usage: {} <bag directory>".format(sys.argv[0]), file=sys.stderr)
    sys.exit(1)