#!/usr/bin/env python3

import os
import sys
import csv

import rosbag2_py

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp_to_ms(stamp):
    return (
        stamp.sec * 1000.0
        + stamp.nanosec / 1e6
    )


def main():

    if len(sys.argv) != 2:
        print(
            "Usage: rosbag_to_csv.py <rosbag_directory>"
        )
        return

    bag_path = sys.argv[1]

    output_dir = "/ADC_27/logs"

    os.makedirs(
        output_dir,
        exist_ok=True
    )


    # ============================================================
    # OPEN ROS BAG
    # ============================================================

    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id="sqlite3"
    )

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr"
    )

    reader.open(
        storage_options,
        converter_options
    )


    # ============================================================
    # GET TOPIC TYPES
    # ============================================================

    topic_types = {
        topic.name: topic.type
        for topic in reader.get_all_topics_and_types()
    }


    # ============================================================
    # OPEN CSV FILES
    # ============================================================

    angular_file = open(
        os.path.join(
            output_dir,
            "truth_angular_velocity.csv"
        ),
        "w",
        newline=""
    )

    base_file = open(
        os.path.join(
            output_dir,
            "truth_gps_base_front.csv"
        ),
        "w",
        newline=""
    )

    rover_file = open(
        os.path.join(
            output_dir,
            "truth_gps_rover_front.csv"
        ),
        "w",
        newline=""
    )


    angular_writer = csv.writer(
        angular_file
    )

    base_writer = csv.writer(
        base_file
    )

    rover_writer = csv.writer(
        rover_file
    )


    # ============================================================
    # CSV HEADERS
    # ============================================================

    angular_writer.writerow([
        "timestamp_ms",
        "omega_x",
        "omega_y",
        "omega_z"
    ])

    base_writer.writerow([
        "timestamp_ms",
        "latitude",
        "longitude",
        "altitude"
    ])

    rover_writer.writerow([
        "timestamp_ms",
        "latitude",
        "longitude",
        "altitude"
    ])


    # ============================================================
    # READ BAG
    # ============================================================

    while reader.has_next():

        topic_name, data, bag_timestamp = (
            reader.read_next()
        )

        if topic_name not in topic_types:
            continue

        message_type = get_message(
            topic_types[topic_name]
        )

        msg = deserialize_message(
            data,
            message_type
        )


        # ========================================================
        # TRUE ANGULAR VELOCITY
        # ========================================================

        if topic_name == "/ground_truth/angular_velocity":

            timestamp_ms = stamp_to_ms(
                msg.header.stamp
            )

            angular_writer.writerow([
                timestamp_ms,
                msg.vector.x,
                msg.vector.y,
                msg.vector.z
            ])


        # ========================================================
        # TRUE BASE GNSS
        # ========================================================

        elif topic_name == "/ground_truth/gps_base_front":

            timestamp_ms = stamp_to_ms(
                msg.header.stamp
            )

            base_writer.writerow([
                timestamp_ms,
                msg.latitude,
                msg.longitude,
                msg.altitude
            ])


        # ========================================================
        # TRUE ROVER GNSS
        # ========================================================

        elif topic_name == "/ground_truth/gps_rover_front":

            timestamp_ms = stamp_to_ms(
                msg.header.stamp
            )

            rover_writer.writerow([
                timestamp_ms,
                msg.latitude,
                msg.longitude,
                msg.altitude
            ])


    # ============================================================
    # CLOSE FILES
    # ============================================================

    angular_file.close()
    base_file.close()
    rover_file.close()

    print("ROS bag converted successfully.")

    print(
        "/ADC_27/logs/truth_angular_velocity.csv"
    )

    print(
        "/ADC_27/logs/truth_gps_base_front.csv"
    )

    print(
        "/ADC_27/logs/truth_gps_rover_front.csv"
    )


if __name__ == "__main__":
    main()
