#!/bin/bash

BAG_DIR="/ADC_27/logs/ekf_ground_truth"
PID_FILE="/ADC_27/logs/ekf_rosbag.pid"
LOG_FILE="/ADC_27/logs/ekf_rosbag_terminal.log"

source /opt/ros/humble/setup.bash
source /home/base/beamng-ros2-ws/install/setup.bash

case "$1" in

    start)

        mkdir -p /ADC_27/logs

        rm -rf "$BAG_DIR"

        nohup ros2 bag record \
            -o "$BAG_DIR" \
            /ground_truth/angular_velocity \
            /ground_truth/gps_base_front \
            /ground_truth/gps_rover_front \
            > "$LOG_FILE" 2>&1 &

        echo $! > "$PID_FILE"

        echo "ROS bag recording started."
        ;;


    stop)

        if [ -f "$PID_FILE" ]; then

            PID=$(cat "$PID_FILE")

            if kill -0 "$PID" 2>/dev/null; then
                kill -INT "$PID"
                wait "$PID" 2>/dev/null
            fi

            rm -f "$PID_FILE"

            echo "ROS bag recording stopped."
        fi
        ;;


    *)

        echo "Usage: $0 {start|stop}"
        exit 1
        ;;

esac
