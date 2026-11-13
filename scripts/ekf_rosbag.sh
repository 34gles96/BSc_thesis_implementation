
#!/usr/bin/env bash

set -eo pipefail

LOG_DIR="/ADC_27/logs"
BAG_DIR="$LOG_DIR/ekf_ground_truth"

PID_FILE="$LOG_DIR/ekf_rosbag.pid"
LOG_FILE="$LOG_DIR/ekf_rosbag_terminal.log"

source /opt/ros/humble/setup.bash
source /home/base/beamng-ros2-ws/install/setup.bash

mkdir -p "$LOG_DIR"


case "${1:-}" in

    start)

        # Prevent deletion while a recorder is running
        if [ -f "$PID_FILE" ]; then
            PID=$(cat "$PID_FILE")

            if kill -0 "$PID" 2>/dev/null; then
                echo "A recording is already running."
                echo "Stop it before starting a new experiment."
                exit 1
            fi

            rm -f "$PID_FILE"
        fi

        echo "Deleting previous ground-truth measurements..."

        # Delete previous fixed-directory ROS bag
        rm -rf -- "$BAG_DIR"

        # Delete old timestamped ROS bag directories
        # created by the earlier version of this script
        find "$LOG_DIR" \
            -mindepth 1 \
            -maxdepth 1 \
            -type d \
            -name "ekf_ground_truth_*" \
            -exec rm -rf -- {} +

        # Delete old ground-truth CSV files
        rm -f \
            "$LOG_DIR/truth_angular_velocity.csv" \
            "$LOG_DIR/truth_gps_base_front.csv" \
            "$LOG_DIR/truth_gps_rover_front.csv"

        echo "Previous measurements deleted."

        # Record ONLY noise-free ground truth
        nohup ros2 bag record \
            -o "$BAG_DIR" \
            /ground_truth/angular_velocity \
            /ground_truth/gps_base_front \
            /ground_truth/gps_rover_front \
            > "$LOG_FILE" 2>&1 &

        echo $! > "$PID_FILE"

        echo "Ground-truth recording started."
        echo "PID: $(cat "$PID_FILE")"
        echo "Directory: $BAG_DIR"
        ;;

    
    stop)

        if [ ! -f "$PID_FILE" ]; then
            echo "No recording PID file found."
            exit 1
        fi

        PID=$(cat "$PID_FILE")

        if kill -0 "$PID" 2>/dev/null; then

            echo "Stopping ROS bag recording..."

            kill -INT "$PID"

            # Wait for rosbag2 to finalize its files
            for ((i=0; i<150; i++)); do

                if ! kill -0 "$PID" 2>/dev/null; then
                    break
                fi

                STATUS=$(ps -o stat= -p "$PID" 2>/dev/null || true)
                STATUS="${STATUS//[[:space:]]/}"

                if [[ "$STATUS" == Z* ]]; then
                    break
                fi

                sleep 0.1
            done

            STATUS=$(ps -o stat= -p "$PID" 2>/dev/null || true)
            STATUS="${STATUS//[[:space:]]/}"

            if kill -0 "$PID" 2>/dev/null &&
               [[ "$STATUS" != Z* ]]; then
                echo "Recorder is still running or finalizing."
                exit 1
            fi
        fi

        rm -f "$PID_FILE"

        # Verify that ROS bag exists
        if [ ! -f "$BAG_DIR/metadata.yaml" ]; then

            echo "ERROR: ROS bag was not created correctly."
            echo "Check: $LOG_FILE"
            exit 1

        fi

        echo "ROS bag recording stopped."

        # Check whether the bag can be read
        ros2 bag info "$BAG_DIR"

        # Convert the recorded data into CSV
        echo "Generating CSV reports..."

        python3 /ADC_27/scripts/rosbag_to_csv.py "$BAG_DIR"

        echo "CSV reports successfully generated."

        ;;


    *)
        echo "Usage: $0 {start|stop}"
        exit 1
        ;;

esac
