#!/bin/bash

# ============================================================
# BeamNG steering calibration experiment
#
# Output:
#   steering_input.txt  -> normalized BeamNG steering input
#   steering_angle.txt  -> physical front-wheel angle [rad]
# ============================================================

INPUT_FILE="steering_input.txt"
ANGLE_FILE="steering_angle.txt"

# Steering commands to test
STEERING_VALUES=(-1.0 -0.75 -0.5 -0.25 0.0 0.25 0.5 0.75 1.0)

# Time to wait after commanding each steering position [seconds]
SETTLE_TIME=3

# Start with new, empty files
> "$INPUT_FILE"
> "$ANGLE_FILE"

echo "Starting steering calibration..."
echo

for u in "${STEERING_VALUES[@]}"
do
    echo "========================================"
    echo "Commanding steering input: $u"

    # Continuously send the control command at 10 Hz.
    # stdout/stderr are hidden so the terminal stays readable.
    ros2 topic pub -r 10 \
        /control \
        beamng_msgs/msg/VehicleControl \
        "{steering: $u, throttle: 0.0, brake: 0.0, parkingbrake: 0.0, clutch: 0.0, gear: 0}" \
        > /dev/null 2>&1 &

    PUB_PID=$!

    echo "Waiting ${SETTLE_TIME} seconds for steering to settle..."
    sleep "$SETTLE_TIME"

    # Read the actual normalized steering input reported by BeamNG.
    actual_input=$(
        ros2 topic echo \
            /vehicles/beamng_bridge/sensors/electrics \
            --field steering_input \
            --once |
        sed -n '1p'
    )

    # Read the physical steering angle calculated from fh3/fh4.
    physical_angle=$(
        ros2 topic echo \
            /vehicle/steering_angle \
            --field data \
            --once |
        sed -n '1p'
    )

    # Save numeric values.
    echo "$actual_input" >> "$INPUT_FILE"
    echo "$physical_angle" >> "$ANGLE_FILE"

    echo "Actual input:   $actual_input"
    echo "Physical angle: $physical_angle rad"

    # Stop continuously publishing this steering command.
    kill "$PUB_PID" 2>/dev/null
    wait "$PUB_PID" 2>/dev/null

    echo
done

echo "========================================"
echo "Calibration finished."
echo
echo "Results:"
echo
echo "steering_input    steering_angle [rad]"
paste "$INPUT_FILE" "$ANGLE_FILE"