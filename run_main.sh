#!/bin/bash
echo "Rebooted at $(date)" >> ~/reboot_log.txt

# Path to the device
DEVICE="/dev/ttyUSB0"

# Timeout in seconds
TIMEOUT=600
START_TIME=$(date +%s)

# Wait for the device to appear
while [ ! -e "$DEVICE" ]; do
    CURRENT_TIME=$(date +%s)
    ELAPSED_TIME=$((CURRENT_TIME - START_TIME))

    if [ "$ELAPSED_TIME" -ge "$TIMEOUT" ]; then
        echo "Timeout waiting for $DEVICE to appear."
        exit 1
    fi

    echo "Waiting for $DEVICE to appear..."
    sleep 5
done

echo "$DEVICE detected."

cd /home/jetson/Robotics
echo "Successfully moved at $(date)" >> ~/reboot_log.txt
/usr/bin/python3 cam_test.py

