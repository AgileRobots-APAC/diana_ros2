#!/bin/bash

SUPPORTED_ROBOT_TYPES=(diana3 diana7 diana7g2)

if [ $# -ne 2 ]; then
    echo "Usage: $0 <robot_type> <robot_ip>"
    echo "  robot_type: ${SUPPORTED_ROBOT_TYPES[*]}"
    exit 1
fi

ROBOT_TYPE=$1
ROBOT_IP=$2

if [[ " ${SUPPORTED_ROBOT_TYPES[*]} " != *" ${ROBOT_TYPE} "* ]]; then
    echo "Error: Unsupported robot_type '${ROBOT_TYPE}'. Use one of: ${SUPPORTED_ROBOT_TYPES[*]}"
    exit 1
fi

RATE_LIMITING_HEADER=install/libfranka/include/franka/rate_limiting.h
BRINGUP_BUILD_DIR=build/diana_bringup

if [ ! -f "$RATE_LIMITING_HEADER" ] || [ ! -d "$BRINGUP_BUILD_DIR" ]; then
    echo "Error: Workspace is not built. Run this script from the workspace root after colcon build."
    exit 1
fi

echo "[1/4] Checking the Diana arm libfranka is built for..."
JERK=$(sed -n 's/^constexpr double kMaxTranslationalJerk = \([0-9.]*\).*/\1/p' "$RATE_LIMITING_HEADER")
case "$JERK" in
    2500) BUILT_ROBOT_TYPE=diana3 ;;
    3000.0) BUILT_ROBOT_TYPE=diana7 ;;
    5000.0) BUILT_ROBOT_TYPE=diana7g2 ;;
    *) BUILT_ROBOT_TYPE=unknown ;;
esac
if [ "$BUILT_ROBOT_TYPE" != "$ROBOT_TYPE" ]; then
    echo "Error: libfranka is built for '${BUILT_ROBOT_TYPE}', not '${ROBOT_TYPE}'."
    echo "       Run src/scripts/select_diana_arm.sh ${ROBOT_TYPE} and rebuild libfranka and its dependents."
    exit 1
fi
echo "      -> libfranka is built for ${ROBOT_TYPE}."

echo "[2/4] Checking connectivity to ${ROBOT_IP}..."
if ! ping -c 3 -W 1 "$ROBOT_IP" > /dev/null; then
    echo "Error: Robot at ${ROBOT_IP} is not reachable."
    exit 1
fi
echo "      -> Robot reachable."

echo "[3/4] Configuring diana_bringup hardware tests for ${ROBOT_TYPE} at ${ROBOT_IP}..."
if ! cmake "$BRINGUP_BUILD_DIR" -DROBOT_TYPE="$ROBOT_TYPE" -DROBOT_IP="$ROBOT_IP" > /dev/null; then
    echo "Error: Failed to configure diana_bringup."
    exit 1
fi

echo "[4/4] Running diana_ros2 hardware tests..."
rm -f reports/*.xml
colcon test \
    --base-paths src \
    --packages-select diana_bringup \
    --event-handlers console_direct+ \
    --ctest-args --tests-regex test_hardware
colcon test-result --verbose --test-result-base "$BRINGUP_BUILD_DIR"
TEST_EXIT_CODE=$?

if [ $TEST_EXIT_CODE -ne 0 ]; then
    exit 1
fi
