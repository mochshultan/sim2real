#!/usr/bin/env bash
# Request driver stop and allow owning processes to execute their cleanup.
set -u

echo "Requesting emergency stop. Support the robot; torque cutoff can cause a fall."
if command -v ros2 >/dev/null 2>&1; then
    timeout 3s ros2 topic pub --once /jaguar/emergency_stop std_msgs/msg/Bool '{data: true}' ||
        echo "Warning: ROS stop delivery was not confirmed."
fi

targets=(robstride_can_node can_hardware_node.py nxp_jaguar_controller.py
         test_sit_stand.py check_joints.py calibrate_sit_zero.py
         calibrate_stand_pose.py robstride_motor_test.py keyboard_teleop.py)
pids=()
for target in "${targets[@]}"; do
    while read -r pid; do
        [[ "$pid" == "$$" || "$pid" == "$PPID" ]] && continue
        pids+=("$pid")
        kill -INT "$pid" 2>/dev/null || true
    done < <(pgrep -u "$(id -u)" -f "(^|/|[[:space:]])${target//./\\.}([[:space:]]|$)" || true)
done

deadline=$((SECONDS + 15))
remaining=()
while (( SECONDS < deadline )); do
    remaining=()
    for pid in "${pids[@]}"; do
        kill -0 "$pid" 2>/dev/null && remaining+=("$pid")
    done
    (("${#remaining[@]}" == 0)) && break
    sleep 0.1
done
if (("${#remaining[@]}")); then
    echo "Processes still running: ${remaining[*]}. Use the physical emergency stop."
    exit 1
fi
echo "Target processes exited. Physical actuator stop is NOT confirmed."
echo "CAN interfaces left up so stop commands remain possible."
