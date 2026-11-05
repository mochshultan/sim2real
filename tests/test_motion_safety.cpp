#include <cassert>
#include "jaguar_control/robstride_hardware_manager.hpp"

using namespace robstride;
using Clock = std::chrono::steady_clock;

void feedback(RobStrideHardwareManager & manager, Clock::time_point now,
              double knee = 0.0, double roll = 0.0)
{
  const auto & configs = manager.getJointConfigs();
  for (size_t i = 0; i < N_JOINTS; ++i) {
    const auto & cfg = configs[i];
    const double position = i == 11 ? knee : (i == 0 ? roll : 0.0);
    can_frame frame{};
    frame.can_id = CAN_EFF_FLAG | (2U << 24) | (cfg.can_id << 8) | 0xFE;
    frame.can_dlc = 8;
    const auto raw = linearMapping((position + cfg.angle_offset) * cfg.direction,
                                  cfg.motor_params.p_min, cfg.motor_params.p_max);
    frame.data[0] = raw >> 8;
    frame.data[1] = raw & 255;
    manager.processFeedbackFrame(frame, cfg.bus_name, now);
  }
}

void command(RobStrideHardwareManager & manager, double knee)
{
  std::vector<JointCommand> commands(N_JOINTS);
  for (auto & cmd : commands) cmd.kp = 14.0;
  commands[11].position = knee;
  assert(manager.setJointCommands(commands));
}

int main()
{
  std::ostringstream captured;
  auto * previous_log = std::cerr.rdbuf(captured.rdbuf());
  const auto now = Clock::now();
  const auto ms = [](int n) { return std::chrono::milliseconds(n); };
  RobStrideHardwareManager limits;
  for (const auto & cfg : limits.getJointConfigs()) {
    assert(cfg.max_effort == 14.0);
  }
  assert(limits.setControlTorqueLimit(14.0));
  assert(!limits.setControlTorqueLimit(14.01));
  assert(!limits.setControlTorqueLimit(0.0));
  assert(!limits.setControlTorqueLimit(std::numeric_limits<double>::quiet_NaN()));
  assert(limits.setControlTorqueLimit(3.0));  // Suspended PACE tests keep their lower cap.
  for (const auto & cfg : limits.getJointConfigs()) assert(cfg.max_effort == 3.0);
  // The reported 1.581 rad tracking error is below the unified 2.0 threshold.
  RobStrideHardwareManager normal;
  command(normal, 2.5);
  feedback(normal, now, 0.919);
  normal.checkMotionSafety(now);
  feedback(normal, now + ms(150), 0.919);
  normal.checkMotionSafety(now + ms(150));
  assert(!normal.safeParkRequested() && !normal.isEmergencyStopped());

  // A short excursion clears the timer; a sustained one requests park.
  RobStrideHardwareManager tracking;
  command(tracking, 2.2);
  feedback(tracking, now);
  tracking.checkMotionSafety(now);
  command(tracking, 0.0);
  tracking.checkMotionSafety(now + ms(50));
  command(tracking, 2.2);
  tracking.checkMotionSafety(now + ms(90));
  tracking.checkMotionSafety(now + ms(150));
  assert(!tracking.safeParkRequested());
  tracking.checkMotionSafety(now + ms(200));
  assert(tracking.safeParkRequested() && !tracking.isEmergencyStopped());
  tracking.setSafeParkActive(true);
  tracking.checkMotionSafety(now + ms(201));
  feedback(tracking, now + ms(310));
  tracking.checkMotionSafety(now + ms(310));
  assert(tracking.isEmergencyStopped());

  // The logged roll overshoot can recover during park without retripping.
  RobStrideHardwareManager position;
  command(position, 0.0);
  feedback(position, now, 0.0, -0.794);
  position.checkMotionSafety(now);
  position.checkMotionSafety(now + ms(110));
  assert(position.safeParkRequested());
  position.setSafeParkActive(true);
  feedback(position, now + ms(220), 0.0, -0.794);
  position.checkMotionSafety(now + ms(220));
  assert(!position.isEmergencyStopped());
  feedback(position, now + ms(330), 0.0, -0.86);
  position.checkMotionSafety(now + ms(330));
  feedback(position, now + ms(440), 0.0, -0.86);
  position.checkMotionSafety(now + ms(440));
  assert(position.isEmergencyStopped());

  RobStrideHardwareManager unacknowledged;
  command(unacknowledged, 2.2);
  feedback(unacknowledged, now);
  unacknowledged.checkMotionSafety(now);
  unacknowledged.checkMotionSafety(now + ms(110));
  feedback(unacknowledged, now + ms(370));
  unacknowledged.checkMotionSafety(now + ms(370));
  assert(unacknowledged.isEmergencyStopped());

  RobStrideHardwareManager stale;
  command(stale, 0.0);
  feedback(stale, now);
  stale.checkMotionSafety(now + ms(251));
  assert(stale.isEmergencyStopped());

  RobStrideHardwareManager missing;
  command(missing, 0.0);
  missing.checkMotionSafety(now);
  assert(missing.isEmergencyStopped());

  RobStrideHardwareManager multiple;
  std::vector<JointCommand> targets(N_JOINTS);
  for (auto & target : targets) target.kp = 14.0;
  targets[2].position = targets[11].position = 2.2;
  assert(multiple.setJointCommands(targets));
  feedback(multiple, now);
  multiple.checkMotionSafety(now);
  feedback(multiple, now + ms(110));
  multiple.checkMotionSafety(now + ms(110));
  assert(multiple.safeParkRequested());

  RobStrideHardwareManager distinct;
  distinct.triggerEmergencyStop("COMMAND_TIMEOUT", "age_s=0.110 limit_s=0.100");
  distinct.triggerEmergencyStop("COMMAND_TIMEOUT", "age_s=0.120 limit_s=0.100");
  distinct.triggerEmergencyStop("CAN_WRITE", "bus=can0 can_id=1", "FR_collar_joint");
  distinct.triggerEmergencyStop("MOTOR_FAULT", "fault=0x1 warning=0x0", "FR_collar_joint");
  distinct.triggerEmergencyStop("MOTOR_FAULT", "fault=0x2 warning=0x0", "FR_collar_joint");
  std::cerr.rdbuf(previous_log);

  const auto logs = captured.str();
  assert(logs.find("event=SAFE_PARK code=TRACKING joint=FR_knee_joint actual_rad=") != std::string::npos);
  assert(logs.find("event=SAFE_PARK code=TRACKING joint=BL_knee_joint actual_rad=") != std::string::npos);
  assert(logs.find("error_rad=") != std::string::npos && logs.find("debounce_ms=100") != std::string::npos);
  assert(logs.find("event=SAFE_PARK code=POSITION_LIMIT joint=BL_collar_joint") != std::string::npos);
  assert(logs.find("event=ESTOP code=POSITION_RANGE joint=BL_collar_joint") != std::string::npos);
  assert(logs.find("event=ESTOP code=FEEDBACK_TIMEOUT joint=BL_collar_joint") != std::string::npos);
  assert(logs.find("event=ESTOP code=FEEDBACK_INVALID joint=BL_collar_joint") != std::string::npos);
  assert(logs.find("event=ESTOP code=PARK_ACK_TIMEOUT elapsed_s=") != std::string::npos);
  const auto command_fault = logs.find("event=ESTOP code=COMMAND_TIMEOUT");
  assert(command_fault != std::string::npos);
  assert(logs.find("event=ESTOP code=COMMAND_TIMEOUT", command_fault + 1) == std::string::npos);
  assert(logs.find("event=ESTOP code=CAN_WRITE joint=FR_collar_joint") != std::string::npos);
  assert(logs.find("fault=0x1") != std::string::npos && logs.find("fault=0x2") != std::string::npos);
}
