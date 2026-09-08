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
  const auto now = Clock::now();
  const auto ms = [](int n) { return std::chrono::milliseconds(n); };
  // The reported 0.854 rad tracking error is below the unified 1.57 threshold.
  RobStrideHardwareManager normal;
  command(normal, 1.93256);
  feedback(normal, now, 1.07865);
  normal.checkMotionSafety(now);
  feedback(normal, now + ms(150), 1.07865);
  normal.checkMotionSafety(now + ms(150));
  assert(!normal.safeParkRequested() && !normal.isEmergencyStopped());

  // A short excursion clears the timer; a sustained one requests park.
  RobStrideHardwareManager tracking;
  command(tracking, 2.0);
  feedback(tracking, now);
  tracking.checkMotionSafety(now);
  command(tracking, 0.0);
  tracking.checkMotionSafety(now + ms(50));
  command(tracking, 2.0);
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
  command(unacknowledged, 2.0);
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
}
