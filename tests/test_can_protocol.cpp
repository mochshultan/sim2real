#include <cassert>
#include <cmath>
#include "jaguar_control/robstride_hardware_manager.hpp"

int main()
{
  using namespace robstride;
  MotorParams params;
  auto frame = buildEnableMotorFrame(1);
  assert(frame.can_dlc == 8);
  assert((frame.can_id & CAN_EFF_MASK & 0xFFFFU) == 0xFE01U);
  frame.can_id = CAN_EFF_FLAG | (2U << 24) | (4U << 8) | 0xFE;
  frame.can_dlc = 8;
  frame.data[0] = frame.data[2] = frame.data[4] = 0x7F;
  frame.data[1] = frame.data[3] = frame.data[5] = 0xFF;
  assert(parseFeedbackFrame(frame, params).valid);
  auto bad = frame;
  bad.can_id = CAN_EFF_FLAG | (1U << 24) | (4U << 8) | 0xFE;
  assert(!parseFeedbackFrame(bad, params).valid);
  bad = frame;
  bad.can_id |= CAN_RTR_FLAG;
  assert(!parseFeedbackFrame(bad, params).valid);
  bad = frame;
  bad.can_id |= CAN_ERR_FLAG;
  assert(!parseFeedbackFrame(bad, params).valid);
  bad = frame;
  bad.can_dlc = 1;
  assert(!parseFeedbackFrame(bad, params).valid);
  bad = frame;
  bad.can_id |= 1U << 16;
  assert(parseFeedbackFrame(bad, params).error);

  // The manager is never initialized: these tests cannot open a CAN socket.
  RobStrideHardwareManager manager;
  assert(!manager.getJointState(0).feedback_valid);
  assert(std::isnan(manager.getJointState(0).position));
  const auto now = std::chrono::steady_clock::now();
  manager.processFeedbackFrame(frame, "can1", now);
  assert(manager.getJointState(0).feedback_valid);
  manager.processFeedbackFrame(frame, "can1", now - std::chrono::seconds(1));
  assert(!manager.getJointState(0).feedback_valid);
  assert(!manager.setJointCommands(std::vector<JointCommand>(1)));
  std::vector<JointCommand> commands(N_JOINTS);
  commands[0].position = NAN;
  assert(!manager.setJointCommands(commands));
  assert(manager.isEmergencyStopped());

  RobStrideHardwareManager raw_manager;
  frame.data[0] = 0xFF;
  frame.data[1] = 0xFF;
  raw_manager.processFeedbackFrame(frame, "can1", now);
  assert(raw_manager.getJointState(0).position > 12.0);
  return 0;
}
