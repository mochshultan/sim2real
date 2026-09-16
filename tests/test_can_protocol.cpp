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
  assert(buildStopMotorFrame(1).data[0] == 0);
  assert(buildStopMotorFrame(1, 0xFE, true).data[0] == 1);
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

  RobStrideHardwareManager fault_manager;
  fault_manager.processFeedbackFrame(bad, "can1", std::chrono::steady_clock::now());
  assert(fault_manager.isEmergencyStopped());
  assert(!fault_manager.requestFaultReset());  // No CAN hardware initialized in this test.

  RobStrideHardwareManager jump_manager;
  const auto jump_time = std::chrono::steady_clock::now();
  jump_manager.processFeedbackFrame(frame, "can1", jump_time);
  auto jumped = frame;
  jumped.data[0] = 0xA0;
  jumped.data[1] = 0x00;
  jump_manager.processFeedbackFrame(jumped, "can1", jump_time + std::chrono::milliseconds(5));
  assert(jump_manager.isEmergencyStopped());

  RobStrideHardwareManager fault_frame_manager;
  auto fault_frame = frame;
  fault_frame.can_id = CAN_EFF_FLAG | (21U << 24) | (4U << 8) | 0xFE;
  fault_frame.data[0] = 0x04;
  fault_frame_manager.processFeedbackFrame(fault_frame, "can1", jump_time);
  assert(fault_frame_manager.isEmergencyStopped());

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
