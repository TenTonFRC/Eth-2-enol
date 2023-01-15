// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>

//void shootCone(){

//}

/**
 * This sample program shows how to control a motor using a joystick. In the
 * operator control part of the program, the joystick is read and the value is
 * written to the motor.
 *
 * Joystick analog values range from -1 to 1 and speed controller inputs as
 * range from -1 to 1 making it easy to work together.
 */
class Robot : public frc::TimedRobot {
 public:
  void TeleopPeriodic() override {
    if (m_control.GetAButtonPressed()){
      m_motor.Set(ControlMode::PercentOutput, -0.15);
    }
    if (m_control.GetAButtonReleased()){
      m_motor.Set(ControlMode::PercentOutput, 0);
    }
    if (m_control.GetXButtonPressed()){
      m_motor.Set(ControlMode::PercentOutput, 0.8);
    }
    if (m_control.GetXButtonReleased()){
      m_motor.Set(ControlMode::PercentOutput, 0);
    }
   }

 private:
  frc::XboxController m_control{0};
  TalonFX m_motor{1};
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
