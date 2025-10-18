// #include "frc/TimedRobot.h"
// #include "frc/Joystick.h"
// #include "frc/motorcontrol/PWMSparkMax.h"
// #include "frc/drive/MecanumDrive.h"
// #include "frc/ADXRS450_Gyro.h"
// #include "Robot.h"
// #include "RobotContainer.h"
// #include <frc2/command/CommandScheduler.h>
// #include <units/angle.h>
// #include <optional>

/*class Robot : public frc::TimedRobot {
 public:
  Robot() : 
    frontLeft{1}, frontRight{2}, 
    backLeft{3}, backRight{4},
    drive{frontLeft, backLeft, frontRight, backRight},
    gyro{}
  {}

  void RobotInit() override {
    gyro.Calibrate();
  }

  void RobotPeriodic() override {
    frc2::CommandScheduler::GetInstance().Run();
  }

  void DisabledInit() override {}
  void DisabledPeriodic() override {}
  void DisabledExit() override {}

  void AutonomousInit() override {
    auto command = m_container.GetAutonomousCommand();
    if (command) {
        m_autonomousCommand = std::move(command);
        m_autonomousCommand->Schedule();
    }
  }

  void AutonomousPeriodic() override {}
  void AutonomousExit() override {}

  void TeleopInit() override {
    if (m_autonomousCommand.has_value()) {
        m_autonomousCommand->Cancel();
        m_autonomousCommand.reset();
    }
  }

  void TeleopPeriodic() override {
    double y = -joystick.GetY();
    double x = joystick.GetX();
    double z = joystick.GetZ();

    drive.DriveCartesian(y, x, z, frc::Rotation2d(units::degree_t(gyro.GetAngle())));
  }

  void TeleopExit() override {}

  void TestInit() override {
    frc2::CommandScheduler::GetInstance().CancelAll();
  }

  void TestPeriodic() override {}
  void TestExit() override {}

 private:
  frc::Joystick joystick{0};
  frc::PWMSparkMax frontLeft{1}, frontRight{2}, backLeft{3}, backRight{4};
  frc::MecanumDrive drive{frontLeft, backLeft, frontRight, backRight};
  frc::ADXRS450_Gyro gyro;
  std::optional<frc2::CommandPtr> m_autonomousCommand;
  RobotContainer m_container;
};

*/
#ifndef RUNNING_FRC_TESTS
int main() {
  return 1;
  // return frc::StartRobot<Robot>();
}
#endif
