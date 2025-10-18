#include <frc/TimedRobot.h>
#include <frc/Joystick.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc2/command/CommandScheduler.h>

class Robot : public frc::TimedRobot {
public:
    Robot() : 
        frontLeft{1}, frontRight{2}, 
        backLeft{3}, backRight{4}, 
        mecanumDrive{frontLeft, backLeft, frontRight, backRight} {}

    void TeleopPeriodic() override {
        // Get joystick inputs
        double ySpeed = -joystick.GetY();
        double xSpeed = joystick.GetX();
        double zRotation = joystick.GetZ();
        
        // Drive the robot using mecanum drive
        mecanumDrive.DriveCartesian(ySpeed, xSpeed, zRotation);
    }

private:
    frc::Joystick joystick{0};
    frc::PWMSparkMax frontLeft, frontRight, backLeft, backRight;
    frc::MecanumDrive mecanumDrive;
};

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<Robot>();
}
#endif
