#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/TimedRobot.h>
#include "frc/MathUtil.h"
#include <frc/drive/MecanumDrive.h>
#include <rev/SparkMax.h>
#include "iostream"
#include "cmath"
#include "frc/SerialPort.h"
#include "frc/SPI.h"
#include "studica/AHRS.h"  //navx library
#include "frc/geometry/Rotation2d.h"

#define ENABLE_FIELD_CENTRIC 1

class Robot : public frc::TimedRobot {
//  studica::AHRS navx{studica::AHRS::NavXComType::kUSB1};
 public:
  Robot() : 
      #if ENABLE_FIELD_CENTRIC
      navx(studica::AHRS::NavXComType::kUSB1,studica::AHRS::NavXUpdateRate::k50Hz),
      #endif
      m_ele_left(1, rev::spark::SparkMax::MotorType::kBrushed),
      m_ele_right(2,rev::spark::SparkMax::MotorType::kBrushed),
      m_frontLeft(60, rev::spark::SparkMax::MotorType::kBrushless),
      m_rearLeft(15, rev::spark::SparkMax::MotorType::kBrushless),
      m_frontRight(30, rev::spark::SparkMax::MotorType::kBrushless),
      m_rearRight(45, rev::spark::SparkMax::MotorType::kBrushless),
      m_robotDrive(m_frontLeft, m_rearLeft, m_frontRight, m_rearRight)
       {

    // Invert the left side motors if necessary
    m_frontLeft.SetInverted(true);
    m_rearLeft.SetInverted(true);
    m_ele_right.SetInverted(true); //invert one of the elevator motors
    
    // navx = studica::AHRS(studica::AHRS::NavXComType::kUSB1,studica::AHRS::NavXUpdateRate::k50Hz);
    



    // If needed, you can enable follower mode for left motors to follow the right ones
    // m_rearLeft.Follow(m_frontLeft);
    // m_rearRight.Follow(m_frontRight);
  }

  void RobotInit() override {

  }

  void Autonomous() {
    double forward = .3;
    double right=0.0;
    double rot = 0.0;
    units::second_t time{2};
    m_robotDrive.DriveCartesian(forward,right,rot);
    frc::Wait(time);
    m_robotDrive.DriveCartesian(0.0,0.0,0.0);
  }

  void AutonomousInit() override {
    //m_robotDrive.DriveCartesian(.3, 0, 0);
  }

  void AutonomousPeriodic() override {
    m_robotDrive.DriveCartesian(.1, 0, 0);
  }

  double SquareJoy(double stick) {
    double sqr_stick = stick * stick;
    if (stick < 0) {
      sqr_stick *= -1;
    }
    return sqr_stick;
  }


  void TeleopPeriodic() override {
    // Drive using joystick inputs
    
    // this->m_frontLeft

    // std::cout << this->m_frontLeft.GetBusVoltage() << std::endl;

    // deadband joystick controlls
    double deadband = 0.01;
    double rotDeadband = .2; //rotation is really jumpy around 0 for our joysticks
    
    double forward_raw = driver_stick.GetLeftY();
    
    double forward = frc::ApplyDeadband(SquareJoy(forward_raw),deadband);//-(m_stick.GetY());
    forward *= 0.25;
    double ele_up_power = -.1; //might need to tune these
    double ele_down_power = .3;
    
    units::second_t toggle_delay{0.2};
    #if ENABLE_FIELD_CENTRIC
    // double heading = ((navx.GetYaw()+180)%360)*360+360;
    // double heading = navx.GetRotation2d().Degrees();
    // std::cout << heading;
    // frc::Rotation2d gyro_heading{units::degree_t{heading}};
    frc::Rotation2d gyro_heading = navx.GetRotation2d();
    // std::cout << forward << std::endl;
    // std::cout << std::abs(forward) << std::endl;
    if (driver_stick.GetStartButtonPressed()){
      navx.ZeroYaw();
    }
    #endif
    if(driver_stick.GetBackButtonPressed()){
      if (dual_stick_flag ==0){
        dual_stick_flag = 1;
      } else{
        dual_stick_flag = 0;
      }
      frc::Wait(toggle_delay);
    }
    if (dual_stick_flag==0){
      if(driver_stick.GetRightTriggerAxis() > 0.5){ //Thumb Button - ELEVATOR DOWN
        m_ele_left.Set(ele_down_power);
        m_ele_right.Set(ele_down_power);
      } else if(driver_stick.GetLeftTriggerAxis() > 0.5){// trigger UP - ELEVATOR UP
        m_ele_left.Set(ele_up_power);
        m_ele_right.Set(ele_up_power);
      } else{
        m_ele_left.Set(0.08);
        m_ele_right.Set(0.08);
      }
    } else{
      if(operator_stick.GetRawButton(1)){ //Thumb Button - ELEVATOR DOWN
        m_ele_left.Set(ele_down_power) ;
        m_ele_right.Set(ele_down_power);
      } else if(operator_stick.GetRawButton(2)){ //Trigger UP - ELEVATOR UP
        m_ele_left.Set(ele_up_power);
        m_ele_right.Set(ele_up_power);
      } else{
        m_ele_left.Set(0.08);
        m_ele_right.Set(0.08);
      }
    }
    // if (std::abs(forward) < deadband) {
    //   forward = 0;
    // }
    // double right_raw = driver_stick.GetX();
    double right_raw = driver_stick.GetLeftX();

    double right = frc::ApplyDeadband(SquareJoy(right_raw),deadband);//-(m_stick.GetX()-deadband)/(1-deadband);
    right *= 0.25;
    // if (std::abs(right) < deadband) {
    //   right = 0;
    // }
    double rot_raw = driver_stick.GetRightX();
  

    double rotation =  -frc::ApplyDeadband(SquareJoy(rot_raw),rotDeadband);   //-((m_stick.GetZ()-rotDeadband)/(1-rotDeadband));
    rotation *= 0.25;
    // if (std::abs(rotation) < rotDeadband) {
    //   rotation = 0;
    // }
    

    #if ENABLE_FIELD_CENTRIC
    m_robotDrive.DriveCartesian(forward, -
    right, rotation, -gyro_heading);
    #else
    m_robotDrive.DriveCartesian(forward, right, rotation);
    #endif
  }

 private:
  // frc::Joystick driver_stick{0};
  // frc::Joystick operator_stick{1};
  frc::XboxController driver_stick{0};
  frc::XboxController operator_stick{1};
  bool dual_stick_flag = 1;
  #if ENABLE_FIELD_CENTRIC
  studica::AHRS navx;
  #endif
  // Use CAN Spark MAX controllers
  rev::spark::SparkMax m_ele_left{1, rev::spark::SparkMax::MotorType::kBrushed}; 
  rev::spark::SparkMax m_ele_right{2, rev::spark::SparkMax::MotorType::kBrushed};
  rev::spark::SparkMax m_frontLeft{60, rev::spark::SparkMax::MotorType::kBrushed}; 
  rev::spark::SparkMax m_rearLeft{15, rev::spark::SparkMax::MotorType::kBrushed};
  rev::spark::SparkMax m_frontRight{30, rev::spark::SparkMax::MotorType::kBrushed};
  rev::spark::SparkMax m_rearRight{45, rev::spark::SparkMax::MotorType::kBrushed};

  // Mecanum drive system
  frc::MecanumDrive m_robotDrive;
};

#ifndef RUNNING_  cdFRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif

