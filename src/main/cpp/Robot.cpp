#include <cmath>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <AHRS.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>

#include <WPILibVersion.h>
#include <frc/smartdashboard/SmartDashboard.h>

// #include "Constants.h"
using namespace std;

const double rotationVectorMultiplier = 1;   // controls how much of the vector addition is dedicated to rotation vs field movement  0 < x < double limit idk
const double speedLimit = 0.2;   // limit motor speed output   0 < x <= 1
const double baseWidth = 14.5;   // inches
const double baseLength = 23.25;

// For determining motor positions on an x,y grid
const double relativeX = baseWidth / 2;
const double relativeY = baseLength / 2;

double magnitude(double x, double y) // magnitude of vector
{
    return sqrt(x*x + y*y);
}

int magnitudeOptimization(double initialAngle, double finalAngle)
{
    double posDiff = fabs(fmod(finalAngle-initialAngle, M_PI*2));
    if(posDiff>M_PI_2 && posDiff<M_PI_2*3){
        return -1;
    }
    return 1;
}
double angleOptimisation(double initialAngle, double finalAngle)
{
    double diff = fmod(finalAngle-initialAngle, M_PI*2);
    double posDiff = fabs(diff);
    if(posDiff>M_PI_2 && posDiff<M_PI_2*3){
        diff = fmod(diff + M_PI, M_PI*2);
    }
    posDiff = fabs(diff);
    
    if(posDiff<=M_PI_2){
        return diff;
    }
    else if(diff>=3*M_PI_2){
        return diff-(2*M_PI);
    }
    else{
        return diff+(2*M_PI);
    }
}

double findMax(double arr[], int a) // finds max value in array
{
    double sus = max(arr[0],arr[1]);
    for(int i=2; i<a; i++){
        sus = max(sus,arr[i]);
    }
    return sus;
}

double getDegree(double x)
{
    double a = x*180.0/M_PI;
    return a;
}

double deadband(double joystickInput)
{
  if(abs(joystickInput)<=0.2){
    return 0.0;
  }
  return joystickInput;
}

struct swerveModule{
    double flm, fla, frm, fra, blm, bla, brm, bra;
    swerveModule(double flm, double fla, double frm, double fra, double blm, double bla, double brm, double bra):
    flm(flm), fla(fla), frm(frm), fra(fra), blm(blm), bla(bla), brm(brm), bra(bra){}
};

swerveModule swerveKinematics(double xLeft, double yLeft, double xRight, double gyro, double FLTheta, double FRTheta, double BLTheta, double BRTheta)
{
    FLTheta=getDegree(FLTheta);
    FRTheta=getDegree(FRTheta);
    BLTheta=getDegree(BLTheta);
    BRTheta=getDegree(BRTheta);
    double posVect[2];
    if(magnitude(xLeft, yLeft)){
        posVect[0] = magnitude(xLeft,yLeft)*sin(atan2(xLeft,yLeft)-gyro);
        posVect[1] = magnitude(xLeft,yLeft)*cos(atan2(xLeft,yLeft)-gyro); // math notation of vector
    }
    else{
        if(xRight<0.1){
            double swerveNumbers[4][2] = {{0.0,10.0}, {0.0,0.0}, {0.0,0.0}, {0.0,0.0}};
            return swerveModule(swerveNumbers[0][0],swerveNumbers[0][1],swerveNumbers[1][0],swerveNumbers[1][1],swerveNumbers[2][0],swerveNumbers[2][1],swerveNumbers[3][0],swerveNumbers[3][1]);
        }
        posVect[0] = 0;
        posVect[1] = 0;
    }
    
    double rotationScalar = rotationVectorMultiplier*xRight/magnitude(relativeX,relativeY);
    
    // declare rotation vector directions and add positional
    double frontLeft[2] = {(rotationScalar*relativeY)+posVect[0], (rotationScalar*relativeX)+posVect[1]};
    double frontRight[2] = {(rotationScalar*relativeY)+posVect[0], -(rotationScalar*relativeX)+posVect[1]};
    double backLeft[2] = {-(rotationScalar*relativeY)+posVect[0], (rotationScalar*relativeX)+posVect[1]};
    double backRight[2] = {-(rotationScalar*relativeY)+posVect[0], -(rotationScalar*relativeX)+posVect[1]}; 
    
    // the most sketchy thing ever.
    double magnitudes[4] = {magnitude(frontLeft[0], frontLeft[1]), magnitude(frontRight[0], frontRight[1]), magnitude(backLeft[0], backLeft[1]), magnitude(backRight[0], backRight[1])};
    double limitingScalar = speedLimit/max(1.0, findMax(magnitudes, sizeof(magnitudes)/sizeof(magnitudes[0])));

    double physFL[2] = {limitingScalar*magnitudes[0], atan2(frontLeft[0], frontLeft[1])}; // converts to physics notation
    double physFR[2] = {limitingScalar*magnitudes[1], atan2(frontRight[0], frontRight[1])};
    double physBL[2] = {limitingScalar*magnitudes[2], atan2(backLeft[0], backLeft[1])};
    double physBR[2] = {limitingScalar*magnitudes[3], atan2(backRight[0], backRight[1])};
    //-----
    double FL[2] = {magnitudeOptimization(FLTheta, physFL[1])*physFL[0], getDegree(angleOptimisation(FLTheta, physFL[1]))};
    double FR[2] = {magnitudeOptimization(FRTheta, physFR[1])*physFR[0], getDegree(angleOptimisation(FRTheta, physFR[1]))};
    double BL[2] = {magnitudeOptimization(BLTheta, physBL[1])*physBL[0], getDegree(angleOptimisation(BLTheta, physBL[1]))};    
    double BR[2] = {magnitudeOptimization(BRTheta, physBR[1])*physBR[0], getDegree(angleOptimisation(BRTheta, physBR[1]))};

    //*** [][0] = [Module][motor strength]; [][1] = [Module][angle to turn]
    //[Module] order: [0,1,2,3] = [FL,FR,BL,BR]
    double *swerveNumbers[4] = {FL, FR, BL, BR};
    return swerveModule(swerveNumbers[0][0],swerveNumbers[0][1],swerveNumbers[1][0],swerveNumbers[1][1],swerveNumbers[2][0],swerveNumbers[2][1],swerveNumbers[3][0],swerveNumbers[3][1]);
}

class Robot : public frc::TimedRobot {
 public:
  void TeleopInit() override {
    m_FLSwerveMotor.SetInverted(true);
    m_FRSwerveMotor.SetInverted(true);
    m_BLSwerveMotor.SetInverted(true);
    m_BRSwerveMotor.SetInverted(true);
    
    m_FLDriveMotor.SetNeutralMode(NeutralMode::Brake);
    m_FRDriveMotor.SetNeutralMode(NeutralMode::Brake);
    m_BLDriveMotor.SetNeutralMode(NeutralMode::Brake);
    m_BRDriveMotor.SetNeutralMode(NeutralMode::Brake);
    
    m_FLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    m_FRSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    m_BLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
    m_BRSwerveMotor.SetNeutralMode(NeutralMode::Brake);

    // encoderTurn.velocityMeasurementPeriod = ctre::phoenix::sensors::SensorVelocityMeasPeriod::Period_100Ms;
    // encoderTurn.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Signed_PlusMinus180;
    // encoderTurn.sensorDirection = false;
    // encoderTurn.sensorCoefficient = M_PI / 2048.0;
    // encoderTurn.sensorTimeBase = ctre::phoenix::sensors::SensorTimeBase::PerSecond;
  }
  void TeleopPeriodic() override {
    swerveModule moduleDesiredStates = swerveKinematics(deadband(m_Controller.GetLeftX()), deadband(m_Controller.GetLeftY()), deadband(m_Controller.GetRightX()), m_navX.GetCompassHeading(), FLCANCoder.GetPosition(), FRCANCoder.GetPosition(), BLCANCoder.GetPosition(), BRCANCoder.GetPosition());
    if(moduleDesiredStates.flm<8){
          frc::SmartDashboard::PutNumber("AFL", moduleDesiredStates.fla);
          frc::SmartDashboard::PutNumber("AFR", moduleDesiredStates.fra);
          frc::SmartDashboard::PutNumber("ABL", moduleDesiredStates.bla);
          frc::SmartDashboard::PutNumber("ABR", moduleDesiredStates.bra);

          // m_FLSwerveMotor.Set(ControlMode::Position, moduleDesiredStates.fla);
          // m_FRSwerveMotor.Set(ControlMode::Position, moduleDesiredStates.fra);
          // m_BLSwerveMotor.Set(ControlMode::Position, moduleDesiredStates.bla);
          // m_BRSwerveMotor.Set(ControlMode::Position, moduleDesiredStates.bra);
    }
    frc::SmartDashboard::PutNumber("MFL", moduleDesiredStates.flm);
    frc::SmartDashboard::PutNumber("MFR", moduleDesiredStates.frm);
    frc::SmartDashboard::PutNumber("MBL", moduleDesiredStates.blm);
    frc::SmartDashboard::PutNumber("MBR", moduleDesiredStates.brm);
    , deadband(m_Controller.GetLeftY()), deadband(m_Controller.GetRightX()), m_navX.GetCompassHeading()
    frc::SmartDashboard::PutNumber("MFR", deadband(m_Controller.GetLeftX()));
    frc::SmartDashboard::PutNumber("MFR", moduleDesiredStates.frm);
    frc::SmartDashboard::PutNumber("MBL", moduleDesiredStates.blm);
    frc::SmartDashboard::PutNumber("MBR", moduleDesiredStates.brm);
    // m_FLDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates.flm);
    // m_FRDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates.frm);
    // m_BLDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates.blm);
    // m_BRDriveMotor.Set(ControlMode::PercentOutput, moduleDesiredStates.brm);
   }

 private:
  frc::XboxController m_Controller{0};
  // ctre::phoenix::sensors::CANCoderConfiguration encoderturn;

  //frc::SmartDashboard::SmartDashboard();

  // Drive Motors
  TalonFX m_FLDriveMotor{3};
  TalonFX m_FRDriveMotor{1};
  TalonFX m_BLDriveMotor{2};
  TalonFX m_BRDriveMotor{4};
  
  // Swerve Motors
  TalonFX m_FLSwerveMotor{7};
  TalonFX m_FRSwerveMotor{5};
  TalonFX m_BLSwerveMotor{6};
  TalonFX m_BRSwerveMotor{8};

  // Encoders
  CANCoder FLCANCoder{11};
  CANCoder FRCANCoder{9};
  CANCoder BLCANCoder{10};
  CANCoder BRCANCoder{12};

  // Gyro
  AHRS m_navX{frc::SPI::kMXP};         
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif