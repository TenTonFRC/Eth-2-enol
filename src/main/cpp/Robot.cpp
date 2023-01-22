#include <cmath>

#include <frc/TimedRobot.h>
#include <ctre/Phoenix.h>
#include <frc/XboxController.h>
#include <AHRS.h>
#include <ctre/phoenix/sensors/WPI_CANCoder.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>
#include "constants.h"

// Proportional value given to turn motors
// Desired turn angles; it's multipurpose to remove clutter :p
double desiredTurnFL;
double desiredTurnFR;
double desiredTurnBL;
double desiredTurnBR;

// Processed drive motor speed
double driveFL;
double driveFR;
double driveBL;
double driveBR;

// magnitude of vector to convert from Cartesian to Polar
double magnitude(double x, double y) 
{
    return sqrt(x * x + y * y);
}

// depending on the angles, the optimal turn may reverse motor direction
// Returns 1 or -1 based on optimal turn angle
// Takes initial and desired angle
int magnitudeOptimization(double initialAngle, double finalAngle)
{
    double posDiff = fabs(fmod(finalAngle - initialAngle, 360.0));
    if (posDiff > 90.0 && posDiff < 270.0)
    {
        return -1;
    }
    return 1;
}

// ideally, the wheel will only ever turn 90 degrees clockwise or counterclockwise and change drive motor direction to reach desired angle
// Returns optimal turn angle based 
// Takes initial and desired angle
double angleOptimisation(double initialAngle, double finalAngle)
{
    double diff = fmod(finalAngle - initialAngle, 360.0);
    double posDiff = fabs(diff);
    if (posDiff > 90.0 && posDiff < 270.0)
    {
        diff = fmod(diff + 180.0, 360.0);
    }
    if (fabs(diff) <= 90.0)
    {
        return diff;
    }
    else if (diff >= 270.0)
    {
        return diff - 360.0;
    }
    return diff + 360.0;
}

// Given an array of values, find the largest member greater than 1
// if all members are less than one, return 1
// else, return largest value
double findMax(double arr[], int len) // finds max value in array
{
    double Max = 1.0;
    for (int i = 0; i < len; i++)
    {
        Max = fmax(Max, arr[i]);
    }
    return Max;
}

// Convert radian to degree
double getDegree(double x)
{
    return x * 180.0 / M_PI;
}

// Convert degree to radian
double getRadian(double x)
{
    return x * M_PI / 180.0;
}

// Deadband for joystick
// prevents drift at values close to 0
double deadband(double joystickInput)
{
    if (abs(joystickInput) <= mathConst::deadband)
    {
        return 0.0;
    }
    return joystickInput;
}

// all doubles
// order: front left magnitude, front left angle, frm, fra, blm, bla, brm, bra
// percentage magnitude
// degree angle
struct swerveModule
{
    double flm, fla, frm, fra, blm, bla, brm, bra;
    swerveModule(double flm, double fla, double frm, double fra, double blm, double bla, double brm, double bra) : flm(flm), fla(fla), frm(frm), fra(fra), blm(blm), bla(bla), brm(brm), bra(bra) {}
};

// Stores a vector
struct Point
{
    double x, y;
    Point(double x, double y) : x(x), y(y) {}
};

// Input degrees
// Automatically applies deadbands
// Outputs a swerveModule object 
// (percentage speeds and angles in the order FL, FR, BL, BR)
swerveModule swerveKinematics(double xLeft, double yLeft, double xRight, double gyro)
{
    // cmath reads radians; applies deadbands
    gyro = getRadian(gyro);
    xLeft = deadband(xLeft);
    yLeft = deadband(yLeft);
    xRight = deadband(xRight);

    Point posVector = Point(0.0, 0.0);
    double joystickMagnitude = magnitude(xLeft, yLeft);
    if (joystickMagnitude)  // Check if left joystick has an input
    {
        // atan2 converts joystick input into angle
        // + gyro makes desired angle calculation field relative 
        // (was -gyro in math but +gyro works in practice?)
        double fieldRelativePosAngle = atan2(xLeft, yLeft) + gyro;
        
        // convert angles back into Cartesian
        posVector.x = joystickMagnitude * sin(fieldRelativePosAngle);
        posVector.y = joystickMagnitude * cos(fieldRelativePosAngle);
    }
    else if (abs(xRight) < 0.1) // the < 0.1 is another reduncancy that is within the deadband
    {
        // if no joystick input, return exit code
        return swerveModule(0.0, 1000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    // Creates a unit vector multiplied by right joystick input and proportionally scaled by "rotationVectorMultiplier"
    double rotationScalar = mathConst::rotationVectorMultiplier * xRight / magnitude(mathConst::relativeX, mathConst::relativeY);

    // Apply rotation vectors to positional vectors to create the combined vector
    // negatives and "y, x" are assigned (negative reciprocal = perpendicular line)
    // actually the y values were negated again because it just worked. I don't know if I did math wrong, but it worked so there's another arbitrary negation
    Point rawFL = Point((rotationScalar * mathConst::relativeY) + posVector.x, -(rotationScalar * mathConst::relativeX) + posVector.y);
    Point rawFR = Point((rotationScalar * mathConst::relativeY) + posVector.x, (rotationScalar * mathConst::relativeX) + posVector.y);
    Point rawBL = Point(-(rotationScalar * mathConst::relativeY) + posVector.x, -(rotationScalar * mathConst::relativeX) + posVector.y);
    Point rawBR = Point(-(rotationScalar * mathConst::relativeY) + posVector.x, (rotationScalar * mathConst::relativeX) + posVector.y);

    // compares magnitudes of resulting vectors to see if any composite vector (rotation + position) exceeded "100%" output speed. 
    // Divide by largest value greate than 100% to limit all magnitudes to 100% at max whilst maintaining relative rotational speeds
    // Limiting Scalar also applies the motor speed limit cap
    double magnitudes[4] = {magnitude(rawFL.x, rawFL.y), magnitude(rawFR.x, rawFR.y), magnitude(rawBL.x, rawBL.y), magnitude(rawBR.x, rawBR.y)};
    double limitingScalar = mathConst::speedLimit / findMax(magnitudes, sizeof(magnitudes) / sizeof(magnitudes[0]));
    
    // Convert to Polar vectors for speed and direction for swerve modules
    Point physFL = Point(limitingScalar * magnitudes[0], atan2(rawFL.x, rawFL.y));
    Point physFR = Point(limitingScalar * magnitudes[1], atan2(rawFR.x, rawFR.y));
    Point physBL = Point(limitingScalar * magnitudes[2], atan2(rawBL.x, rawBL.y));
    Point physBR = Point(limitingScalar * magnitudes[3], atan2(rawBR.x, rawBR.y));

    return swerveModule(physFL.x, getDegree(physFL.y), physFR.x, getDegree(physFR.y), physBL.x, getDegree(physBL.y), physBR.x, getDegree(physBR.y));
}

class Robot : public frc::TimedRobot
{
    public:
        void TeleopInit() override
        {
            m_FLDriveMotor.SetNeutralMode(NeutralMode::Brake);
            m_FRDriveMotor.SetNeutralMode(NeutralMode::Brake);
            m_BLDriveMotor.SetNeutralMode(NeutralMode::Brake);
            m_BRDriveMotor.SetNeutralMode(NeutralMode::Brake);

            m_FLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
            m_FRSwerveMotor.SetNeutralMode(NeutralMode::Brake);
            m_BLSwerveMotor.SetNeutralMode(NeutralMode::Brake);
            m_BRSwerveMotor.SetNeutralMode(NeutralMode::Brake);

            m_navX.ZeroYaw();

        // Uncomment this, run it, deploy with this commented out again, do not run, turn off robot, zero wheels manually, boot
        // Zeros CANCoders (if someone finds a better way, PLEASE implement it ASAP)
            // FLCANCoder.ConfigSensorInitializationStrategy(BootToZero);
            // FRCANCoder.ConfigSensorInitializationStrategy(BootToZero);
            // BLCANCoder.ConfigSensorInitializationStrategy(BootToZero);
            // BRCANCoder.ConfigSensorInitializationStrategy(BootToZero);
            FLCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
            FRCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
            BLCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
            BRCANCoder.ConfigAbsoluteSensorRange(Signed_PlusMinus180);
        }
        void TeleopPeriodic() override
        {
            // moduleDesiredStates contains all calculated desired values.
            // .flm is "Front left magnitude" (percentage)
            // .fla is "Front left angle" (degrees)
            // fl[], fr[], bl[], br[]
            swerveModule moduleDesiredStates = swerveKinematics(m_Controller.GetLeftX(), m_Controller.GetLeftY(), m_Controller.GetRightX(), m_navX.GetAngle());
            
            // when controller joysticks have no input, fla is 1000.0 (pseudo exit code)
            // this only updates the "desired angle" read by the turn motors if the exit code is not detected
            // 600 is an arbitrary value that is over a full rotation less than 1000 for reduncancy; anything 90<x<1000 works
            if (moduleDesiredStates.fla < 600.0)
            {
                // Update wheel angles for the turn motors to read
                desiredTurnFL = moduleDesiredStates.fla;
                desiredTurnFR = moduleDesiredStates.fra;
                desiredTurnBL = moduleDesiredStates.bla;
                desiredTurnBR = moduleDesiredStates.bra;
            }

            // convert desired angle to optimal turn angle and divide by 90 degrees to convert to percentage
            // limit motor turn speed
            desiredTurnFL = mathConst::speedLimit/90.0*angleOptimisation(FLCANCoder.GetPosition(), desiredTurnFL);
            m_FLSwerveMotor.Set(TalonFXControlMode::PercentOutput, desiredTurnFL);

            desiredTurnFR = mathConst::speedLimit/90.0*angleOptimisation(FRCANCoder.GetPosition(), desiredTurnFR);
            m_FRSwerveMotor.Set(TalonFXControlMode::PercentOutput, desiredTurnFR);

            desiredTurnBL = mathConst::speedLimit/90.0*angleOptimisation(BLCANCoder.GetPosition(), desiredTurnBL);
            m_BLSwerveMotor.Set(TalonFXControlMode::PercentOutput, desiredTurnBL);
 
            desiredTurnBR = mathConst::speedLimit/90.0*angleOptimisation(BRCANCoder.GetPosition(), desiredTurnBR);
            m_BRSwerveMotor.Set(TalonFXControlMode::PercentOutput, desiredTurnBR);
            
            // Controls whether the wheels go forwards or backwards depending on the ideal turn angle
            driveFL = moduleDesiredStates.flm*magnitudeOptimization(FLCANCoder.GetPosition(), moduleDesiredStates.fla);
            driveFR = moduleDesiredStates.frm*magnitudeOptimization(FRCANCoder.GetPosition(), moduleDesiredStates.fra);
            driveBL = moduleDesiredStates.blm*magnitudeOptimization(BLCANCoder.GetPosition(), moduleDesiredStates.bla);
            driveBR = moduleDesiredStates.brm*magnitudeOptimization(BRCANCoder.GetPosition(), moduleDesiredStates.bra);
                
            m_FLDriveMotor.Set(ControlMode::PercentOutput, driveFL);
            m_FRDriveMotor.Set(ControlMode::PercentOutput, driveFR);
            m_BLDriveMotor.Set(ControlMode::PercentOutput, driveBL);
            m_BRDriveMotor.Set(ControlMode::PercentOutput, driveBR);

        // Debug Math Outputs
            // Drive motor speeds (percentage)
            frc::SmartDashboard::PutNumber("MFL", driveFL);
            frc::SmartDashboard::PutNumber("MFR", driveFR);
            frc::SmartDashboard::PutNumber("MBL", driveBL);
            frc::SmartDashboard::PutNumber("MBR", driveBR);
            
            // Desired turn angles (degrees)
            frc::SmartDashboard::PutNumber("AFL", desiredTurnFL);
            frc::SmartDashboard::PutNumber("AFR", desiredTurnFR);
            frc::SmartDashboard::PutNumber("ABL", desiredTurnBL);
            frc::SmartDashboard::PutNumber("ABR", desiredTurnBR);
            
            // Gyro angle (degrees)
            frc::SmartDashboard::PutNumber("Yaw", m_navX.GetAngle());

            // Zero gyro (sets PDP direction to North)
            if (m_Controller.GetAButtonPressed())
            {
                m_navX.ZeroYaw();
            }
        }

    private:
        // Initialize all components
        
        // Main Controller
        frc::XboxController m_Controller{ControllerIDs::kControllerMainID};

        // Drive Motors
        TalonFX m_FLDriveMotor{CanIDs::kFLDriveMotor};
        TalonFX m_FRDriveMotor{CanIDs::kFRDriveMotor};
        TalonFX m_BLDriveMotor{CanIDs::kBLDriveMotor};
        TalonFX m_BRDriveMotor{CanIDs::kBRDriveMotor};

        // Swerve Motors
        TalonFX m_FLSwerveMotor{CanIDs::kFLSwerveMotor};
        TalonFX m_FRSwerveMotor{CanIDs::kFRSwerveMotor};
        TalonFX m_BLSwerveMotor{CanIDs::kBLSwerveMotor};
        TalonFX m_BRSwerveMotor{CanIDs::kBRSwerveMotor};

        // Encoders
        CANCoder FLCANCoder{CanIDs::kFLCANCoder};
        CANCoder FRCANCoder{CanIDs::kFRCANCoder};
        CANCoder BLCANCoder{CanIDs::kBLCANCoder};
        CANCoder BRCANCoder{CanIDs::kBRCANCoder};

        // Gyro
        AHRS m_navX{frc::SPI::kMXP};
};

#ifndef RUNNING_FRC_TESTS
int main()
{
    return frc::StartRobot<Robot>();
}
#endif