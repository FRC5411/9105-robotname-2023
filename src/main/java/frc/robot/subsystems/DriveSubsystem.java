
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase 
{  
  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftBackMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightBackMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private DifferentialDrive robotDrive;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  public Gyro navX;

  private DifferentialDriveOdometry odometry;

  public DriveSubsystem()
  {
    //Drive
    leftFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.leftFrontMotor, 
    CANSparkMax.MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(Constants.DrivetrainConstants.leftBackMotor, 
    CANSparkMax.MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.rightFrontMotor, 
    CANSparkMax.MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(Constants.DrivetrainConstants.rightBackMotor, 
    CANSparkMax.MotorType.kBrushless);

    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);

    leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

    leftMotors.setInverted(true);

    leftFrontMotor.setSmartCurrentLimit(80);
    leftBackMotor.setSmartCurrentLimit(80);
    rightFrontMotor.setSmartCurrentLimit(80);
    rightBackMotor.setSmartCurrentLimit(80);

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);

    //Enocders
    leftEncoder = leftFrontMotor.getEncoder();
    rightEncoder = rightFrontMotor.getEncoder();

    leftEncoder.setPositionConversionFactor
    (Constants.DriveConstants.kLinearDistanceConversionFactor);
    rightEncoder.setPositionConversionFactor
    (Constants.DriveConstants.kLinearDistanceConversionFactor);

    leftEncoder.setVelocityConversionFactor
    (Constants.DriveConstants.kLinearDistanceConversionFactor / 60); //Divides by 60, so it's per 60 seconds
    rightEncoder.setVelocityConversionFactor
    (Constants.DriveConstants.kLinearDistanceConversionFactor / 60);

    resetEncoders();

    //Gyro
    navX = new AHRS(SPI.Port.kMXP);

    navX.reset();
    navX.calibrate();

    //Odeometry
    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    odometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getGyroHeading(), getPose());
  }

  public void arcadeDrive(double speed, double rotation)
  {
    //Setting "deadzones"
    if (speed < 0.1 && speed > -0.1)
    {
      speed = 0;
    }
    if (rotation < 0.1 && rotation > -0.1)
    {
      rotation = 0;
    }

    speed = speed * 0.7;
    robotDrive.arcadeDrive(speed, rotation, true);
  }

  //Lots of getter/setter methods

  public double getRightEncoderPosition()
  {
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderPosition()
  {
    return -leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity()
  {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity()
  {
    return -leftEncoder.getVelocity();
  }

  public double getGyroHeading()
  {
    return navX.getRotation2d().getDegrees(); //Make sure it's in degrees
  }

  public double getTurnRate()
  {
    return -navX.getRate(); //Returns turn rate in degrees per sec
  }

  public Pose2d getPose()
  {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds()
  {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public double getAverageEncoderDistance()
  {
    return (leftEncoder.getPosition() + rightEncoder.getPosition()) / 2.0;
  }

  public RelativeEncoder getRightEncoder()
  {
    return rightEncoder;
  }

  public RelativeEncoder getLeftEncoder()
  {
    return leftEncoder;
  }
  
  public Gyro getGyro()
  {
    return navX;
  }

  public double getLeftMotorTemp()
  {
    return leftFrontMotor.getMotorTemperature();
  }

  public double getRightMotorTemp()
  {
    return rightFrontMotor.getMotorTemperature();
  }

  public void resetOdometry(Pose2d pose)
  {
    resetEncoders();
    odometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getGyroHeading(), pose);
  }

  public void resetEncoders()
  {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public void setTankDriveVolts(double leftVolts, double rightVolts)
  {
    rightMotors.setVoltage(rightVolts);
    leftMotors.setVoltage(leftVolts);
    robotDrive.feed();
  }

  public void setMaxOutput(double maxOutput)
  {
    robotDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading()
  {
    navX.calibrate();
    navX.reset();
  }

  //End of getter/setter methods

  @Override
  public void periodic() 
  {
    odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    //Smartdash goes here
    SmartDashboard.putNumber("Left Encoder Val: ", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Val: ", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro Heading: ", getGyroHeading());
    SmartDashboard.putNumber("Left Motor Temp: ", getLeftMotorTemp());
    SmartDashboard.putNumber("Right Motor Temp: ", getRightMotorTemp());
  }

  @Override
  public void simulationPeriodic() {}
}
