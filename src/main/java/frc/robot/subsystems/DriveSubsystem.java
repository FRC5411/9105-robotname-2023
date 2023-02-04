
package frc.robot.subsystems;

import java.util.function.BiConsumer;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftBackMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightBackMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private DifferentialDrive robotDrive;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private AHRS navX;

  private DifferentialDriveOdometry odometry;

  public DriveSubsystem() {

    /* Robot Drive */
    leftFrontMotor = new CANSparkMax(
      Constants.DrivebaseConstants.LF_MOTOR,
      CANSparkMax.MotorType.kBrushless
    );

    leftBackMotor = new CANSparkMax(
      Constants.DrivebaseConstants.LB_MOTOR,
      CANSparkMax.MotorType.kBrushless
    );
    
    rightFrontMotor = new CANSparkMax(
      Constants.DrivebaseConstants.RF_MOTOR,
      CANSparkMax.MotorType.kBrushless
    );
    
    rightBackMotor = new CANSparkMax(
      Constants.DrivebaseConstants.RB_MOTOR,
      CANSparkMax.MotorType.kBrushless
    );

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

    /* Encoders */
    leftEncoder = leftFrontMotor.getEncoder();
    rightEncoder = rightFrontMotor.getEncoder();

    leftEncoder.setPositionConversionFactor(
      Constants.AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR
    );

    rightEncoder.setPositionConversionFactor(
      Constants.AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR
    );

    leftEncoder.setVelocityConversionFactor(
      Constants.AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR / 60
    );
    
    rightEncoder.setVelocityConversionFactor(
      Constants.AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR / 60
    );

    resetEncoders();

    /* Gyro */
    navX = new AHRS(SPI.Port.kMXP);
    zeroHeading();

    /* Odometry */
    odometry = new DifferentialDriveOdometry
    (navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    resetOdometry(getPose());
  }

  public void arcadeDrive(double speed, double rotation, boolean sniperMode) {

    speed = (speed < 0.1 && speed > -0.1) ? 0 : speed * 0.7; // Also reduces the speed to 70%
    speed = (sniperMode) ? speed : speed * 0.5;
    rotation = (rotation < 0.1 && rotation > -0.1) ? 0 : rotation;

    leftMotors.set(speed + rotation);
    rightMotors.set(speed - rotation);
  }

  /* Autonomous Getter / Setter Methods */

  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderPosition() {
    return -leftEncoder.getPosition();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
  }

  public double getLeftEncoderVelocity() {
    return -leftEncoder.getVelocity();
  }

  public double getGyroHeading() {
    return navX.getRotation2d().getDegrees(); //Make sure it's in degrees
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
  }

  public double getLeftMotorTemp() {
    return leftFrontMotor.getMotorTemperature();
  }

  public double getRightMotorTemp() {
    return rightFrontMotor.getMotorTemperature();
  }

  public void setTankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    robotDrive.feed();
  }

  public void setMaxOutput(double maxOutput) {
    robotDrive.setMaxOutput(maxOutput);
  }

  public void zeroHeading() {
    navX.calibrate();
    navX.reset();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(navX.getRotation2d(), getLeftEncoderPosition(), getGyroHeading(), pose);
  }

  public void resetEncoders() {
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public Command followPath(PathPlannerTrajectory trajectory, boolean resetOdometry) {

    return new SequentialCommandGroup(
      new InstantCommand(() -> {
        if (resetOdometry) {
          this.resetOdometry(trajectory.getInitialPose());
        }
      }
    ),
    new PPRamseteCommand(
      trajectory,
      this::getPose,
      new RamseteController(Constants.AutonoumousConstants.RAMSETE_B, Constants.AutonoumousConstants.RAMSETE_ZETA),
      new SimpleMotorFeedforward(Constants.AutonoumousConstants.VOLTS, Constants.AutonoumousConstants.VOLT_SECONDS_PER_METER, Constants.AutonoumousConstants.VOLT_SECONDS_SQUARED_PER_METER),
      Constants.AutonoumousConstants.DRIVE_KINEMATICS,
      this::getWheelSpeeds,
      new PIDController(Constants.AutonoumousConstants.DRIVE_VELOCITY, 0, 0),
      new PIDController(Constants.AutonoumousConstants.DRIVE_VELOCITY, 0, 0),
      this::setTankDriveVolts,
      this
      )
    );
  }

  /* Dashboard Display */
  @Override
  public void periodic() {
    odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putNumber("Left Encoder Val: ", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Val: ", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro Heading: ", getGyroHeading());
    SmartDashboard.putNumber("Left Motor Temp: ", getLeftMotorTemp());
    SmartDashboard.putNumber("Right Motor Temp: ", getRightMotorTemp());
  }

  @Override
  public void simulationPeriodic() {}
}
