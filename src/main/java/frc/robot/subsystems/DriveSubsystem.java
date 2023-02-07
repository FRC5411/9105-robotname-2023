
package frc.robot.subsystems;

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

  private double DEADZONE_VAL;
  private double SNIPER_SPEED;
  private double SPEED;
  private double ROTATION;

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

    /* 
    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);

    leftBackMotor.follow(leftFrontMotor);
    rightBackMotor.follow(rightFrontMotor);

    leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

    leftMotors.setInverted(true);
     
    leftFrontMotor.setSmartCurrentLimit(Constants.DrivebaseConstants.MOTOR_AMP_LIMIT);
    leftBackMotor.setSmartCurrentLimit(Constants.DrivebaseConstants.MOTOR_AMP_LIMIT);
    rightFrontMotor.setSmartCurrentLimit(Constants.DrivebaseConstants.MOTOR_AMP_LIMIT);
    rightBackMotor.setSmartCurrentLimit(Constants.DrivebaseConstants.MOTOR_AMP_LIMIT);
    */

    robotDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    DEADZONE_VAL = Constants.DrivebaseConstants.DEADZONE;
    SNIPER_SPEED = Constants.DrivebaseConstants.SNIPER_SPEED;
    SPEED = 0.95;
    ROTATION = 0.4;

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
    odometry = new DifferentialDriveOdometry(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    resetOdometry(getPose());
  }

  public void arcadeDrive(double speed, double rotation, boolean sniperMode) {
    //speed = (speed < 0.1 && speed > -0.1) ? 0 : speed * 0.7; // Also reduces the speed to 70%
    //rotation = (rotation < 0.1 && rotation > -0.1) ? 0 : rotation;

    //speed = (speed < Constants.DrivebaseConstants.DEADZONE && speed > -Constants.DrivebaseConstants.DEADZONE) ? ((speed > 0) ? Math.sqrt(speed) : -1 * Math.sqrt(Math.abs(speed))) : speed;
    //rotation = (rotation < Constants.DrivebaseConstants.DEADZONE && rotation > -Constants.DrivebaseConstants.DEADZONE) ? ((rotation > 0) ? Math.sqrt(rotation) : -1 * Math.sqrt(Math.abs(rotation))) : rotation;
    
    if (speed < DEADZONE_VAL && speed > -DEADZONE_VAL) {
      if (speed > 0) {
        speed = Math.sqrt(speed);
      }
      else {
        speed = -1 * Math.sqrt(Math.abs(speed));
      }
    }

    if (rotation < DEADZONE_VAL && rotation > -DEADZONE_VAL) {
      if (rotation > 0) {
        rotation = Math.sqrt(rotation);
      }
      else {
        rotation = -1 * Math.sqrt(Math.abs(rotation));
      }
    }

    speed = (sniperMode) ?  speed * SNIPER_SPEED : speed * SPEED;
    rotation = (sniperMode) ?  rotation * SNIPER_SPEED : rotation * ROTATION;

    /* 
    leftMotors.set(speed - rotation);
    rightMotors.set(speed + rotation);

    robotDrive.feed();*/
    robotDrive.arcadeDrive(speed, rotation);
  }

  /* Autonomous Getter / Setter Methods */

  public double getLeftEncoderPosition() {
    return -leftEncoder.getPosition();
  }

  public double getRightEncoderPosition() {
    return rightEncoder.getPosition();
  }

  public double getLeftEncoderVelocity() {
    return -leftEncoder.getVelocity();
  }

  public double getRightEncoderVelocity() {
    return rightEncoder.getVelocity();
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
    leftFrontMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(rightVolts);
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

  /* Auton Command */
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

  /* Auto Engage */
  public Command autoEngage(double setpoint) {
    // Note: tune PIDs
    int P = 1;
    int I = 1;
    int D = 1;
    PIDController pid = new PIDController(P, I, D);
    double pitch = navX.getPitch();
    double speed = pid.calculate(pitch, setpoint);

    return new SequentialCommandGroup(
      new InstantCommand(
        () -> {
          leftFrontMotor.set(speed);
          rightFrontMotor.set(speed);
        }
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
