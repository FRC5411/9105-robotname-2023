
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.GlobalVars;
import frc.robot.Constants.*;
import frc.robot.util.Logger;

public class DriveSubsystem extends SubsystemBase {

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftBackMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightBackMotor;

  private DifferentialDrive robotDrive;
  private LimelightSubsystem vision;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;
  
  private Pose2d initialPose;

  private AHRS navX;

  private DifferentialDrivePoseEstimator odometry;

  private Timer timer;
  private Logger dataLogger;
  private PowerDistribution PDH;

  public DriveSubsystem(LimelightSubsystem vision) {
    leftFrontMotor = new CANSparkMax(
      DrivebaseConstants.LF_MOTOR_CANID,
      CANSparkMax.MotorType.kBrushless
    );

    leftBackMotor = new CANSparkMax(
      DrivebaseConstants.LB_MOTOR_CANID,
      CANSparkMax.MotorType.kBrushless
    );
   
    rightFrontMotor = new CANSparkMax(
      DrivebaseConstants.RF_MOTOR_CANID,
      CANSparkMax.MotorType.kBrushless
    );
   
    rightBackMotor = new CANSparkMax(
      DrivebaseConstants.RB_MOTOR_CANID,
      CANSparkMax.MotorType.kBrushless
    );

    this.vision = vision;
    initialPose = new Pose2d();

    robotDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

    leftEncoder = leftFrontMotor.getEncoder();
    rightEncoder = rightFrontMotor.getEncoder();

    leftEncoder.setPositionConversionFactor(
      AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR
    );

    rightEncoder.setPositionConversionFactor(
      AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR
    );

    leftEncoder.setVelocityConversionFactor(
      AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR / 60
    );
   
    rightEncoder.setVelocityConversionFactor(
      AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR / 60
    );

    leftFrontMotor.setInverted(true);
    resetEncoders();

    navX = new AHRS(SPI.Port.kMXP);
    zeroHeading();

    odometry = new DifferentialDrivePoseEstimator(
      AutonoumousConstants.DRIVE_KINEMATICS, 
      navX.getRotation2d(), 
      leftEncoder.getPosition(), 
      rightEncoder.getPosition(), 
      initialPose);

    resetOdometry(getPose());

    dataLogger = new Logger();
    timer = new Timer();
    PDH = new PowerDistribution(DrivebaseConstants.PDH_PORT_CANID, ModuleType.kRev);
  }

  public void arcadeDrive(double speed, double rotation) {
    if (speed < DrivebaseConstants.DEADZONE && speed > -DrivebaseConstants.DEADZONE) {
      speed = 0;
    }
    else {
      if (speed > 0) {
        speed = speed * speed;
      }
      else {
        speed = -speed * speed;
      }
    }

    if (rotation < DrivebaseConstants.DEADZONE && rotation > -DrivebaseConstants.DEADZONE) {
      rotation = 0;
    }
    else {
      if (rotation > 0) {
        rotation = rotation * rotation;
      }
      else {
        rotation = -rotation * rotation;
      }
    }

    speed = (GlobalVars.sniperMode) ?  speed * DrivebaseConstants.SNIPER_SPEED : speed * DrivebaseConstants.SPEED_REDUCTION;
    rotation = (GlobalVars.sniperMode) ?  rotation * DrivebaseConstants.SNIPER_SPEED : rotation * DrivebaseConstants.ROTATION_REDUCTION;

    robotDrive.arcadeDrive(speed, rotation);
  }

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
    return odometry.getEstimatedPosition();
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
      new RamseteController(AutonoumousConstants.RAMSETE_B, AutonoumousConstants.RAMSETE_ZETA),
      new SimpleMotorFeedforward(AutonoumousConstants.VOLTS, AutonoumousConstants.VOLT_SECONDS_PER_METER, AutonoumousConstants.VOLT_SECONDS_SQUARED_PER_METER),
      AutonoumousConstants.DRIVE_KINEMATICS,
      this::getWheelSpeeds,
      new PIDController(AutonoumousConstants.DRIVE_VELOCITY, 0, 0),
      new PIDController(AutonoumousConstants.DRIVE_VELOCITY, 0, 0),
      this::setTankDriveVolts,
      this
      )
    );
  }

  @Override
  public void periodic() {
    if (vision.hastarget()) {
      odometry.addVisionMeasurement(vision.getPose(), 
      Timer.getFPGATimestamp());
    }

    odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    SmartDashboard.putNumber("Left Encoder Val: ", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Val: ", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro Heading: ", getGyroHeading());
    SmartDashboard.putNumber("Left Motor Temp: ", getLeftMotorTemp());
    SmartDashboard.putNumber("Right Motor Temp: ", getRightMotorTemp());

    if (timer.get() == 1) {
      timer.reset();
      double velocity = (getRightEncoderVelocity() + getLeftEncoderVelocity()) / 2;

      dataLogger.logTelemetryData(
        getLeftMotorTemp(),
        getRightMotorTemp(),
        velocity,
        PDH.getVoltage()
        ); 
    }
  }

  /**
  * Position robot for score type.
  * @param HeldObjectType - The type of object: cone(true), cube(false)
  * @param ScoreType - The type of score: low(0), mid(1), high(2)
  * @param Yaw - Angle to target
  * @param TargetDistance - Magnitude to target
  * @author Cody Washington
  */
  public void autoAlignment(boolean HeldObjectType, int ScoreType, double TargetAngle, double TargetDistance)
  {
    double Target_X_Distance = (Math.sin(TargetAngle) * TargetDistance);
    double Target_Z_Distance = (Math.cos(TargetAngle) * TargetDistance);
    ScoreType = (ScoreType > 2)? (2): (ScoreType);
    toAngle((Math.atan(Target_Z_Distance/Target_X_Distance) > 0)? (90): (-90));
    toDistance(((HeldObjectType && ((ScoreType == 1)? (true): (false)||(ScoreType == 2)? (true): (false)))? (Target_X_Distance-=18.5): (Target_X_Distance)));
    toAngle((Math.atan(Target_Z_Distance/Target_X_Distance) > 0)? (-180): (180));
    //Target type distance
    switch(ScoreType)
    {
      //Low score distance
      case 0:
        toDistance(DrivebaseConstants.LOW_SCORE_DISTANCE);
      //Mid score distance
      case 1:
        toDistance(DrivebaseConstants.MID_SCORE_DISTANCE);
      //High score distance
      case 2:
        toDistance(DrivebaseConstants.HIGH_SCORE_DISTANCE);
    }
  }

  /**
  * Position robot to new angle
  * @param Angle - New angle to reach.
  * @author Cody Washington
  */
  public void toAngle(double Angle)
  {
    Angle = (int)Math.round(Angle);
    double StartingYaw = (int)Math.round(navX.getAngle());
    while(Math.round(navX.getAngle()) != (StartingYaw + Angle))
    {
      //Positive turn
      if(Math.round(navX.getAngle()) < (StartingYaw + Angle))
        arcadeDrive(0.0,((navX.getAngle() - Angle)/(StartingYaw + Angle)));
      //Negative turn
      else
        arcadeDrive(0.0,(-(navX.getAngle() - Angle)/(StartingYaw + Angle)));
    }
  }

  /**
  * Position robot to new distance
  * @param Distance - Distance to move forwards
  * @author Cody Washington
  */
  public void toDistance(double Distance) 
  {
    double time = Timer.getFPGATimestamp();
    while(Math.round(Distance) != Math.round(rightBackMotor.getEncoder().getVelocity() * time))
    {
      time = Timer.getFPGATimestamp();
      //Positive translation
      if(Math.round(Distance) < Math.round(rightBackMotor.getEncoder().getVelocity() * time))
        arcadeDrive(Math.round((rightBackMotor.getEncoder().getPosition() / rightBackMotor.getEncoder().getCountsPerRevolution())),0.0);
      //Negative translation
      else
        arcadeDrive(-Math.round((rightBackMotor.getEncoder().getPosition() / rightBackMotor.getEncoder().getCountsPerRevolution())), 0.0);
    }
  }

  public void stop() {
    arcadeDrive(0, 0);
    GlobalVars.sniperMode = false;
  }
 
  @Override
  public void simulationPeriodic() {}
}
