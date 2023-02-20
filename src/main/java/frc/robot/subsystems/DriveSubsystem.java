
package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonoumousConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.GlobalVars;
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
   * Align the robot with a given apriltag target
   * @param Limelight - The limelight subsystem responsible for tracking targets
   * @author Cody Washington
   */
  public void apriltagAlignment(LimelightSubsystem Limelight)
  {
    GlobalVars.sniperMode = true;
    //Case one; target oriented to direct (Within 5 degrees of error)
    if(Math.abs(Limelight.getYaw()) > 15)
    {
      Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation());
      switch(GlobalVars.scoreType)
      {
        case 0:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.LOW_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.LOW_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
        case 1:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.MID_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.MID_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
        case 2:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.HIGH_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.HIGH_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
      }
    }
    //Case two; target oriented to perpendicular (Within 5 degrees of error)
    else if (Math.abs(Math.round(Limelight.getPose().minus(Limelight.getTarget2d()).getRotation().getDegrees())) < 15)
    {
      switch(GlobalVars.gamePieceMode)
      {
        case "CONE":
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.CONE_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.CONE_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
        case "CUBE":
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.CUBE_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.CUBE_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
      }
      arcadeDrive(0.0, (Limelight.getTarget2d().getRotation().getDegrees() > 0)? (-90): (90));
      switch(GlobalVars.scoreType)
      {
        case 0:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.LOW_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.LOW_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
        case 1:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.MID_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.MID_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
        case 2:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.HIGH_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.HIGH_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
      }
    }
    //Case three; target oriented other
    else
    {
      Pose2d PerpendicularPose = new Pose2d(((Limelight.getPose().getY() * Limelight.getTarget2d().getRotation().getDegrees()) - (Limelight.getPose().getRotation().getDegrees() * Limelight.getTarget2d().getY())),
      ((Limelight.getPose().getRotation().getDegrees() * Limelight.getTarget2d().getX()) - (Limelight.getPose().getX() * Limelight.getTarget2d().getRotation().getDegrees())), 
      (new Rotation2d((Limelight.getPose().getX() * Limelight.getTarget2d().getY()) - (Limelight.getPose().getY() * Limelight.getTarget2d().getX()))));
      arcadeDrive(0, PerpendicularPose.getRotation().getDegrees());
      switch(GlobalVars.gamePieceMode)
      {
        case "CONE":
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.CONE_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.CONE_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
        case "CUBE":
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.CUBE_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.CUBE_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
      }
      arcadeDrive(0.0, (Limelight.getTarget2d().getRotation().getDegrees() > 0)? (-90): (90));
      switch(GlobalVars.scoreType)
      {
        case 0:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.LOW_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.LOW_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
        case 1:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.MID_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.MID_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
        case 2:
          while(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) != Math.round(DrivebaseConstants.HIGH_SCORE_DISTANCE))
          {
            if(Math.round(Limelight.getTarget2d().getTranslation().getDistance(Limelight.getPose().getTranslation())) > Math.round(DrivebaseConstants.HIGH_SCORE_DISTANCE))
              arcadeDrive(1, 0);
            else
              arcadeDrive((-1), 0);
          }
      }
    }
  }


  public void stop() {
    arcadeDrive(0, 0);
    GlobalVars.sniperMode = false;
  }
 
  @Override
  public void simulationPeriodic() {}

}