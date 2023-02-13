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
import frc.robot.Constants;
import frc.robot.Constants.*;
//import frc.robot.util.Logger;


/** Drive subsystem */
public class DriveSubsystem extends SubsystemBase {


  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftBackMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightBackMotor;


  // private MotorControllerGroup leftMotors;
  // private MotorControllerGroup rightMotors;


  private DifferentialDrive robotDrive;
  private LimelightSubsystem m_vision;


  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;


  private double DEADZONE_VAL;
  private double SNIPER_SPEED;
  private double SPEED;
  private double ROTATION;
  private Pose2d initial_pose;


  private AHRS navX;


  private DifferentialDrivePoseEstimator odometry;


//  Logger dataLogger;
  Timer timer;
  PowerDistribution PDH;


  public DriveSubsystem(LimelightSubsystem vision) {
    /* Robot Drive */
    leftFrontMotor = new CANSparkMax(
      DrivebaseConstants.LF_MOTOR,
      CANSparkMax.MotorType.kBrushless
    );


    leftBackMotor = new CANSparkMax(
      DrivebaseConstants.LB_MOTOR,
      CANSparkMax.MotorType.kBrushless
    );
   
    rightFrontMotor = new CANSparkMax(
      DrivebaseConstants.RF_MOTOR,
      CANSparkMax.MotorType.kBrushless
    );
   
    rightBackMotor = new CANSparkMax(
      DrivebaseConstants.RB_MOTOR,
      CANSparkMax.MotorType.kBrushless
    );


    m_vision = vision;


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


    initial_pose = new Pose2d();


    robotDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);


    DEADZONE_VAL = DrivebaseConstants.DEADZONE;
    SNIPER_SPEED = DrivebaseConstants.SNIPER_SPEED;
    SPEED = 0.95;
    ROTATION = 0.4;


    /* Encoders */
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


    resetEncoders();


    /* Gyro */
    navX = new AHRS(SPI.Port.kMXP);
    zeroHeading();


    /* Odometry */
    odometry = new DifferentialDrivePoseEstimator(AutonoumousConstants.DRIVE_KINEMATICS, navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), initial_pose);


    resetOdometry(getPose());


    /* Logger */
//    dataLogger = new Logger();
    timer = new Timer();
    PDH = new PowerDistribution(Constants.DrivebaseConstants.PDH_PORT, ModuleType.kRev);
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


  /** Auton Command */
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
    if(m_vision.hastarget()) {odometry.addVisionMeasurement(m_vision.getPose(), Timer.getFPGATimestamp());}
    odometry.update(navX.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());


    SmartDashboard.putNumber("Left Encoder Val: ", getLeftEncoderPosition());
    SmartDashboard.putNumber("Right Encoder Val: ", getRightEncoderPosition());
    SmartDashboard.putNumber("Gyro Heading: ", getGyroHeading());
    SmartDashboard.putNumber("Left Motor Temp: ", getLeftMotorTemp());
    SmartDashboard.putNumber("Right Motor Temp: ", getRightMotorTemp());


    if (timer.get() == 1) {
      timer.reset();
      double velocity = (getRightEncoderVelocity() + getLeftEncoderVelocity()) / 2;
/*      dataLogger.logTelemetryData(
        getLeftMotorTemp(),
        getRightMotorTemp(),
        velocity,
        PDH.getVoltage()
        ); */
    }
  }
  /**
  * Position robot for score type.
  * @param ObjectType - The type of object: cone(true), cube(false)
  * @param ScoreType - The type of score: low(0), mid(1), high(2)
  * @param Target_X_Distance - X distance to limelight target
  * @param Target_Y_Distance - Y distance to limelight target
  * @author Cody Washington
  */
  public void position(boolean ObjectType, int ScoreType, double Target_X_Distance, double Target_Y_Distance)
  {
    ScoreType = (ScoreType > 2)? (2): (ScoreType);
    toAngle((Math.atan(Target_Y_Distance/Target_X_Distance) > 0)? (90): (-90));
    toDistance(((ObjectType && ((ScoreType == 1)? (true): (false)||(ScoreType == 2)? (true): (false)))? (Target_X_Distance-=18.5): (Target_X_Distance)));
    toAngle((Math.atan(Target_Y_Distance/Target_X_Distance) > 0)? (-180): (180));
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
        arcadeDrive(0.0,((navX.getAngle() - Angle)/(StartingYaw + Angle)), false);
      //Negative turn
      else
        arcadeDrive(0.0,(-(navX.getAngle() - Angle)/(StartingYaw + Angle)), false);
    }
  }








  /**
  * Position robot to new distance
  * @param Distance - Distance to move forwards
  * @author Cody Washington
  */
  public void toDistance(double Distance)
  {
    while(Math.round(Distance) != Math.round(AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR / rightBackMotor.getEncoder().getVelocity()))
    {
      //Positive translation
      if(Math.round(Distance) < Math.round(AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR / rightBackMotor.getEncoder().getVelocity()))
        arcadeDrive(Math.round((AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR / rightBackMotor.getEncoder().getVelocity())/Distance),0.0,false);
      //Negative translation
      else
        arcadeDrive(-Math.round((AutonoumousConstants.LINEAR_DIST_CONVERSION_FACTOR / rightBackMotor.getEncoder().getVelocity())/Distance), 0.0, false);
    }
  }


  public void stop() {
    arcadeDrive(0, 0, false);
  }
 
  @Override
  public void simulationPeriodic() {}
}