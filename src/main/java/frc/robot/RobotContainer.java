//In Java We Trust

//IMPORTANT NOTE: Comment out score sequential command initialisation and button config before deploying code

package frc.robot;

import java.io.IOException;
import java.nio.file.FileSystem;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.ReleaseCommand;
import frc.robot.commands.ScoreLowCommand;
import frc.robot.commands.ScoreMidCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.*;

public class RobotContainer 
{
  private CommandXboxController controller;

  private DriveSubsystem robotDrive;
  private ArmSubsystem intake;
  
  private SequentialCommandGroup scoreLow;
  private SequentialCommandGroup scoreMedium;

  private Trigger aButton;
  private Trigger bButton;
  private Trigger xButton;

  SendableChooser <Command> chooser;

  public RobotContainer() 
  {
    controller = new CommandXboxController(Constants.DrivetrainConstants.controllerPort);
    robotDrive = new DriveSubsystem();
    intake = new ArmSubsystem();

    aButton = controller.a();
    bButton = controller.b();
    xButton = controller.x();

    robotDrive.setDefaultCommand(new ArcadeCommand(
      () -> controller.getLeftY(), 
      () -> controller.getRightX(), 
      robotDrive
      ));
      
    //scoreLow.addCommands(new ScoreLowCommand(intake, 1), new ReleaseCommand(intake));
    //scoreMedium.addCommands(new ScoreMidCommand(intake, 1), new ReleaseCommand(intake));

    chooser = new SendableChooser<>();

    //Adds button options
    chooser.addOption("Test Path", loadPathPlannerTrajetoryToRamseteCommand(
      "../../../deploy/pathplanner/generatedJSON/TestPath.wpilib.json",
     true));

    Shuffleboard.getTab("Autonomous: ").add(chooser);

    configureBindings();
  }

  public Command loadPathPlannerTrajetoryToRamseteCommand(String filename, boolean resetOdometry)
  {
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(filename);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }catch(IOException exception){
      DriverStation.reportError("Unable to open trajectory: " + filename, exception.getStackTrace());
      System.out.println("Unable to read from file: " + filename);

      //Returns this so robot doesnt do anything
      return new InstantCommand();
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
      trajectory, 
      robotDrive::getPose, 
      new RamseteController(Constants.DriveConstants.kRamseteB, Constants.DriveConstants.kRamseteZeta), 
      new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter), 
      Constants.DriveConstants.kDriveKinematics, 
      robotDrive::getWheelSpeeds, 
      new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0), 
      new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0), 
      robotDrive::setTankDriveVolts, 
      robotDrive);

    //Reset odometry first and then run command
    if (resetOdometry) 
    {
      return new SequentialCommandGroup(
        new InstantCommand(() -> robotDrive.resetOdometry(trajectory.getInitialPose())),ramseteCommand);
    }
    else 
    {
      return ramseteCommand;
    }
  }

  private void configureBindings() 
  {
    //aButton.onTrue(scoreLow);
    //bButton.onTrue(scoreMedium);
  }

  public Command getAutonomousCommand() 
  {
    return null;
  }
}
