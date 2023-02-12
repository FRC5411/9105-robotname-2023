//In Java We Trust

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.commands.ArcadeCommand;

public class RobotContainer {

  private CommandXboxController controller;

  private DriveSubsystem robotDrive;
  private LEDSubsystem LEDs;
  private boolean sniperMode;

  private ArcadeCommand sniperCommand;

  Trigger LT;
  Trigger xButton;
  Trigger yButton;

  Debouncer debouncer;

  SendableChooser <Command> autonChooser;

  public RobotContainer() {
    /* Default Drive & Controller */
    controller = new CommandXboxController(Constants.DrivebaseConstants.CONTROLLER_PORT);
    robotDrive = new DriveSubsystem();
    LEDs = new LEDSubsystem();
    sniperMode = false;

    LT = controller.leftTrigger(0.1);
    xButton = controller.x();
    yButton = controller.y();

    debouncer = new Debouncer(4);

    robotDrive.setDefaultCommand(new ArcadeCommand(
      () -> controller.getLeftY(), 
      () -> controller.getRightX(), 
      sniperMode,
      robotDrive
      ));

    /* Sniper command */
    if (debouncer.calculate(LT.getAsBoolean())) {
      sniperCommand = new ArcadeCommand(
        () -> controller.getLeftY(), 
        () -> controller.getRightX(), 
        !sniperMode, 
        robotDrive);
    }

    /* Auton Button */
    autonChooser = new SendableChooser<>();

    PathConstraints trajectoryConstraints = new PathConstraints(Constants.AutonoumousConstants.DRIVE_VELOCITY, Constants.AutonoumousConstants.MAX_ACCELERATION);
    PathPlannerTrajectory mainTrajectory = PathPlanner.loadPath("TestPath.wpilib.json" , trajectoryConstraints);


    autonChooser.addOption("Test Path", robotDrive.followPath(
      mainTrajectory,
     true));

    Shuffleboard.getTab("Autonomous: ").add(autonChooser);

    SmartDashboard.putNumber("Left Joystick Y: ", controller.getLeftY());
    SmartDashboard.putNumber("Right Joystick X: ", controller.getRightX());
    
    configureBindings();
  }

  private void configureBindings() {
    
    // NOTE: fix button bindings
    LT.onTrue(sniperCommand);

    /* LEDs */
    //xButton.toggleOnTrue(new InstantCommand(LEDs::setBlue, LEDs));
    //yButton.toggleOnTrue(new InstantCommand(LEDs::setRed, LEDs));
  }

  public Command getAutonomousCommand() {
    
    return null;
  }
}
