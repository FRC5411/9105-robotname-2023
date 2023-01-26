//In Java We Trust

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArcadeCommand;
import frc.robot.commands.ReleaseCommand;
import frc.robot.commands.ScoreHighCommand;
import frc.robot.commands.ScoreLowCommand;
import frc.robot.commands.ScoreMidCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class RobotContainer 
{
  private CommandXboxController controller;

  private DriveSubsystem robotDrive;
  private ArmSubsystem intake;
  
  private SequentialCommandGroup scoreLow;
  private SequentialCommandGroup scoreMedium;
  private SequentialCommandGroup scoreHigh;

  private Trigger aButton;
  private Trigger bButton;
  private Trigger xButton;

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
      
    scoreLow.addCommands(new ScoreLowCommand(intake, 1), new ReleaseCommand(intake));
    scoreMedium.addCommands(new ScoreMidCommand(intake, 1), new ReleaseCommand(intake));
    scoreHigh.addCommands(new ScoreHighCommand(intake, 1), new ReleaseCommand(intake));

    configureBindings();
  }

  private void configureBindings() 
  {
    aButton.onTrue(scoreLow);
    bButton.onTrue(scoreMedium);
    xButton.onTrue(scoreHigh);
  }

  public Command getAutonomousCommand() 
  {
    return null;
  }
}
