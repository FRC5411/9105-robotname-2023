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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 import edu.wpi.first.wpilibj2.command.button.Trigger;
 import frc.robot.subsystems.*;
 import frc.robot.commands.ArcadeCommand;
 
 public class RobotContainer {
 
   private CommandXboxController controller;

   private DriveSubsystem robotDrive;
   private ArmSubsystem robotArm;
   private IntakeSubsystem robotIntake;
   private LEDSubsystem LEDs;
   private LimelightSubsystem Limelight;
  
   Trigger LT;
   Trigger aButton;
   Trigger bButton;
   Trigger xButton;
   Trigger yButton;
 
   Debouncer debouncer;
 
   SendableChooser <Command> autonChooser;
 
   public RobotContainer() {
     controller = new CommandXboxController(Constants.DrivebaseConstants.CONTROLLER_PORT);
     Limelight = new LimelightSubsystem();
     robotDrive = new DriveSubsystem(Limelight);
     robotArm = new ArmSubsystem();
     robotIntake = new IntakeSubsystem();

     LEDs = new LEDSubsystem();
 
     LT = controller.leftTrigger(0.5);
     aButton = controller.a();
     bButton = controller.b();
     xButton = controller.x();
     yButton = controller.y();
  
     robotDrive.setDefaultCommand(new ArcadeCommand(
       () -> controller.getLeftY(),
       () -> controller.getRightX(),
       robotDrive
       ));
 
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
    LT.onTrue(new InstantCommand( () -> {
      GlobalVars.sniperMode = true;
    }));

    LT.onFalse(new InstantCommand( () -> {
      GlobalVars.sniperMode = false;
    }));

    yButton.onTrue(new InstantCommand( () -> {
      robotArm.armUp();
    }));

    yButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    aButton.onTrue(new InstantCommand( () -> {
      robotArm.armDown();
    }));

    aButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    xButton.onTrue(new InstantCommand( () -> {
      robotIntake.spinin();
    }));

    xButton.onFalse(new InstantCommand( () -> {
      robotIntake.setSpin(0);
    }));

    bButton.onTrue(new InstantCommand( () -> {
      robotIntake.spinout();
    }));

    bButton.onFalse(new InstantCommand( () -> {
      robotIntake.setSpin(0);
    }));

    
    
   }
 
   public Command getAutonomousCommand() {
    
     return null;
   }
 }
 