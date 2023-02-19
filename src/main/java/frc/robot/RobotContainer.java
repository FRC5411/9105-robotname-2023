//In Java We Trust

 package frc.robot;

 import com.pathplanner.lib.PathConstraints;
 import com.pathplanner.lib.PathPlanner;
 import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
 import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
 import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
 import frc.robot.subsystems.*;
import frc.robot.Constants.ButtonBoardConstants;
import frc.robot.commands.ArcadeCommand;
 
 public class RobotContainer {
 
   private CommandXboxController controller;
   GenericHID buttonBoard;

   private DriveSubsystem robotDrive;
   private ArmSubsystem robotArm;
   private IntakeSubsystem robotIntake;
   private LEDSubsystem LEDs;
   private LimelightSubsystem Limelight;
  
   private Trigger LT;
   private Trigger aButton;
   private Trigger bButton;
   private Trigger xButton;
   private Trigger yButton;

   JoystickButton scoreHighButton;
   JoystickButton scoreMidButton;
   JoystickButton scoreLowButton;
   JoystickButton toggleCubeModeButton;
   JoystickButton toggleConeModeButton;
   JoystickButton pickupSubstationButton;
   JoystickButton pickupGroundButton;
   JoystickButton toggleAlignmentButton;
   JoystickButton toggleSniperModeButton;
   JoystickButton armsToIdleButton;
   JoystickButton armsExtendButton;
   JoystickButton armUpButton;
   JoystickButton armDownButton;
   JoystickButton cancelCurrentCommandButton;
 
   SendableChooser <Command> autonChooser;
 
   public RobotContainer() {
     LEDs = new LEDSubsystem();
     Limelight = new LimelightSubsystem();
     robotDrive = new DriveSubsystem(Limelight);
     robotArm = new ArmSubsystem();
     robotIntake = new IntakeSubsystem();

     controller = new CommandXboxController(Constants.DrivebaseConstants.CONTROLLER_PORT);

     LT = controller.leftTrigger(0.5);
     aButton = controller.a();
     bButton = controller.b();
     xButton = controller.x();
     yButton = controller.y();

     scoreHighButton = new JoystickButton(buttonBoard, ButtonBoardConstants.SCORE_HIGH_BUTTON);
     scoreMidButton = new JoystickButton(buttonBoard, ButtonBoardConstants.SCORE_MID_BUTTON);
     scoreLowButton = new JoystickButton(buttonBoard, ButtonBoardConstants.SCORE_LOW_BUTTON);
     toggleCubeModeButton = new JoystickButton(buttonBoard, ButtonBoardConstants.TOGGLE_CUBE_MODE_BUTTON);
     toggleConeModeButton = new JoystickButton(buttonBoard, ButtonBoardConstants.TOGGLE_CONE_MODE_BUTTON);
     pickupSubstationButton = new JoystickButton(buttonBoard, ButtonBoardConstants.PICKUP_SUBSTATION_BUTTON);
     pickupGroundButton = new JoystickButton(buttonBoard, ButtonBoardConstants.PICKUP_GROUND_BUTTON);
     toggleAlignmentButton = new JoystickButton(buttonBoard, ButtonBoardConstants.TOGGLE_ALIGNMENT_BUTTON);
     toggleSniperModeButton = new JoystickButton(buttonBoard, ButtonBoardConstants.TOGGLE_SNIPER_MODE_BUTTON);
     armsToIdleButton = new JoystickButton(buttonBoard, ButtonBoardConstants.RETURN_TO_IDLE_BUTTON);
     armsExtendButton = new JoystickButton(buttonBoard, ButtonBoardConstants.FULL_EXTENSION_BUTTON);
     armUpButton = new JoystickButton(buttonBoard, ButtonBoardConstants.ARM_UP_BUTTON);
     armDownButton = new JoystickButton(buttonBoard, ButtonBoardConstants.ARM_DOWN_BUTTON);
     cancelCurrentCommandButton = new JoystickButton(buttonBoard, ButtonBoardConstants.CANCEL_CURRENT_COMMAND_BUTTON);

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
    LT.whileTrue(new InstantCommand( () -> {
      GlobalVars.sniperMode = true;
    }));

    LT.whileFalse(new InstantCommand( () -> {
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

    //#region Button board

    /* 
    scoreHighButton.onTrue(new InstantCommand( () -> {
      robotArm.highArmScore(GlobalVars.gamePieceMode);
      GlobalVars.scoreType = 2;
    }));

    scoreHighButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    scoreMidButton.onTrue(new InstantCommand( () -> {
      robotArm.midArmScore(GlobalVars.gamePieceMode);
      GlobalVars.scoreType = 1;
    }));

    scoreMidButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    scoreLowButton.onTrue(new InstantCommand( () -> {
      robotArm.lowArmScore(GlobalVars.gamePieceMode);
      GlobalVars.scoreType = 0;
    }));

    scoreLowButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    toggleCubeModeButton.toggleOnTrue(new InstantCommand( () -> {
      GlobalVars.gamePieceMode = "CUBE";
    }));

    toggleCubeModeButton.toggleOnFalse(new InstantCommand());

    toggleConeModeButton.toggleOnTrue(new InstantCommand( () -> {
      GlobalVars.gamePieceMode = "CONE";
    }));

    toggleConeModeButton.toggleOnFalse(new InstantCommand());

    pickupSubstationButton.onTrue(new InstantCommand( () -> {
      robotArm.fetch(GlobalVars.gamePieceMode);
    }));

    pickupSubstationButton.onFalse(new InstantCommand());

    pickupGroundButton.onTrue(new InstantCommand( () -> {
      robotArm.front();
    }));

    pickupGroundButton.onFalse(new InstantCommand());

    // TO-DO: Get target angle and target distance
    toggleAlignmentButton.toggleOnTrue(new InstantCommand( () -> {
      boolean piece = (GlobalVars.gamePieceMode.equals("CONE")) ? true : false;
      robotDrive.autoAlignment(false, 0, 0, 0);
    }));

    toggleAlignmentButton.toggleOnFalse(new InstantCommand());

    toggleSniperModeButton.toggleOnTrue(new InstantCommand( () -> {
      GlobalVars.sniperMode = !GlobalVars.sniperMode;
    }));

    toggleSniperModeButton.toggleOnFalse(new InstantCommand());

    armsToIdleButton.onTrue(new InstantCommand( () -> {
      robotArm.idleArmScore();
    }));

    armsToIdleButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    armsExtendButton.onTrue(new InstantCommand( () -> {
      robotArm.front();
    }));

    armsExtendButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    armUpButton.whileTrue(new InstantCommand( () -> {
      robotArm.setArm(0.5);
    }));

    armUpButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    armDownButton.whileTrue(new InstantCommand( () -> {
      robotArm.setArm(-0.5);
    }));

    armDownButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    cancelCurrentCommandButton.onTrue(new InstantCommand( () -> {
      CommandScheduler.getInstance().cancelAll();
    }));
    */
    
    //#endregion
   }
 
   public Command getAutonomousCommand() {
    
     return null;
   }
 }
 