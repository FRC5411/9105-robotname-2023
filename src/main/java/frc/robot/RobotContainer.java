//In Java We Trust

/*
 * TO-DO: Fix commands
 */

 package frc.robot;

 import com.pathplanner.lib.PathConstraints;
 import com.pathplanner.lib.PathPlanner;
 import com.pathplanner.lib.PathPlannerTrajectory;

 import edu.wpi.first.wpilibj.GenericHID;
 import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
 import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.InstantCommand;
 import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
 import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 import edu.wpi.first.wpilibj2.command.button.Trigger;
 import frc.robot.subsystems.*;
 import frc.robot.Constants.ArmConstants;
 import frc.robot.Constants.AutonoumousConstants;
 import frc.robot.Constants.ButtonBoardConstants;
 import frc.robot.Constants.DrivebaseConstants;
 import frc.robot.commands.ArcadeCommand;
 import frc.robot.commands.ArmCommand;
 
 public class RobotContainer {
 
   private CommandXboxController controller;
   GenericHID buttonBoard;

   private DriveSubsystem robotDrive;
   private ArmSubsystem robotArm;
   private IntakeSubsystem robotIntake;
   private LEDSubsystem LEDs;
   private LimelightSubsystem Limelight;
  
   private Trigger LT;
   private Trigger LB;
   private Trigger RT;
   private Trigger RB;
   private Trigger aButton;
   private Trigger bButton;
   private Trigger xButton;
   private Trigger yButton;

   private JoystickButton scoreHighButton;
   private JoystickButton scoreMidButton;
   private JoystickButton scoreLowButton;
   private JoystickButton toggleCubeModeButton;
   private JoystickButton toggleConeModeButton;
   private JoystickButton pickupSubstationButton;
   private JoystickButton pickupGroundButton;
   private JoystickButton toggleSniperModeButton;
   private JoystickButton armsToIdleButton;
   private JoystickButton armUpButton;
   private JoystickButton armDownButton;
 
   SendableChooser <Command> autonChooser;
 
   public RobotContainer() {
     LEDs = new LEDSubsystem();
     Limelight = new LimelightSubsystem();
     robotDrive = new DriveSubsystem(Limelight);
     robotArm = new ArmSubsystem();
     robotIntake = new IntakeSubsystem();

     controller = new CommandXboxController(DrivebaseConstants.CONTROLLER_PORT);
     buttonBoard = new GenericHID(1);

     LT = controller.leftTrigger(0.5);
     LB = controller.leftBumper();
     RT = controller.rightTrigger(0.5);
     RB = controller.rightBumper();
     aButton = controller.a();
     bButton = controller.b();
     xButton = controller.x();
     yButton = controller.y();

     scoreHighButton = new JoystickButton(buttonBoard, ButtonBoardConstants.SCORE_HIGH_BUTTON);
     scoreMidButton = new JoystickButton(buttonBoard, ButtonBoardConstants.SCORE_MID_BUTTON);
     scoreLowButton = new JoystickButton(buttonBoard, ButtonBoardConstants.SCORE_LOW_BUTTON);
     toggleConeModeButton = new JoystickButton(buttonBoard, ButtonBoardConstants.TOGGLE_CONE_MODE_BUTTON);
     toggleCubeModeButton =  new JoystickButton(buttonBoard, ButtonBoardConstants.TOGGLE_CONE_MODE_BUTTON);
     pickupGroundButton = new JoystickButton(buttonBoard, ButtonBoardConstants.PICKUP_GROUND_BUTTON);
     pickupSubstationButton = new JoystickButton(buttonBoard, ButtonBoardConstants.PICKUP_SUBSTATION_BUTTON);
     toggleSniperModeButton = new JoystickButton(buttonBoard, ButtonBoardConstants.TOGGLE_SNIPER_MODE_BUTTON);
     armsToIdleButton = new JoystickButton(buttonBoard, ButtonBoardConstants.RETURN_TO_IDLE_BUTTON);
     armUpButton = new JoystickButton(buttonBoard, ButtonBoardConstants.ARM_UP_BUTTON);
     armDownButton = new JoystickButton(buttonBoard, ButtonBoardConstants.ARM_DOWN_BUTTON);

     robotDrive.setDefaultCommand(new ArcadeCommand(
       () -> controller.getLeftY(),
       () -> controller.getRightX(),
       robotDrive
       ));
 
     autonChooser = new SendableChooser<>();
     PathConstraints trajectoryConstraints = new PathConstraints(AutonoumousConstants.DRIVE_VELOCITY, AutonoumousConstants.MAX_ACCELERATION);
     PathPlannerTrajectory mainTrajectory = PathPlanner.loadPath("TestPath.wpilib.json" , trajectoryConstraints);
 
     autonChooser.addOption("Test Path", robotDrive.followPath(
       mainTrajectory,
      true));
 
     Shuffleboard.getTab("Autonomous: ").add(autonChooser);

     configureBindings();
   }
 
   private void configureBindings() {
    
    LT.whileTrue(new InstantCommand( () -> {
      GlobalVars.driveSniperMode = true;
    }));

    LT.whileFalse(new InstantCommand( () -> {
      GlobalVars.driveSniperMode = false;
    }));

    /* 
    RT.whileTrue(new InstantCommand( () -> {
      GlobalVars.armSniperMode = true;
    }));

    RT.whileFalse(new InstantCommand( () -> {
      GlobalVars.armSniperMode = false;
    }));
    */

    LB.onTrue(new InstantCommand( () -> {
      if(robotIntake.getIntakeCurrent() > ArmConstants.GRABBER_MOTOR_CURRENT_LIMIT) {
        robotArm.setArm(0);
      }
      if (GlobalVars.gamePieceMode.equals("CUBE")) {
        robotIntake.spinout();
      }
      else {
        robotIntake.spinin();
      }
    }));

    LB.onFalse(new InstantCommand( () -> {
      robotIntake.setSpin(0);
    }));

    RB.onTrue(new InstantCommand( () -> {
      if(robotIntake.getIntakeCurrent() > ArmConstants.GRABBER_MOTOR_CURRENT_LIMIT) {
        robotArm.setArm(0);
      }
      if (GlobalVars.gamePieceMode.equals("CUBE")) {
        robotIntake.spinin();
      }
      else {
        robotIntake.spinout();
      }
    }));

    RB.onFalse(new InstantCommand( () -> {
      robotIntake.setSpin(0);
    }));

    /*
    yButton.onTrue(new InstantCommand( () -> {
      if (robotArm.getArmCurrent() > ArmConstants.ARM_MOTOR_CURRENT_LIMIT) {
        robotArm.setArm(0);
      }
      robotArm.setArm(ArmConstants.ARM_MOTOR_SPEED);
      GlobalVars.currentArmSpeed = 1;
    }));

    yButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    aButton.onTrue(new InstantCommand( () -> {
      if (robotArm.getArmCurrent() > ArmConstants.ARM_MOTOR_CURRENT_LIMIT) {
        robotArm.setArm(0);
      }
      robotArm.setArm(-ArmConstants.ARM_MOTOR_SPEED);
      GlobalVars.currentArmSpeed = -1;
    }));

    aButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));
    */

    /* LEAVE HERE FOR TESTING PURPOSES
    bButton.whileTrue(new ArmCommand(robotArm, 217.2));

    bButton.onFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));
    */
    
    scoreHighButton.whileTrue(new ArmCommand(robotArm, GlobalVars.HIGH_ANG));
    
    scoreHighButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    scoreMidButton.whileTrue(new ArmCommand(robotArm, GlobalVars.MID_ANG));
    
    scoreMidButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    scoreLowButton.whileTrue(new ArmCommand(robotArm, GlobalVars.LOW_ANG));
    
    scoreLowButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    toggleCubeModeButton.toggleOnTrue(new InstantCommand( () -> {
      GlobalVars.gamePieceMode = "CUBE";
    }));

    toggleCubeModeButton.toggleOnFalse(new InstantCommand( () -> {
      GlobalVars.gamePieceMode = "CONE";
    }));

    pickupGroundButton.whileTrue(new ArmCommand(robotArm, GlobalVars.FETCH_GRND));

    pickupGroundButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    pickupSubstationButton.whileTrue(new ArmCommand(robotArm, GlobalVars.FETCH_SUBSTATION));

    pickupSubstationButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    toggleSniperModeButton.toggleOnTrue(new InstantCommand( () -> {
      GlobalVars.armSniperMode = true;
    }));

    toggleSniperModeButton.toggleOnFalse(new InstantCommand( () -> {
      GlobalVars.armSniperMode = false;
    }));

    armsToIdleButton.whileTrue(new ArmCommand(robotArm, ArmConstants.IDLE));

    armsToIdleButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    armUpButton.whileTrue(new InstantCommand( () -> {
      if (robotArm.getArmCurrent() > ArmConstants.ARM_MOTOR_CURRENT_LIMIT) {
        robotArm.setArm(0);
      }
      robotArm.setArm(ArmConstants.ARM_MOTOR_SPEED);
      GlobalVars.currentArmSpeed = 1;      
    }));

    armUpButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

    armDownButton.whileTrue(new InstantCommand( () -> {
      if (robotArm.getArmCurrent() > ArmConstants.ARM_MOTOR_CURRENT_LIMIT) {
        robotArm.setArm(0);
      }
      robotArm.setArm(-ArmConstants.ARM_MOTOR_SPEED);
      GlobalVars.currentArmSpeed = -1;      
    }));

    armDownButton.whileFalse(new InstantCommand( () -> {
      robotArm.setArm(0);
    }));

   }
 
   public Command getAutonomousCommand() {
    
     return null;
   }
 }
 