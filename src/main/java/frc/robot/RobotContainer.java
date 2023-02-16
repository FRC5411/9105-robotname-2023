//In Java We Trust

/*
 * TO-DO: Fix sniper mode not working
 */

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
 import edu.wpi.first.wpilibj2.command.Commands;
 import frc.robot.commands.ArcadeCommand;
 // import frc.robot.subsystems.LimelightSubsystem;
 
 public class RobotContainer {
 
   private CommandXboxController controller;

   private DriveSubsystem robotDrive;
   private LEDSubsystem LEDs;
   private LimelightSubsystem Limelight;
   private boolean sniperMode;
 
   private ArcadeCommand sniperCommand;
 
   Trigger LT;
   Trigger aButton;
   Trigger xButton;
   Trigger yButton;
 
   Debouncer debouncer;
 
   SendableChooser <Command> autonChooser;
 
   public RobotContainer() {
     controller = new CommandXboxController(Constants.DrivebaseConstants.CONTROLLER_PORT);
     Limelight = new LimelightSubsystem();
     robotDrive = new DriveSubsystem(Limelight);

     // Limelight = new LimelightSubsystem();
     LEDs = new LEDSubsystem();
     sniperMode = false;
 
     LT = controller.leftTrigger(0.1);
     aButton = controller.a();
     xButton = controller.x();
     yButton = controller.y();
 
     debouncer = new Debouncer(4);
 
     robotDrive.setDefaultCommand(new ArcadeCommand(
       () -> controller.getLeftY(),
       () -> controller.getRightX(),
       sniperMode,
       robotDrive
       ));
 
     if (debouncer.calculate(LT.getAsBoolean())) {
       sniperCommand = new ArcadeCommand(
         () -> controller.getLeftY(),
         () -> controller.getRightX(),
         !sniperMode,
         robotDrive);
     }
 
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
     //LT.onTrue(sniperCommand);

     //xButton.toggleOnTrue(new InstantCommand(LEDs::setBlue, LEDs));
     //yButton.toggleOnTrue(new InstantCommand(LEDs::setRed, LEDs)); 

     //TODO: APRIlTAG AUTO-ALIGNMENT CODE
     aButton.whileTrue(new InstantCommand(() -> robotDrive.autoAlignment(false, 0, Limelight.getYaw(),Limelight.getDistance()), robotDrive));

   }
 
   public Command getAutonomousCommand() {
    
     return null;
   }
 }
 