
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SniperCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private DoubleSupplier speed;
    private DoubleSupplier rotation;
    private DriveSubsystem robotDrive;
    private boolean sniperMode;
    
    public SniperCommand(DoubleSupplier speed, DoubleSupplier rotation, boolean sniperMode, DriveSubsystem robotDrive) {

        this.speed = speed;
        this.rotation = rotation;
        this.robotDrive = robotDrive;
        this.sniperMode = true;
        addRequirements(robotDrive);
    }

  @Override
  public void initialize() {

    System.out.println("Command SNIPER has started");
  }

  @Override
  public void execute() {

    robotDrive.arcadeDrive(speed.getAsDouble(), rotation.getAsDouble(), sniperMode);
  }

  @Override
  public void end(boolean interrupted) {

    System.out.println("Command SNIPER has ended");
  }

  @Override
  public boolean isFinished() {
    
    return false;
  }
}
