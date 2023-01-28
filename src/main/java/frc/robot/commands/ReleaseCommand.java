
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ReleaseCommand extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem armSubsystem;

    public ReleaseCommand(ArmSubsystem armSubsystem)
    {
        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() 
    {
        System.out.println("Command RELEASE has started");
    }
  
    @Override
    public void execute() 
    {
        armSubsystem.setHandMotorSped(1);
    }
  
    @Override
    public void end(boolean interrupted) 
    {
      armSubsystem.setHandMotorSped(0);
      System.out.println("Command RELEASE has stopped");
    }
  
    @Override
    public boolean isFinished() 
    {
      return false;
    }
}
