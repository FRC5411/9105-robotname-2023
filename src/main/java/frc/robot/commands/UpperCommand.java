
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class UpperCommand extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem armSubsystem;
    private PIDController pidController;

    private int kP = 1;
    private int kI = 1;
    private int kD = 1;
    
    public UpperCommand(ArmSubsystem armSubsystem, double setpoint)
    {
        this.armSubsystem = armSubsystem;
        this.pidController = new PIDController(kP, kI, kD);
        pidController.setSetpoint(setpoint);
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize()
    {
        pidController.reset();
        System.out.println("Command UPPER has started");
    }

    @Override
    public void execute()
    {
        double speed = pidController.calculate(armSubsystem.getArmEncoder());
        armSubsystem.setArmMotorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        armSubsystem.setArmMotorSpeed(0);
        System.out.println("Command UPPER has ended");
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
