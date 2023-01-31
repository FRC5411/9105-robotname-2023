
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class OrientateCommand extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    DriveSubsystem robotDrive;
    PIDController pidController;
    double setpoint;
    int pitch;

    int kP = 1;
    int kI = 1;
    int kD = 1;

    public OrientateCommand(DriveSubsystem driveSubsystem, double setpoint, int gyroPitch)
    {
        this.robotDrive = driveSubsystem;
        this.pitch = gyroPitch;
        this.setpoint = setpoint;
        pidController = new PIDController(kP, kI, kD);
        pidController.setSetpoint(setpoint);
        addRequirements(robotDrive);
    }

    @Override
    public void initialize()
    {
        pidController.reset();
        pidController.setTolerance(2.5);
        System.out.println("Command ORIENTATE has started");
    }

    @Override
    public void execute()
    {
        double pidCalc = pidController.calculate(pitch, setpoint);
        robotDrive.setLeftMotorSpeed(pidCalc);
        robotDrive.setRightMotorSpeed(pidCalc);
    }

    @Override
    public void end(boolean interrupted)
    {
        System.out.println("Command ORIENTATE has ended");
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
