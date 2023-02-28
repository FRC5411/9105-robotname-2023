package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private ArmSubsystem robotArm;
    private double setpoint;
    private PIDController pid;
    private double kP = 0.031219;
    private double kI = 0.2;
    private double kD = 0.5;

    public ArmCommand(ArmSubsystem robotArm, double setpoint) {
        this.robotArm = robotArm;
        this.setpoint = setpoint;
        pid = new PIDController(kP, kI, kD);
    }

    @Override
    public void initialize() {
      System.out.println("Command ARM ALIGN has started");

      pid.setTolerance(1);
    }
  
    @Override
    public void execute() {
        double angle = setpoint *= 22.755;
        robotArm.setArm(pid.calculate(-robotArm.getBiscepEncoderPosition(), angle));
    }
  
    @Override
    public void end(boolean interrupted) {
      System.out.println("Command ARM ALIGN has ended");
    }
  
    @Override
    public boolean isFinished() {
      
      return false;
    }
}
