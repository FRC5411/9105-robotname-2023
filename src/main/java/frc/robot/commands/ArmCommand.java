
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.GlobalVars;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {

    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private ArmSubsystem robotArm;
    private double setpoint;
    private PIDController pid;

     private static double kP = SmartDashboard.getNumber("PID kP", 0.0);
     private static double kI = SmartDashboard.getNumber("PID kI", 0.0);
     private static double kD = SmartDashboard.getNumber("PID kD", 0.0);
 
    /*
     private double kP = 0.031219;
     private double kI = 0.0;
     private double kD = 0.2;
     */

    public ArmCommand(ArmSubsystem robotArm, double setpoint) {
        this.robotArm = robotArm;
        this.setpoint = setpoint;
        pid = new PIDController(kP, kI, kD);

        SendableRegistry.setName(pid, "ArmSubsystem", "PID");
    }

    @Override
    public void initialize() {
      System.out.println("Command ARM ALIGN has started");
      System.out.println("KP: " + kP + "\nKI: " + kI + "KD: " + kD);

      pid.setTolerance(1);
    }
  
    @Override
    public void execute() {
        double calc = pid.calculate(-robotArm.getBiscepEncoderPosition(), setpoint);
        robotArm.setArm(-calc);
        GlobalVars.armPIDCalculationOutput = calc;
        GlobalVars.currentArmSpeed = calc;
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
