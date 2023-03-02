
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.GlobalVars;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax biscep;
    private Encoder armBoreEncoder;
    private PIDController pid;
    public DigitalInput frontStopSwitch;

    private double kP = 0.031219;
    private double kI = 0;
    private double kD = 0.5;
    private double setpoint;

    public double highestCurrent;
    public double pidCalculation;

    public ArmSubsystem() {
        biscep = new CANSparkMax(ArmConstants.ARM_MOTOR_CANID, MotorType.kBrushless); 

        biscep.setIdleMode(IdleMode.kBrake);
        armBoreEncoder = new Encoder(0, 1);

        biscep.setSmartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);

        pid = new PIDController(kP, kI, kD);

        highestCurrent = GlobalVars.highestAmp;

        frontStopSwitch = new DigitalInput(2);

        SendableRegistry.setName(armBoreEncoder, "ArmSubsystem", "Arm Encoder");
    }

    public void setArm(double speed) {
        if (GlobalVars.armSniperMode) {
            speed *= 0.2;
        }
        biscep.set(speed);
    }

    public void positionArm(double angle) {
        angle *= 22.755; // Must convert pos in angle to approx encoder ticks
        setpoint = angle;

        pid.setTolerance(1);
        
        setArm(-pid.calculate(armBoreEncoder.getDistance(), angle));
    }

    public double getBiscepEncoderPosition() {
        return armBoreEncoder.getDistance() / 22.755;
    }

    public double getArmBoreEncoderDistance() {
        return armBoreEncoder.getDistance();
    }

    public void lowArmScore(String mode) {
        if(mode == "CONE") {
            positionArm(ArmConstants.LOW_CONE_ANG);
        }
        if(mode == "CUBE") {
            positionArm(ArmConstants.LOW_CUBE_ANG);
        }
    }

    public void highArmScore(String mode) {        
        if (mode == "CONE") {
            positionArm(ArmConstants.HIGH_CONE_ANG);
        }
        if (mode == "CUBE") {
            positionArm(ArmConstants.HIGH_CUBE_ANG);
        }
    }
   
    public void midArmScore(String mode) {
        if (mode == "CONE") {
            positionArm(ArmConstants.MID_CONE_ANG);
        }
        if (mode == "CUBE") {
            positionArm(ArmConstants.MID_CUBE_ANG);
        }
    }

    public void idleArmScore() {
        positionArm(ArmConstants.HOLD);
    }

    public void front() {
        positionArm(ArmConstants.FRONT);
    }

    public void straight() {
        positionArm(ArmConstants.STRAIGHT);
    }

    public void fetch(String Mode) {
        if (Mode == "CONE") {
            positionArm(ArmConstants.FETCH_CONE_ANG);
        }
        if (Mode == "CUBE") {
            positionArm(ArmConstants.FETCH_CUBE_ANG);
        }
    }

    public void checkFrontSwitch() {
        if (frontStopSwitch.get()) {
            if (GlobalVars.currentArmSpeed < 0) {
                setArm(0);
            }
        }
    }

    public double getArmCurrent() {
        return biscep.getOutputCurrent();
    }

    public Command midScore(String MODE, ArmSubsystem arm) {
        return new InstantCommand(() -> midArmScore(MODE), arm);
    }

    public Command highScore(String MODE, ArmSubsystem arm) {
        return new InstantCommand(() -> highArmScore(MODE), arm);
    }
   
    public Command lowScore(String MODE, ArmSubsystem arm) {
        return new InstantCommand(() -> lowArmScore(MODE), arm);
    }

    public Command hold(String MODE, ArmSubsystem arm) {
        return new InstantCommand(() -> lowArmScore(MODE), arm);
    }

    public Command front(ArmSubsystem arm) {
        return new InstantCommand(() -> front(), arm);
    }

    public Command straight(ArmSubsystem arm) {
        return new InstantCommand(() -> straight(), arm);
    }

    public Command fetch(String MODE,ArmSubsystem arm) {
        return new InstantCommand(() -> fetch(MODE), arm);
    }

    @Override  
    public void periodic() {
        SmartDashboard.putNumber("Setpoint: ", setpoint);
        SmartDashboard.putNumber("Arm Current: ", getArmCurrent());
        SmartDashboard.putNumber("Arm Encoder: ", getBiscepEncoderPosition());
        SmartDashboard.putNumber("Highest AMP: ", highestCurrent);
        SmartDashboard.putNumber("ARM PID Calculation: ", pidCalculation);

        highestCurrent = (biscep.getOutputCurrent() > highestCurrent) ? biscep.getOutputCurrent() : highestCurrent;
        pidCalculation = GlobalVars.armPIDCalculationOutput;
        checkFrontSwitch();
    }
   
    @Override  public void simulationPeriodic() {}
}
