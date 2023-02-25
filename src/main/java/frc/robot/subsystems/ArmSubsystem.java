
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.GlobalVars;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax biscep;
    private RelativeEncoder biscepEncoder;
    private SparkMaxAlternateEncoder.Type altEncoderType;  
    private SparkMaxPIDController biscepPID;
    private double setpoint;

    public ArmSubsystem() {
        biscep = new CANSparkMax(ArmConstants.ARM_MOTOR_CANID, MotorType.kBrushless); 

        biscep.setIdleMode(IdleMode.kBrake);

        altEncoderType = SparkMaxAlternateEncoder.Type.kQuadrature;

        biscepEncoder = biscep.getAlternateEncoder(altEncoderType, 8192);

        biscepEncoder.setPositionConversionFactor(360);
        biscepEncoder.setVelocityConversionFactor(360);

        biscep.setSmartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);

        biscepPID = biscep.getPIDController();

        // TO-DO: Adjust soft limits for testing
        /* 
        biscep.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        biscep.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

        biscep.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, 30);
        biscep.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);
        */

        // TO-DO: Tune this so it doesn't kill us
        configPID(0.031219, 0, 0.014033, 0.1, 0.1, 0, biscepEncoder, biscepPID);
    }

    public void setArm(double speed) {
        if (GlobalVars.armSniperMode) {
            speed *= 0.2;
        }

        biscep.set(speed);
    }

    public void armUp() {
        biscep.set(0.5);
    }

    public void armDown() {
        biscep.set(-0.5);
    }

    public void posArm(double angle) {
        setpoint = angle;
        biscepPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }

    public void lowArmScore(String mode) {
        if(mode == "CONE") {
            posArm(ArmConstants.LOW_CONE_ANG);
        }
        if(mode == "CUBE") {
            posArm(ArmConstants.LOW_CUBE_ANG);
        }
    }

    public void highArmScore(String mode) {        
        if (mode == "CONE") {
            posArm(ArmConstants.HIGH_CONE_ANG);
        }
        if (mode == "CUBE") {
            posArm(ArmConstants.HIGH_CUBE_ANG);
        }
    }
   
    public void midArmScore(String mode) {
        if (mode == "CONE") {
            posArm(ArmConstants.MID_CONE_ANG);
        }
        if (mode == "CUBE") {
            posArm(ArmConstants.MID_CUBE_ANG);
        }
    }

    public void idleArmScore() {
        posArm(ArmConstants.HOLD);
    }

    public void front() {
        posArm(ArmConstants.FRONT);
    }

    public void straight() {
        posArm(ArmConstants.STRAIGHT);
    }

    public void fetch(String Mode) {
        if (Mode == "CONE") {
            posArm(ArmConstants.FETCH_CONE_ANG);
        }
        if (Mode == "CUBE") {
            posArm(ArmConstants.FETCH_CUBE_ANG);
        }
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
        SmartDashboard.putNumber("Setpoint", setpoint);
    }
   
    @Override  public void simulationPeriodic() {}

    public void configPID(double kp, double kd, double FF, double maxV, double maxA, int profile, RelativeEncoder encoder, SparkMaxPIDController controller) {
        controller.setP(kp, profile);
        controller.setD(kd, profile);
        controller.setFF(FF, profile);
        controller.setSmartMotionMaxAccel(maxA, profile);
        controller.setSmartMotionMaxVelocity(maxV, profile);
        controller.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, profile);
        controller.setFeedbackDevice(encoder);
    }
}
