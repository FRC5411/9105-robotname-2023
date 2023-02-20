
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax biscep;
    private RelativeEncoder biscepEncoder;  
    private SparkMaxPIDController biscepPID;  

    public ArmSubsystem() {
        biscep = new CANSparkMax(ArmConstants.ARM_MOTOR_CANID, MotorType.kBrushless); 

        biscep.setIdleMode(IdleMode.kBrake);

        biscepEncoder = biscep.getEncoder();

        biscepEncoder.setPositionConversionFactor(360);
        biscepEncoder.setVelocityConversionFactor(360);

        biscep.setSmartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
        
        biscep.enableSoftLimit(SoftLimitDirection.kForward, true);
        biscep.enableSoftLimit(SoftLimitDirection.kReverse, true);
        biscep.setSoftLimit(SoftLimitDirection.kForward, 15);
        biscep.setSoftLimit(SoftLimitDirection.kReverse, 0);

        biscepPID = biscep.getPIDController();

        configPID(0, 0, 0, 0, 0, 0, biscepEncoder, biscepPID);
    }

    public void setArm(double speed) {
        biscep.set(speed);
    }

    public void armUp() {
        biscep.set(0.5);
    }

    public void armDown() {
        biscep.set(-0.5);
    }

    public void posArm(double angle) {
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

    @Override  public void periodic() {}
   
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
