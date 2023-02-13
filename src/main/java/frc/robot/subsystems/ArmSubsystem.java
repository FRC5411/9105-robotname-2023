package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;


public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax mBiscep;
    private final CANSparkMax mBiscep2;
    private final RelativeEncoder biscepEncoder;  
    private final SparkMaxPIDController biscepPID;  


    public ArmSubsystem() {
        mBiscep = new CANSparkMax(1, MotorType.kBrushless);
        mBiscep2 = new CANSparkMax(1, MotorType.kBrushless);  

        mBiscep.setIdleMode(IdleMode.kBrake);
        mBiscep2.setIdleMode(IdleMode.kBrake);
        mBiscep2.follow(mBiscep);

        biscepEncoder = mBiscep.getEncoder();
        biscepEncoder.setPositionConversionFactor(360);
        biscepEncoder.setVelocityConversionFactor(360);

        biscepPID = mBiscep.getPIDController();

        configPID(0, 0, 0, 0, 0, 0, biscepEncoder, biscepPID);
    }

    public void setArm(double speed) {
        mBiscep.set(speed);
    }


    public void posArm(double angle) {
        biscepPID.setReference(angle, CANSparkMax.ControlType.kPosition);
    }


    public void lowArmScore(String Mode) {
        if(Mode == "CONE") {
            posArm(ARM.LOW_CONE_ANG);
        }
        if(Mode == "CUBE") {
            posArm(ARM.LOW_CUBE_ANG);
        }
    }


    public void highArmScore(String Mode) {        
    if(Mode == "CONE") {
        posArm(ARM.HIGH_CONE_ANG);
    }
    if(Mode == "CUBE") {
        posArm(ARM.HIGH_CUBE_ANG);
    }
    }
   
    public void midArmScore(String Mode) {
    if(Mode == "CONE") {
        posArm(ARM.MID_CONE_ANG);
    }
    if(Mode == "CUBE") {
        posArm(ARM.MID_CUBE_ANG);
    }
    }


    public void idleArmScore() {
        posArm(ARM.HOLD);
    }


    public void front() {
        posArm(ARM.FRONT);
    }


    public void straight() {
        posArm(ARM.STRAIGHT);
    }


    public void fetch(String Mode) {
        if(Mode == "CONE") {
            posArm(ARM.FETCH_CONE_ANG);
        }
        if(Mode == "CUBE") {
            posArm(ARM.FETCH_CUBE_ANG);
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
