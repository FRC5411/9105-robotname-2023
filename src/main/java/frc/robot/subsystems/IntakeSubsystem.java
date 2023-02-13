
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase{

  private CANSparkMax grabber;

  public IntakeSubsystem() {
    grabber = new CANSparkMax(
      ArmConstants.GRABBER_MOTOR_CANID, 
      MotorType.kBrushless
    );
  }

  public void spinin() {
    grabber.set(0.5);
  }

  public void spinout() {
    grabber.set(-0.5);
  }

  public void spinoff() {
    grabber.set(0);
  }

  public InstantCommand inspin(IntakeSubsystem intake) {
    return new InstantCommand(() -> spinin(), intake);
  }

  public InstantCommand outspin(IntakeSubsystem intake) {
    return new InstantCommand(() -> spinout(), intake);
  }

  public InstantCommand offspin(IntakeSubsystem intake) {
    return new InstantCommand(() -> spinoff(), intake);
  }
}