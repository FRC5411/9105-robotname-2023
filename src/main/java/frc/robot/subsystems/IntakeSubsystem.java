package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class IntakeSubsystem extends SubsystemBase{
    CANSparkMax spinner;
    public IntakeSubsystem() {
        spinner = new CANSparkMax(25, MotorType.kBrushless);
    }

  public void spinin() {
    spinner.set(0.5);
  }

  public void spinout() {
    spinner.set(-0.5);
  }

  public void spinoff() {
    spinner.set(0);
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