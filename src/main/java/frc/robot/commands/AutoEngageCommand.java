/*
 This code calculates a virtual spring force based on the current pitch angle from the gyro and applies it to the drivetrain 
 using arcade drive. The virtual spring force is calculated as -kSpringConstant * angle - kDampingCoefficient * m_gyro.getRate(), 
 where kSpringConstant and kDampingCoefficient are constants that control the strength and damping of the virtual spring. 
 The maximum power that can be applied to the motors is limited to kMaxPower. The current pitch angle is also displayed on the 
 SmartDashboard for debugging purposes. The command finishes when the robot's pitch angle is within the acceptable range of kTolerance.
 */

package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class AutoEngageCommand extends CommandBase {
  
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  private final double kSpringConstant = 0.1; // virtual spring constant
  private final double kDampingCoefficient = 0.05; // virtual damping coefficient
  private final double kMaxPower = 0.5; // maximum power to apply to motors
  private final double kTolerance = 2.0; // acceptable range in degrees from the desired angle

  private final AHRS m_gyro;
  private final DriveSubsystem m_drivetrain;

  public AutoEngageCommand(AHRS gyro, DriveSubsystem drivetrain) {
      m_gyro = gyro;
      m_drivetrain = drivetrain;

      addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
      m_drivetrain.stop(); // ensure motors are stopped at the beginning
  }

  @Override
  public void execute() {
      double angle = m_gyro.getPitch(); // get current pitch angle from gyro
      double power = -kSpringConstant * angle - kDampingCoefficient * m_gyro.getRate(); // calculate virtual spring force
      power = Math.min(Math.max(power, -kMaxPower), kMaxPower); // limit power to a maximum value
      m_drivetrain.arcadeDrive(power, 0.0); // apply power to drivetrain

      SmartDashboard.putNumber("Pitch Angle", angle); // display current angle on dashboard
  }

  @Override
  public boolean isFinished() {
      double angle = m_gyro.getPitch();
      return Math.abs(angle) < kTolerance; // command is finished when the robot is within acceptable range of the desired angle
  }
}
