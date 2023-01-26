
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase 
{  
  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftBackMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightBackMotor;

  private MotorControllerGroup leftMotors;
  private MotorControllerGroup rightMotors;

  private DifferentialDrive robotDrive;

  public DriveSubsystem()
  {
    leftFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.leftFrontMotor, 
    CANSparkMaxLowLevel.MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(Constants.DrivetrainConstants.leftBackMotor, 
    CANSparkMaxLowLevel.MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.rightFrontMotor, 
    CANSparkMaxLowLevel.MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(Constants.DrivetrainConstants.rightBackMotor, 
    CANSparkMaxLowLevel.MotorType.kBrushless);

    leftFrontMotor.setIdleMode(IdleMode.kBrake);
    leftBackMotor.setIdleMode(IdleMode.kBrake);
    rightFrontMotor.setIdleMode(IdleMode.kBrake);
    rightBackMotor.setIdleMode(IdleMode.kBrake);

    leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
    rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);

    leftMotors.setInverted(true);

    robotDrive = new DifferentialDrive(leftMotors, rightMotors);
  }

  public void arcadeDrive(double speed, double rotation)
  {
    speed = speed * 0.7;
    robotDrive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
