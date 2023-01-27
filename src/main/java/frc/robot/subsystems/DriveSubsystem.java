
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel;
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
    CANSparkMax.MotorType.kBrushless);
    leftBackMotor = new CANSparkMax(Constants.DrivetrainConstants.leftBackMotor, 
    CANSparkMax.MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(Constants.DrivetrainConstants.rightFrontMotor, 
    CANSparkMax.MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(Constants.DrivetrainConstants.rightBackMotor, 
    CANSparkMax.MotorType.kBrushless);

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
    if (speed < 0.1 && speed > -0.1)
    {
      speed = 0;
    }
    if (rotation < 0.1 && rotation > -0.1)
    {
      rotation = 0;
    }
    speed = speed * 0.7;
    robotDrive.arcadeDrive(speed, rotation);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
