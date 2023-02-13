
package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.kauailabs.navx.frc.AHRS;

// This command self=balances on the charging station using gyroscope pitch as feedback
public class AutoEngageCommand extends CommandBase {

  private DriveSubsystem robotDrive;
  private AHRS navX;

  private double errorPitch;
  private double errorYaw;
  private double currentAnglePitch;
  private double currentAngleYaw;
  private double drivePowerPitch;
  private double drivePowerYaw;

  /** Command to use Gyro data to resist the tip angle from the beam - to stabalize and balanace */
  public AutoEngageCommand(DriveSubsystem parameterSubsystem, AHRS parameternavX) {
    this.robotDrive = parameterSubsystem;
    this.navX = parameternavX;
    addRequirements(parameterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Uncomment the line below this to simulate the gyroscope axis with a controller joystick
    // Double currentAngle = -1 * Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
    this.currentAnglePitch = navX.getPitch();
    this.currentAngleYaw = navX.getYaw(); 

    errorPitch = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAnglePitch;
    errorYaw = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAngleYaw;
    drivePowerPitch = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * errorPitch, 1);
    drivePowerYaw = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * errorYaw, 1);

    // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
    if (drivePowerYaw < 0) {
      drivePowerYaw *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }
    if (drivePowerPitch < 0) {
      drivePowerPitch *= Constants.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
    }

    // Limit the max power
    if (Math.abs(drivePowerPitch) > 0.4) {
      drivePowerPitch = Math.copySign(0.4, drivePowerPitch);
    }

    if (Math.abs(drivePowerYaw) > 0.4) {
      drivePowerYaw = Math.copySign(0.4, drivePowerYaw);
    }

    robotDrive.arcadeDrive(drivePowerPitch, currentAngleYaw, true);
    
    // Debugging Print Statments
    System.out.println("Current Angle: " + currentAngleYaw);
    System.out.println("Current Angle: " + currentAnglePitch);
    System.out.println("Error Pitch " + errorPitch);
    System.out.println("Drive Power: " + drivePowerPitch);
    System.out.println("Error Yaw " + errorYaw);
    System.out.println("Drive Power Yaw: " + drivePowerYaw);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    robotDrive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(errorPitch) < Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the specified threshold of being 'flat' (gyroscope pitch of 0 degrees)
  }
}