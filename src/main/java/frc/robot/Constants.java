
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  static double convertToLinDist(double GEAR_RATIO, double WHEEL_RADIUS){
    return Units.inchesToMeters (
        1 / GEAR_RATIO
        * 2 * Math.PI 
        * Units.inchesToMeters(WHEEL_RADIUS) // Circumfrence
        * 10
        );
  }

  public static class DrivebaseConstants {
    public static final int LF_MOTOR = 1;
    public static final int LB_MOTOR = 2;
    public static final int RF_MOTOR = 3;
    public static final int RB_MOTOR = 4;

    public static final int CONTROLLER_PORT = 0;

    public static final int MOTOR_AMP_LIMIT = 80;
  }
  
  public static class LEDsConstants {
    public static final int LED_PORT = 0;
  }

  public static class AutonoumousConstants {
    // Note: Update values accordingly
    public static final double VOLTS = 0;
    public static final double VOLT_SECONDS_PER_METER = 0;
    public static final double VOLT_SECONDS_SQUARED_PER_METER = 0;
    public static final double DRIVE_VELOCITY = 0;
    
    // Note: Update values accordingly
    public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(6); // Horizontal dist between 2 wheels (Meters)
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new 
      DifferentialDriveKinematics(TRACK_WIDTH_METERS);
    
    public static final double MAX_SPEED_METERS_PER_SECOND = 3; // Leave as is
    public static final double MAX_ACCELERATION = 3; // Leave as is
    
    public static final double RAMSETE_B = 2; // Leave as is
    public static final double RAMSETE_ZETA = 0.7; // Leave as is
    
    // Note: update ratios accordingly
    public static final double GEAR_RATIO = 7.9; // Gearbox ratio
    public static final double WHEEL_RADIUS = 3; // Radius of wheel in inches
    
    public static final double LINEAR_DIST_CONVERSION_FACTOR = 
      (convertToLinDist(GEAR_RATIO, WHEEL_RADIUS)); // Converts ticks to metres
  }

  public static class IntakeConstants {
    public static final int ARM_MOTOR = 5;
    public static final int GRABBER_MOTOR = 6;
  }
}
