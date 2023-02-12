
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
    public static final int LF_MOTOR = 11;
    public static final int LB_MOTOR = 12;
    public static final int RF_MOTOR = 13;
    public static final int RB_MOTOR = 14;

    public static final int CONTROLLER_PORT = 0;

    public static final int MOTOR_AMP_LIMIT = 80;
    public static final int PDH_PORT = 1;
    public static final double DEADZONE = 0.1;
    public static final double SNIPER_SPEED = 0.2;

    //TODO: GET CORRECT DISTANCE VALUES
    public static final double LOW_SCORE_DISTANCE = 1;
    public static final double MID_SCORE_DISTANCE = 1;
    public static final double HIGH_SCORE_DISTANCE = 1;
  }
  
  public static class LEDsConstants {
    public static final int LED_PORT = 0;
    public static final int LED_NUMBER = 60;
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

    public static final int ARM_MOTOR_CURRENT_LIMIT = 60;
    public static final int GRABBER_MOTOR_CURRENT_LIMIT = 30;
  }

  public static class ARM {       
    public static final double LOW_CUBE_ANG = 45.4;
    public static final double MID_CUBE_ANG = 106.6;
    public static final double HIGH_CUBE_ANG = 130.6;
    public static final double FETCH_CONE_ANG = 116.3;
    public static final double LOW_CONE_ANG = 52.2;
    public static final double MID_CONE_ANG = 93.5;
    public static final double HIGH_CONE_ANG = 116.6;
    public static final double FETCH_CUBE_ANG = 125.4;
    public static final double FRONT = 90.3;
    public static final double STRAIGHT = 180;
    public static final double HOLD = 61.5;
   }

  public static class alliance{

    // options are "blue" and "red"
    public static final String teamColor = "blue".toLowerCase().strip();
  }

  public static final double BEAM_BALANACED_DRIVE_KP =   1;
  public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
  public static final double BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER = 1.35;
  public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;
}
