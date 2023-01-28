
package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants 
{
  public static class DrivetrainConstants 
  {
    public static final int leftFrontMotor = 1;
    public static final int rightFrontMotor = 3;
    public static final int leftBackMotor = 2;
    public static final int rightBackMotor = 4;

    public static final int controllerPort = 0;
  }

  public static class DriveConstants
  {
    //Note: Update values accordingly
    public static final double ksVolts = 0;
    public static final double kvVoltSecondsPerMeter = 0;
    public static final double kaVoltSecondsSquaredPerMeter = 0;
    public static final double kPDriveVel = 0;

    //Note: Update values accordingly
    public static final double kTrackWidthMeters = Units.inchesToMeters(1); //Horizontal dist between 2 wheels (Meters)
    public static final DifferentialDriveKinematics kDriveKinematics = new 
      DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 3; //Leave as is
    public static final double kMaxAcceleraitionMetersPerSecondSquared = 3; //Leave as is

    public static final double kRamseteB = 2; //Leave as is
    public static final double kRamseteZeta = 0.7; //Leave as is

    //Note: update ratios accordingly
    public static final double kGearRatio = 12.6; //Gearbox ratio
    public static final double kWheelRadius = 0; //Radius of wheel in inches

    public static final double kLinearDistanceConversionFactor = 
    (Units.inchesToMeters
    (1 / kGearRatio
    * 2 * Math.PI * Units.inchesToMeters(kWheelRadius) //Circumfrence
    * 10)); //Converts ticks to metres
  }

  public static class IntakeConstants
  {
    public static final int armMotor = 5;
    public static final int handMotor = 6;
    
    //Note: update ratios accordingly
    public static final double kGearRatio = 18.8;
    public static final double kWheelRatioInches = 19.9;
  
    public static final double kLinearDistanceConversionFactor = (Units
      .inchesToMeters(1/kGearRatio*2*Math.PI*Units.inchesToMeters(kWheelRatioInches)*10));
  }
}
