
package frc.robot;

import edu.wpi.first.math.util.Units;

public final class Constants 
{
  public static class DrivetrainConstants 
  {
    public static int leftFrontMotor = 1;
    public static int rightFrontMotor = 3;
    public static int leftBackMotor = 2;
    public static int rightBackMotor = 4;

    public static int controllerPort = 0;
  }

  public static class IntakeConstants
  {
    public static int armMotor = 5;
    public static int handMotor = 6;
    
    public static final double kGearRatio = 18.8;
    public static final double kWheelRatioInches = 19.9;
  
    public static final double kLinearDistanceConversionFactor = (Units
      .inchesToMeters(1/kGearRatio*2*Math.PI*Units.inchesToMeters(kWheelRatioInches)*10));
  }
}
