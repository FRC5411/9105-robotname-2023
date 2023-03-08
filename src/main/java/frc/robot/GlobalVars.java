
package frc.robot;

public class GlobalVars {
    public static boolean driveSniperMode = false;
    public static boolean armSniperMode = false;
    public static String gamePieceMode = "CONE"; // NOTE: Cone is default mode
    public static int scoreType = 0;
    public static double highestAmp = 0.0;
    public static double armPIDCalculationOutput = 0.0;
    public static double currentArmSpeed = 0;

    public static double HIGH_ANG = 0;
    public static double MID_ANG = 0;
    public static double LOW_ANG = 0;

    public static double FETCH_SUBSTATION = 0;
    public static double FETCH_GRND = 0;

    public static void checkGamePieceMode() {
        if (GlobalVars.gamePieceMode.equals("CONE")) {
          HIGH_ANG = 175;
          MID_ANG = 200;
          LOW_ANG = 280;
          FETCH_SUBSTATION = 116.3;
          FETCH_GRND = 264;
        }
        else if (GlobalVars.gamePieceMode.equals("CUBE")) {
          HIGH_ANG = 175;
          MID_ANG = 220;
          LOW_ANG = 280;
          FETCH_SUBSTATION = 125.4;
          FETCH_GRND = 265;
        }
      }
}


