package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/*
GET BASIC TARGET DATA VAR NAMES
tv	         Whether the limelight has any valid targets (0 or 1)
tx	         Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
ty	         Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
ta	         Target Area (0% of image to 100% of image)
tl	         The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
tshort	     Sidelength of shortest side of the fitted bounding box (pixels)
tlong	       Sidelength of longest side of the fitted bounding box (pixels)
thor	       Horizontal sidelength of the rough bounding box (0 - 320 pixels)
tvert	       Vertical sidelength of the rough bounding box (0 - 320 pixels)
getpipe	     True active pipeline index of the camera (0 .. 9)
json	       Full JSON dump of targeting results
tclass	     Class ID of primary neural detector result or neural classifier result


/////////////////////////////////////////////////////////////////////////


GET APRILTAG / 3D DATA:
botpose	                     Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z)
botpose_wpiblue	             Robot transform in field-space (blue driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
botpose_wpired	             Robot transform in field-space (red driverstation WPILIB origin). Translation (X,Y,Z) Rotation(X,Y,Z)
camerapose_targetspace	     3D transform of the camera in the coordinate system of the primary in-view AprilTag (array (6))
targetpose_cameraspace	     3D transform of the primary in-view AprilTag in the coordinate system of the Camera (array (6))
targetpose_robotspace	       3D transform of the primary in-view AprilTag in the coordinate system of the Robot (array (6))
botpose_targetspace	         3D transform of the robot in the coordinate system of the primary in-view AprilTag (array (6))
tid	                         ID of the primary in-view AprilTag


/////////////////////////////////////////////////////////////////////////


EXECUTE CAMERA CONTROLS VAR NAMES AND ACTION VALUES:

ledMode     	Sets limelight’s LED state
0	            use the LED Mode set in the current pipeline
1	            force off
2	            force blink
3	            force on

camMode	      Sets limelight’s operation mode
0	            Vision processor
1	            Driver Camera (Increases exposure, disables vision processing)

pipeline	    Sets limelight’s current pipeline
0 .. 9	      Select pipeline 0..9

stream	      Sets limelight’s streaming mode
0	            Standard -  Side-by-side streams if a webcam is attached to Limelight
1	            PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
2	            PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream

snapshot	    Allows users to take snapshots during a match
0	            Reset snapshot mode
1	            Take exactly one snapshot

*/

public class LimelightSubsystem extends SubsystemBase {

  static double getBasicTargetData(String varName) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(varName).getDouble(0);  
  }
  
  static double[] getApriltagData(String varName) {
    return NetworkTableInstance.getDefault().getTable("limelight").getEntry(varName).getDoubleArray(new double[6]);
  }
  
  static void execCameraControls(String varName, int stateValue) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry(varName).setNumber(stateValue);
  }

  public LimelightSubsystem() {
    
    
  }
  
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
