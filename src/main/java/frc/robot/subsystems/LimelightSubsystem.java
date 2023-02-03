//Pipeline 1 = Cones | Pipeline 2 = Cubes

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {

    private NetworkTable limelight;
    private boolean pipeline;
    private boolean team;
    double[] positionValues;

    public LimelightSubsystem() {

        limelight = NetworkTableInstance.getDefault().getTable("limelight");
        pipeline = true; // true = cone | false = cube
        team = true; // true = blue | false = red
    }

    public double getHorizontalOffset() {

        return limelight.getEntry("tx").getDouble(0);
    }

    public double getVerticalOffset() {

        return limelight.getEntry("ty").getDouble(0);
    }

    public double getArea() {

        return limelight.getEntry("ta").getDouble(0);
    }

    public double hasValidTarget() {

        return limelight.getEntry("tv").getDouble(0);
    }

    public double robotAlign(double distance) {

        PIDController robotAlign = new PIDController(0, 0, 0);
        return robotAlign.calculate(distance, 0);
    }

    public int getPipeline() {

        return pipeline ? 1 : 2;
    }

    public Pose2d getRobotPose() {

        positionValues = team ? 
        limelight.getEntry("botpose_wpiblue").getDoubleArray(new double[6]) 
        : limelight.getEntry("botpose_wpired").getDoubleArray(new double[6]);

        Translation2d translation = new Translation2d(positionValues[0], positionValues[1]);
        Rotation2d rotation = new Rotation2d(positionValues[3], positionValues[4]);

        return new Pose2d(translation, rotation);
    }

    public void switchPipeline() {

        limelight.getEntry("pipeline").setNumber(! pipeline ? 1 : 2);
    }
}
