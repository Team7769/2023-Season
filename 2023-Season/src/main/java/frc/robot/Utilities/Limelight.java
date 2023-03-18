package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class Limelight {

    public static Limelight _instance;
    private static NetworkTableEntry _botpose;
    private static NetworkTableEntry _hasTag;
    private static NetworkTableEntry _tl;
    private static NetworkTableEntry _cl;

    private final double kXOffset = 8.27;
    private final double kYOffset = 4.00;
    
    public Limelight() {
        _botpose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose");
        _hasTag = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid");
        _tl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl");
        _cl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("cl");
    }

    public static Limelight getInstance()
    {
        if (_instance == null)
        {
            _instance = new Limelight();
        }

        return _instance;
    }

    public boolean hasTag() {
        return _hasTag.getInteger(-1) >= 0;
    }

    public VisionPose getPose() {
        if (!hasTag()) {
            return null;
        }

        var botpose = _botpose.getDoubleArray(new double[0]);
        var tl = _tl.getDouble(0.0);
        var cl = _cl.getDouble(0.0);

        if (botpose == null || botpose.length == 0) {
            return null;
        }

        var pose = new Pose2d(new Translation2d(botpose[0] + kXOffset, botpose[1] + kYOffset), Rotation2d.fromDegrees(botpose[5]));
        var latencyAdjustedTimestamp = Timer.getFPGATimestamp() - (tl / 1000.0) - (cl/1000.0);
        return new VisionPose(pose, latencyAdjustedTimestamp);
    }
}
