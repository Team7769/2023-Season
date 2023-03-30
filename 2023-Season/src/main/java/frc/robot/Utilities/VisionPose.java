package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionPose {
    public Pose2d pose;
    public double latencyAdjustedTimestamp;

    public VisionPose(Pose2d botPose, double timestamp) {
        pose = botPose;
        latencyAdjustedTimestamp = timestamp;
    }
}
