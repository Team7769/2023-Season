package frc.robot.Utilities;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveTrajectory {
    private Trajectory _trajectory;
    private Rotation2d _desiredRotation;

    public SwerveTrajectory(Trajectory trajectory, Rotation2d desiredRotation) {
        _trajectory = trajectory;
        _desiredRotation = desiredRotation;
    }

    public Trajectory getTrajectory() {
        return _trajectory;
    }

    public Rotation2d getDesiredRotation()
    {
        return _desiredRotation;
    }
}
