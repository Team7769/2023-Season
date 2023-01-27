package frc.robot.Utilities;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Configuration.Constants;

public class PathFollower {
    private RamseteController _controller;
    private Trajectory _currentPath;
    private Timer _timer;

    public PathFollower() {
        _controller = new RamseteController();
        _timer = new Timer();   
    }

    public void startPath() {
        _timer.reset();
        _timer.start();
    }

    public boolean isPathFinished() {
        return _timer.get() > _currentPath.getTotalTimeSeconds();
    }

    public Trajectory getCurrentTrajectory() {
        return _currentPath;
    }

    public Pose2d getStartingPose() {
        return _currentPath.getInitialPose();
    }

    public Trajectory swerve(){
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            Constants.MAX_VELOCITY_METERS_PER_SECOND,
            Constants.MAX_ANGULAR_VELOCITY_PER_SECOND_SQUARED
        ).setKinematics(Constants._kinematics);
        
        return _currentPath;
    }
}
