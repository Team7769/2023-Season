package frc.robot.Utilities;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Configuration.Constants;

public class PathFollower {
    private static PathFollower _instance;
    private HolonomicDriveController _controller;
    private SwerveTrajectory _currentPath;
    private Timer _timer;
    private PIDController _translationXPID;
    private PIDController _translationYPID;
    private ProfiledPIDController _thetaController;

    private final double _translateKp = 2; //5.5
    private final double _translateKi = 0.0;
    private final double _translateKd = 0.0;

    private final double _rotateKp = 2; //4.5
    private final double _rotateKi = 0.0;
    private final double _rotateKd = 0.0;

    private int _pathIndex = 0;
    
    private Trajectory _blueSideStartToConePath;
    private Trajectory _blueSideConeToScorePath;

    private ArrayList<SwerveTrajectory> _currentAutonomous;

    PathFollower()
    {
        _translationXPID = new PIDController(_translateKp, _translateKi, _translateKd);
        _translationYPID = new PIDController(_translateKp, _translateKi, _translateKd);
        _thetaController = new ProfiledPIDController(_rotateKp, _rotateKi, _rotateKd, new TrapezoidProfile.Constraints(
            Units.degreesToRadians(3600), //max angular velocity
            Units.degreesToRadians(10800) //max angular acceleration
          ));
        _thetaController.enableContinuousInput(-Math.PI, Math.PI);
        _controller = new HolonomicDriveController(_translationXPID, _translationYPID, _thetaController);
        _timer = new Timer();

        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Start To Cone.wpilib.json");
            _blueSideStartToConePath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + "paths/Start To Cone.wpilib.json", ex.getStackTrace());
         }
         
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Cone To Goal.wpilib.json");
            _blueSideConeToScorePath = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + "paths/Cone To Goal.wpilib.json", ex.getStackTrace());
         }
    }

    public static PathFollower getInstance() {
        if (_instance == null) {
            _instance = new PathFollower();
        }
        
        return _instance;
    }

    public void setBlueSideTwoConeAutonomous() {
        _currentAutonomous = new ArrayList<SwerveTrajectory>();
        _currentAutonomous.add(new SwerveTrajectory(_blueSideStartToConePath, Rotation2d.fromDegrees(0)));
        _currentAutonomous.add(new SwerveTrajectory(_blueSideConeToScorePath, Rotation2d.fromDegrees(180)));

        _pathIndex = 0;
        _currentPath = _currentAutonomous.get(_pathIndex);
        _timer.reset();
    }

    public void setNextPath() {        
        if (_pathIndex >= _currentAutonomous.size()){
            return;
        } else {
            _pathIndex++;
            _currentPath = _currentAutonomous.get(_pathIndex);
        }
    }

    public void startPath()
    {
        _timer.reset();
        _timer.start();
    }

    public boolean isPathFinished()
    {
        return _timer.get() > _currentPath.getTrajectory().getTotalTimeSeconds();
    }

    public Trajectory getCurrentTrajectory()
    {
        return _currentPath.getTrajectory();
    }

    public Pose2d getStartingPose()
    {
        return _currentPath.getTrajectory().getInitialPose();
    }

    public SwerveModuleState[] getPathTarget(Pose2d currentPose) {
        var desiredState = _currentPath.getTrajectory().sample(_timer.get());
        var outputModuleStates = _controller.calculate(currentPose, desiredState, _currentPath.getDesiredRotation());

        return Constants._kinematics.toSwerveModuleStates(outputModuleStates);
    }
}
