package frc.robot.Utilities;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class PathFollower {
    private static PathFollower _instance;
    private PPHolonomicDriveController _controller;
    private PathPlannerTrajectory _currentPath;
    private List<PathPlannerTrajectory> _currentSegmentedPath;
    private Timer _timer;
    private PIDController _translationXPID;
    private PIDController _translationYPID;
    private PIDController _thetaController;

    private final double _translateKp = 5.5; //5.5
    private final double _translateKi = 0.0;
    private final double _translateKd = 0.0;

    private final double _rotateKp = 4.5; //4.5
    private final double _rotateKi = 0.0;
    private final double _rotateKd = 0.0;

    private int _pathIndex = 0;
    
    private PathPlannerTrajectory _testTrajectory;
    private List<PathPlannerTrajectory> _loadSideLinkBalance;
    private List<PathPlannerTrajectory> _loadSidePickupBalance;
    private List<PathPlannerTrajectory> _loadSidePickupScore;
    private List<PathPlannerTrajectory> _cableSideTwoConeBalance;
    private List<PathPlannerTrajectory> _cableSideLinkBalance;
    private List<PathPlannerTrajectory> _loadingSideLinkNoBalance;
    private List<PathPlannerTrajectory> _loadingSideLinkConeNoBalance;

    //private ArrayList<SwerveTrajectory> _currentAutonomous;

    PathFollower()
    {
        _translationXPID = new PIDController(_translateKp, _translateKi, _translateKd);
        _translationYPID = new PIDController(_translateKp, _translateKi, _translateKd);
        // _thetaController = new ProfiledPIDController(_rotateKp, _rotateKi, _rotateKd, new TrapezoidProfile.Constraints(
        //     Units.degreesToRadians(3600), //max angular velocity
        //     Units.degreesToRadians(10800) //max angular acceleration
        //   ));
        // _thetaController.enableContinuousInput(-Math.PI, Math.PI);
        _thetaController = new PIDController(_rotateKp, _rotateKi, _rotateKd);
        _thetaController.enableContinuousInput(-Math.PI, Math.PI);
        //_controller = new HolonomicDriveController(_translationXPID, _translationYPID, _thetaController);
        
        _controller = new PPHolonomicDriveController(_translationXPID, _translationYPID, _thetaController);
        _timer = new Timer();
        _testTrajectory = PathPlanner.loadPath("TestPath", new PathConstraints(2.25, 3), true);
        _cableSideTwoConeBalance = PathPlanner.loadPathGroup("CableTwoConeBalance", true, new PathConstraints(2.25, 3));
        _loadSideLinkBalance = PathPlanner.loadPathGroup("LoadSideLinkBalance", true, new PathConstraints(2.25, 3));
        _loadSidePickupBalance = PathPlanner.loadPathGroup("LoadSidePickupBalance", true, new PathConstraints(2.25, 3));
        _loadSidePickupScore = PathPlanner.loadPathGroup("LoadSidePickupScore", true, new PathConstraints(2.25, 3));
        _cableSideLinkBalance = PathPlanner.loadPathGroup("CableSide-Link-Balance", true, new PathConstraints(2.25, 3));
        _loadingSideLinkNoBalance = PathPlanner.loadPathGroup("LoadingSide-Link-NoBalance", true, new PathConstraints(2.25, 3));
        _loadingSideLinkConeNoBalance = PathPlanner.loadPathGroup("LoadingSide-LinkCone-NoBalance", true, new PathConstraints(2.25, 3));
    }

    public static PathFollower getInstance() {
        if (_instance == null) {
            _instance = new PathFollower();
        }
        
        return _instance;
    }

    public void logData() {
        SmartDashboard.putNumber("timer", _timer.get());
    }

    public void setTestAuto() {
        initializeTrajectory(_testTrajectory);
    }

    public void setCableSideLinkBalance() {
        _pathIndex = 0;
        _currentSegmentedPath = _cableSideLinkBalance;
        initializeTrajectory(_cableSideLinkBalance.get(_pathIndex));
    }

    public void setLoadingSideLinkNoBalance() {
        _pathIndex = 0;
        _currentSegmentedPath = _loadingSideLinkNoBalance;
        initializeTrajectory(_loadingSideLinkNoBalance.get(_pathIndex));
    }

    public void setLoadingSideLinkConeNoBalance() {
        _pathIndex = 0;
        _currentSegmentedPath = _loadingSideLinkConeNoBalance;
        initializeTrajectory(_loadingSideLinkConeNoBalance.get(_pathIndex));
    }

    public void setLoadsideLinkBalance() {
        _pathIndex = 0;
        _currentSegmentedPath = _loadSideLinkBalance;
        initializeTrajectory(_loadSideLinkBalance.get(_pathIndex));
    }
    
    public void setLoadsidePickupBalance() {
        _pathIndex = 0;
        _currentSegmentedPath = _loadSidePickupBalance;
        initializeTrajectory(_loadSidePickupBalance.get(_pathIndex));
    }
    
    public void setLoadsidePickupScore() {
        _pathIndex = 0;
        _currentSegmentedPath = _loadSidePickupScore;
        initializeTrajectory(_loadSidePickupScore.get(_pathIndex));
    }

    public void setCableSideTwoConeBalance() {
        _pathIndex = 0;
        _currentSegmentedPath = _cableSideTwoConeBalance;
        initializeTrajectory(_cableSideTwoConeBalance.get(_pathIndex));
    }

    private void initializeTrajectory(PathPlannerTrajectory trajectory) {
        _currentPath =
            PathPlannerTrajectory.transformTrajectoryForAlliance(
                trajectory, DriverStation.getAlliance());
        _timer.stop();
        _timer.reset();
        PathPlannerServer.sendActivePath(_currentPath.getStates());
    }

    public void setNextPath() {
        if (_currentSegmentedPath == null || _pathIndex >= _currentSegmentedPath.size()){
            return;
        } else {
            _pathIndex++;
            initializeTrajectory(_currentSegmentedPath.get(_pathIndex));
        }
    }

    public void startPath()
    {
        _timer.reset();
        _timer.start();
    }

    public boolean isPathFinished()
    {
        return _timer.hasElapsed(_currentPath.getTotalTimeSeconds());
    }

    public PathPlannerTrajectory getCurrentTrajectory()
    {
        return _currentPath;
    }

    public Pose2d getStartingPose()
    {
        return _currentPath.getInitialHolonomicPose();
    }
    public PathPlannerState getInitialState() {
        return _currentPath.getInitialState();
    }

    public SwerveModuleState[] getPathTarget(Pose2d currentPose) {
        PathPlannerState desiredState = (PathPlannerState) _currentPath.sample(_timer.get());

        // SmartDashboard.putNumber("desiredX", desiredState.poseMeters.getX());
        // SmartDashboard.putNumber("desiredY", desiredState.poseMeters.getY());
        // SmartDashboard.putNumber("desiredZ", desiredState.poseMeters.getRotation().getDegrees());
        // SmartDashboard.putNumber("desiredHolonomic", desiredState.holonomicRotation.getDegrees());

        // PathPlannerServer.sendPathFollowingData(
        // new Pose2d(desiredState.poseMeters.getTranslation(), desiredState.holonomicRotation),
        // currentPose);

        var outputModuleStates = _controller.calculate(currentPose, desiredState);
        return Constants._kinematics.toSwerveModuleStates(outputModuleStates);
    }
}
