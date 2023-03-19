package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.server.PathPlannerServer;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.ADIS16448_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16448_IMU.IMUAxis;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Utilities.Limelight;
import frc.robot.Utilities.PathFollower;

public class Drivetrain extends Subsystem {

    private static Drivetrain _instance = null;
    private static PathFollower _pathFollower;
    private static Limelight _limelight;

    private Gyro _gyro;
    private AHRS _ahrs;

    private SwerveModule _frontLeftModule;
    private SwerveModule _frontRightModule;
    private SwerveModule _backLeftModule;
    private SwerveModule _backRightModule;
    private SwerveDrivePoseEstimator _drivePoseEstimator;
    private SwerveModuleState[] _moduleStates = new SwerveModuleState[4];
    private PIDController _wallFacingController;

    private double _gyroOffset = 0.0;
    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0);
    private final Field2d m_field = new Field2d();
    
    Drivetrain() {
        SmartDashboard.putData("Field", m_field);
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        _ahrs = new AHRS(Port.kMXP);
        _gyro = _ahrs;

        _frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
               .withSize(2, 4)
               .withPosition(0, 0),
               Mk4SwerveModuleHelper.GearRatio.L2, 
               Constants.kFrontLeftDriveId, 
               Constants.kFrontLeftSteerId, 
               Constants.kFrontLeftSteerEncoderId, 
               Constants.kFrontLeftEncoderOffset);
               
        _frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
               .withSize(2, 4)
               .withPosition(2, 0),
               Mk4SwerveModuleHelper.GearRatio.L2, 
               Constants.kFrontRightDriveId, 
               Constants.kFrontRightSteerId, 
               Constants.kFrontRightSteerEncoderId, 
               Constants.kFrontRightEncoderOffset);
               
        _backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
               .withSize(2, 4)
               .withPosition(4, 0),
               Mk4SwerveModuleHelper.GearRatio.L2, 
               Constants.kBackLeftDriveId, 
               Constants.kBackLeftSteerId, 
               Constants.kBackLeftSteerEncoderId, 
               Constants.kBackLeftEncoderOffset);
               
        _backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
               .withSize(2, 4)
               .withPosition(6, 0),
               Mk4SwerveModuleHelper.GearRatio.L2, 
               Constants.kBackRightDriveId, 
               Constants.kBackRightSteerId, 
               Constants.kBackRightSteerEncoderId, 
               Constants.kBackRightEncoderOffset);

        _moduleStates = Constants._kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));
        
        _drivePoseEstimator = new SwerveDrivePoseEstimator(Constants._kinematics, getGyroscopeRotation(), new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        }, new Pose2d());

        _pathFollower = PathFollower.getInstance();
        _limelight = Limelight.getInstance();
        _wallFacingController = new PIDController(0.125, 0.02, .01);
        _wallFacingController.setIntegratorRange(-0.1, 0.1);
        _wallFacingController.enableContinuousInput(-180, 180);
    }

    public static Drivetrain getInstance() {
        if (_instance == null) {
            _instance = new Drivetrain();
        }

        return _instance;
    }

    @Override
    public void zeroSensors() {
        resetOdometry();
    }

    @Override
    public void logTelemetry() {
        var pose = _drivePoseEstimator.getEstimatedPosition();
        m_field.setRobotPose(pose);
        //SmartDashboard.putData("Field", m_field);
        SmartDashboard.putNumber("drivetrainGyroAngle", getGyroscopeRotation().getDegrees());
        SmartDashboard.putNumber("drivetrainChassisSpeedsVx", _chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("drivetrainChassisSpeedsVy", _chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("drivetrainChassisSpeedsWz", _chassisSpeeds.omegaRadiansPerSecond);

        // SmartDashboard.putNumber("drivetrainFrontLeftModuleDistance", _frontLeftModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR);
        // SmartDashboard.putNumber("drivetrainFrontLeftModuleVelocity", _frontLeftModule.getDriveVelocity());
        // SmartDashboard.putNumber("drivetrainFrontLeftModuleAngle", Math.toDegrees(_frontLeftModule.getSteerAngle()));
        // SmartDashboard.putNumber("drivetrainFrontRightModuleDistance", _frontRightModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR);
        // SmartDashboard.putNumber("drivetrainFrontRightModuleVelocity", _frontRightModule.getDriveVelocity());
        // SmartDashboard.putNumber("drivetrainFrontRightModuleAngle", Math.toDegrees(_frontRightModule.getSteerAngle()));
        // SmartDashboard.putNumber("drivetrainBackLeftModuleDistance", _backLeftModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR);
        // SmartDashboard.putNumber("drivetrainBackLeftModuleVelocity", _backLeftModule.getDriveVelocity());
        // SmartDashboard.putNumber("drivetrainBackLeftModuleAngle", Math.toDegrees(_backLeftModule.getSteerAngle()));
        // SmartDashboard.putNumber("drivetrainBackRightModuleDistance", _backRightModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR);
        // SmartDashboard.putNumber("drivetrainBackRightModuleVelocity", _backRightModule.getDriveVelocity());
        // SmartDashboard.putNumber("drivetrainBackRightModuleAngle", Math.toDegrees(_backRightModule.getSteerAngle()));

        // SmartDashboard.putNumber("drivetrainFrontLeftModuleTargetSpeed", _moduleStates[0].speedMetersPerSecond);
        // SmartDashboard.putNumber("drivetrainFrontLeftModuleTargetAngle", _moduleStates[0].angle.getDegrees());
        // SmartDashboard.putNumber("drivetrainFrontRightModuleTargetSpeed", _moduleStates[1].speedMetersPerSecond);
        // SmartDashboard.putNumber("drivetrainFrontRightModuleTargetAngle", _moduleStates[1].angle.getDegrees());
        // SmartDashboard.putNumber("drivetrainBackLeftModuleTargetSpeed", _moduleStates[2].speedMetersPerSecond);
        // SmartDashboard.putNumber("drivetrainBackLeftModuleTargetAngle", _moduleStates[2].angle.getDegrees());
        // SmartDashboard.putNumber("drivetrainBackRightModuleTargetSpeed", _moduleStates[3].speedMetersPerSecond);
        // SmartDashboard.putNumber("drivetrainBackRightModuleTargetAngle", _moduleStates[3].angle.getDegrees());
        SmartDashboard.putNumber("drivetrainGyroOffset", _gyroOffset);
        // SmartDashboard.putNumber("drivetrainPitch", _ahrs.getRoll());

        SmartDashboard.putNumber("drivetrainOdometryX", pose.getX());
        SmartDashboard.putNumber("drivetrainOdometryY", pose.getY());
        SmartDashboard.putNumber("drivetrainOdometryZ", pose.getRotation().getDegrees());
    }

    public void logPose() {
        var currentState = _pathFollower.getInitialState();
        //PathPlannerServer.sendPathFollowingData(currentState.poseMeters, _odometry.getPoseMeters());
        SmartDashboard.putNumber("drivetrainPathStateInitialPoseX", currentState.poseMeters.getX());
        SmartDashboard.putNumber("drivetrainPathStateInitialPoseY", currentState.poseMeters.getY());
        SmartDashboard.putNumber("drivetrainPathStateInitialPoseZ", currentState.poseMeters.getRotation().getDegrees());
        SmartDashboard.putNumber("drivetrainPathStateInitialPoseHolonomic", currentState.holonomicRotation.getDegrees());
    }

    public void resetOdometry() {
        resetGyro();
        _drivePoseEstimator.resetPosition(getGyroscopeRotation(), new SwerveModulePosition[] {
            new SwerveModulePosition(_frontLeftModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_frontLeftModule.getSteerAngle())),
            new SwerveModulePosition(_frontRightModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_frontRightModule.getSteerAngle())),
            new SwerveModulePosition(_backLeftModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_backLeftModule.getSteerAngle())),
            new SwerveModulePosition(_backRightModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_backRightModule.getSteerAngle()))
        }, new Pose2d());
    }

    public void resetGyro() {
        _gyro.reset();
        _gyroOffset = 0.0;
    }
    
    public Rotation2d getGyroscopeRotation() {
        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        //return Rotation2d.fromDegrees(_gyro.getYaw() - 180);
        return _gyro.getRotation2d();
    }

    public Rotation2d getGyroscopeRotationWithOffset() {
        return Rotation2d.fromDegrees(_gyro.getRotation2d().getDegrees() + _gyroOffset);
    }

    public void updateOdometry() {
        var currentRotation = getGyroscopeRotation();
        _drivePoseEstimator.updateWithTime(Timer.getFPGATimestamp(), currentRotation, new SwerveModulePosition[] {
            new SwerveModulePosition(_frontLeftModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_frontLeftModule.getSteerAngle())),
            new SwerveModulePosition(_frontRightModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_frontRightModule.getSteerAngle())),
            new SwerveModulePosition(_backLeftModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_backLeftModule.getSteerAngle())),
            new SwerveModulePosition(_backRightModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_backRightModule.getSteerAngle()))
        });

        // var visionPose = _limelight.getPose();
        // if (visionPose != null) {
        //     _drivePoseEstimator.addVisionMeasurement(visionPose.pose, visionPose.latencyAdjustedTimestamp);
        // }
    }
    public boolean isLevel() {
        return Math.abs(_ahrs.getRoll()) <= 5;
    }

    public boolean isTilting(){
        return Math.abs(_ahrs.getRoll()) <= 13 && Math.abs(_ahrs.getRoll()) > 5;
    }

    public double getBalanceSpeed() {
        var roll = _ahrs.getRoll();
        var absRoll = Math.abs(roll);

        if (absRoll <= 5) {
            return 0.0;
        }

        if (absRoll <= 13 && absRoll > 5) {
            return roll > 0 ? -0.10 : 0.10;
        } else {
            return roll > 0 ? 0.18 : -0.18;
        }
    }

    public void robotOrientedDrive(double translationX, double translationY, double rotationZ) {
        _chassisSpeeds = new ChassisSpeeds(translationX, translationY, rotationZ);
        drive(_chassisSpeeds);
    }

    public void fieldOrientedDrive( double translationX, double translationY, double rotationZ) {
        _chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translationX, translationY, rotationZ, getGyroscopeRotationWithOffset());

        drive(_chassisSpeeds);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        var moduleStates = Constants._kinematics.toSwerveModuleStates(_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);

        setModuleStates(moduleStates);
        _moduleStates = moduleStates;
    }

    public void followTrajectory() {
        var output = _pathFollower.getPathTarget(_drivePoseEstimator.getEstimatedPosition());
        SwerveDriveKinematics.desaturateWheelSpeeds(output, Constants.MAX_VELOCITY_METERS_PER_SECOND);
        
        setModuleStates(output);
    }

    public void initAutonPosition() {
        resetGyro();
        var pathInitialState = _pathFollower.getInitialState();
        _gyroOffset = pathInitialState.holonomicRotation.getDegrees();
        _drivePoseEstimator.resetPosition(getGyroscopeRotation(), new SwerveModulePosition[] {
            new SwerveModulePosition(_frontLeftModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_frontLeftModule.getSteerAngle())),
            new SwerveModulePosition(_frontRightModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_frontRightModule.getSteerAngle())),
            new SwerveModulePosition(_backLeftModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_backLeftModule.getSteerAngle())),
            new SwerveModulePosition(_backRightModule.getDistance() * Constants.DRIVE_ENCODER_CONVERSION_FACTOR, new Rotation2d(_backRightModule.getSteerAngle()))
        }, new Pose2d(pathInitialState.poseMeters.getTranslation(), pathInitialState.holonomicRotation));

        // SmartDashboard.putNumber("drivetrainInitialStateRotation", pathInitialState.holonomicRotation.getDegrees());
        m_field.getObject("traj").setTrajectory(_pathFollower.getCurrentTrajectory());
    }

    public void resetWallFacingController() {
        _wallFacingController.reset();
    }

    public double getWallRotationTarget(double targetRotation) {
        var output = _wallFacingController.calculate(getGyroscopeRotationWithOffset().getDegrees(), targetRotation);

        // SmartDashboard.putNumber("drivetrainWallRotationTarget", targetRotation);
        // SmartDashboard.putNumber("drivetrainWallRotationOutput", output);
        return output;
    }

    private void setModuleStates(SwerveModuleState[] moduleStates) {
        _frontLeftModule.set(moduleStates[0].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[0].angle.getRadians());
        _frontRightModule.set(moduleStates[1].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[1].angle.getRadians());
        _backLeftModule.set(moduleStates[2].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[2].angle.getRadians());
        _backRightModule.set(moduleStates[3].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[3].angle.getRadians());
    }
}
