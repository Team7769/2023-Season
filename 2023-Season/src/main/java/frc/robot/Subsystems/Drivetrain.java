package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;

public class Drivetrain extends Subsystem {

    private static Drivetrain _instance = null;

    // TODO: Move to Constants
    private static final double DRIVETRAIN_TRACK_WIDTH_METERS = Units.inchesToMeters(19.5);
    private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19.5);

    private SwerveDriveKinematics _kinematics;
    private AHRS _gyro;

    private SwerveModule _frontLeftModule;
    private SwerveModule _frontRightModule;
    private SwerveModule _backLeftModule;
    private SwerveModule _backRightModule;
    private SwerveDriveOdometry _odometry;
    private SwerveModuleState[] _moduleStates = new SwerveModuleState[4];

    private double _gyroOffset = 0.0;
    private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0);
    private final Field2d m_field = new Field2d();
    
    Drivetrain() {
        SmartDashboard.putData("Field", m_field);
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
        _kinematics = new SwerveDriveKinematics(
            // Front Left
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Front Right
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Back Left
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Back Right
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );

        _gyro = new AHRS(Port.kMXP);

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

        _moduleStates = _kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 0.0));
        _odometry = new SwerveDriveOdometry(_kinematics, Rotation2d.fromDegrees(0.0), new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        });
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
        m_field.setRobotPose(_odometry.getPoseMeters());
        SmartDashboard.putNumber("gryoAngle", getGyroscopeRotation().getDegrees());
        SmartDashboard.putNumber("drivetrainChassisSpeedsVx", _chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("drivetrainChassisSpeedsVy", _chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("drivetrainChassisSpeedsWz", _chassisSpeeds.omegaRadiansPerSecond);

        SmartDashboard.putNumber("drivetrainFrontLeftModuleDistance", _frontLeftModule.getPosition() / (Constants.DRIVE_ENCODER_COUNTS_PER_REVOLUTION * Constants.DRIVE_ENCODER_CONVERSION_FACTOR));
        SmartDashboard.putNumber("drivetrainFrontLeftModuleVelocity", _frontLeftModule.getDriveVelocity());
        SmartDashboard.putNumber("drivetrainFrontLeftModuleAngle", Math.toDegrees(_frontLeftModule.getSteerAngle()));
        SmartDashboard.putNumber("drivetrainFrontRightModuleDistance", _frontRightModule.getPosition() / (Constants.DRIVE_ENCODER_COUNTS_PER_REVOLUTION * Constants.DRIVE_ENCODER_CONVERSION_FACTOR));
        SmartDashboard.putNumber("drivetrainFrontRightModuleVelocity", _frontRightModule.getDriveVelocity());
        SmartDashboard.putNumber("drivetrainFrontRightModuleAngle", Math.toDegrees(_frontRightModule.getSteerAngle()));
        SmartDashboard.putNumber("drivetrainBackLeftModuleDistance", _backLeftModule.getPosition() / (Constants.DRIVE_ENCODER_COUNTS_PER_REVOLUTION * Constants.DRIVE_ENCODER_CONVERSION_FACTOR));
        SmartDashboard.putNumber("drivetrainBackLeftModuleVelocity", _backLeftModule.getDriveVelocity());
        SmartDashboard.putNumber("drivetrainBackLeftModuleAngle", Math.toDegrees(_backLeftModule.getSteerAngle()));
        SmartDashboard.putNumber("drivetrainBackRightModuleDistance", _backRightModule.getPosition() / (Constants.DRIVE_ENCODER_COUNTS_PER_REVOLUTION * Constants.DRIVE_ENCODER_CONVERSION_FACTOR));
        SmartDashboard.putNumber("drivetrainBackRightModuleVelocity", _backRightModule.getDriveVelocity());
        SmartDashboard.putNumber("drivetrainBackRightModuleAngle", Math.toDegrees(_backRightModule.getSteerAngle()));

        SmartDashboard.putNumber("drivetrainFrontLeftModuleTargetSpeed", _moduleStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("drivetrainFrontLeftModuleTargetAngle", _moduleStates[0].angle.getDegrees());
        SmartDashboard.putNumber("drivetrainFrontRightModuleTargetSpeed", _moduleStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("drivetrainFrontRightModuleTargetAngle", _moduleStates[1].angle.getDegrees());
        SmartDashboard.putNumber("drivetrainBackLeftModuleTargetSpeed", _moduleStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("drivetrainBackLeftModuleTargetAngle", _moduleStates[2].angle.getDegrees());
        SmartDashboard.putNumber("drivetrainBackRightModuleTargetSpeed", _moduleStates[3].speedMetersPerSecond);
        SmartDashboard.putNumber("drivetrainBackRightModuleTargetAngle", _moduleStates[3].angle.getDegrees());
    }

    public void resetOdometry() {
        resetGyro();
        _odometry.resetPosition(getGyroscopeRotation(), new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        }, new Pose2d(0, 0, new Rotation2d()));
    }

    public void resetGyro() {
        _gyro.zeroYaw();
        _gyroOffset = 0.0;
    }
    
    public Rotation2d getGyroscopeRotation() {
        if (_gyro.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(_gyro.getFusedHeading());
        }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees(360.0 - _gyro.getYaw());
        //return _gyro.getRotation2d();
    }

    public void updateOdomery() {
        _odometry.update(getGyroscopeRotation(), new SwerveModulePosition[] {
            new SwerveModulePosition(_frontLeftModule.getPosition() / (Constants.DRIVE_ENCODER_COUNTS_PER_REVOLUTION * Constants.DRIVE_ENCODER_CONVERSION_FACTOR), new Rotation2d(_frontLeftModule.getSteerAngle())),
            new SwerveModulePosition(_frontRightModule.getPosition() / (Constants.DRIVE_ENCODER_COUNTS_PER_REVOLUTION * Constants.DRIVE_ENCODER_CONVERSION_FACTOR), new Rotation2d(_frontRightModule.getSteerAngle())),
            new SwerveModulePosition(_backLeftModule.getPosition() / (Constants.DRIVE_ENCODER_COUNTS_PER_REVOLUTION * Constants.DRIVE_ENCODER_CONVERSION_FACTOR), new Rotation2d(_backLeftModule.getSteerAngle())),
            new SwerveModulePosition(_backLeftModule.getPosition() / (Constants.DRIVE_ENCODER_COUNTS_PER_REVOLUTION * Constants.DRIVE_ENCODER_CONVERSION_FACTOR), new Rotation2d(_backLeftModule.getSteerAngle()))
        });
    }

    public void robotOrientedDrive(double translationX, double translationY, double rotationZ) {
        _chassisSpeeds = new ChassisSpeeds(translationX, translationY, rotationZ);
        drive(_chassisSpeeds);
    }

    public void fieldOrientedDrive( double translationX, double translationY, double rotationZ) {
        _chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translationX, translationY, rotationZ, getGyroscopeRotation());

        drive(_chassisSpeeds);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        var moduleStates = _kinematics.toSwerveModuleStates(_chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.MAX_VELOCITY_METERS_PER_SECOND);

        setModuleStates(moduleStates);
        _moduleStates = moduleStates;
    }

    private void setModuleStates(SwerveModuleState[] moduleStates) {
        _frontLeftModule.set(moduleStates[0].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[0].angle.getRadians());
        _frontRightModule.set(moduleStates[1].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[1].angle.getRadians());
        _backLeftModule.set(moduleStates[2].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[2].angle.getRadians());
        _backRightModule.set(moduleStates[3].speedMetersPerSecond / Constants.MAX_VELOCITY_METERS_PER_SECOND * Constants.MAX_VOLTAGE, moduleStates[3].angle.getRadians());
    }
}
