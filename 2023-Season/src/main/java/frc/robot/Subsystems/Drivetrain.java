package frc.robot.Subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Drivetrain extends Subsystem {

    private static Drivetrain _instance = null;

    // TODO: Move to Constants
    private static final double MAX_VOLTAGE = 12.0;
    private static final double MAX_VELOCITY_METERS_PER_SECOND = 0;
    private static final double MAX_ANGULAR_VELOCITY_PER_SECOND = 0;
    private static final double DRIVETRAIN_TRACK_WIDTH_METERS = 0;
    private static final double DRIVETRAIN_WHEELBASE_METERS = 0;

    private SwerveDriveKinematics _kinematics;
    private Gyro _gyro;

    private SwerveModule _frontLeftModule;
    private SwerveModule _frontRightModule;
    private SwerveModule _backLeftModule;
    private SwerveModule _backRightModule;

    Drivetrain() {
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
    }

    public static Drivetrain GetInstance() {
        if (_instance == null) {
            _instance = new Drivetrain();
        }

        return _instance;
    }
}
