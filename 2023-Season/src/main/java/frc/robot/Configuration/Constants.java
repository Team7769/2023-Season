package frc.robot.Configuration;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.estimator.AngleStatistics;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
public final class Constants {
    
    // Usb Slots
    public static final int kDriverControllerUsbSlot = 0;
    public static final int kOperatorControllerUsbSlot = 1;

    public static final int kPlacerDownerKp = 0; // Change Later
    public static final int kPlacerDownerKi = 0; // Change Later
    public static final int kPlacerDownerKd = 0; // Change Later

    public static final double kPlacerDownerFeedforwardKs = 0.29736; // Change Later
    public static final double kPlacerDownerFeedforwardKg = 0.26; // Change Later
    public static final double kPlacerDownerFeedforwardKv = 0.066; // Change Later

    // Photo Eye
    public static final int kCollectorPort = 0; // change later this is a random number

    // Swerve Slots
    public static final int kFrontLeftDriveId = 2;
    public static final int kFrontLeftSteerId = 3;
    public static final int kFrontLeftSteerEncoderId = 4;
    
    public static final int kFrontRightDriveId = 5;
    public static final int kFrontRightSteerId = 6;
    public static final int kFrontRightSteerEncoderId = 7;
    
    public static final int kBackLeftDriveId = 8;
    public static final int kBackLeftSteerId = 9;
    public static final int kBackLeftSteerEncoderId = 10;

    public static final int kBackRightDriveId = 11;
    public static final int kBackRightSteerId = 12;
    public static final int kBackRightSteerEncoderId = 13;

    // PickerUpper Slots
    public static final int kPickerUpperLeftMotorDeviceId = 14; // Change Later
    public static final int kPickerUpperRightMotorDeviceId = 15; // Change Later
    
    // PlaceDowner Slots
    public static final int kTheClawDeviceId = 16; // Change Later
    public static final int kPlacerDownerElevatorMotorDeviceId = 17; 
    
    // Solenoids
    public static final int kTilterForward = 1;
    public static final int kTilterReverse =  2;
    public static final int kPivoterForward = 3;
    public static final int kPivoterReverse = 4;
    public static final int kBoxerForward = 5;
    public static final int kBoxerReverse = 6;
    public static final int kFlexerForward = 7;
    public static final int kFlexerReverse = 8;

    // Swerve Encoder Offsets
    // public static final double kFrontLeftEncoderOffset = -Math.toRadians(89.73);
    // public static final double kFrontRightEncoderOffset = -Math.toRadians(239.06);
    // public static final double kBackLeftEncoderOffset = -Math.toRadians(22.324);
    // public static final double kBackRightEncoderOffset = -Math.toRadians(144.22);
    public static final double kFrontLeftEncoderOffset = -Math.toRadians(89.469);
    public static final double kFrontRightEncoderOffset = -Math.toRadians(239.502);
    //public static final double kBackLeftEncoderOffset = -Math.toRadians(23.112);
    public static final double kBackLeftEncoderOffset = -Math.toRadians(16.52);
    public static final double kBackRightEncoderOffset = -Math.toRadians(144.044);

    // Swerve Properties
    public static final double MAX_VOLTAGE = 12.0;
    public static final double DRIVE_ENCODER_COUNTS_PER_REVOLUTION = 2048;
    public static final double DRIVE_ENCODER_CONVERSION_FACTOR = SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI / DRIVE_ENCODER_COUNTS_PER_REVOLUTION;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
    SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    private static final double DRIVETRAIN_TRACK_WIDTH_METERS = Units.inchesToMeters(19.5);
    private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19.5);
    public static final double MAX_ANGULAR_VELOCITY_PER_SECOND = 3 * Math.PI;
    public static final double MAX_ANGULAR_VELOCITY_PER_SECOND_SQUARED = MAX_ANGULAR_VELOCITY_PER_SECOND*
    MAX_ANGULAR_VELOCITY_PER_SECOND;

    //setting up kinematics
    public static final SwerveDriveKinematics _kinematics = new SwerveDriveKinematics(
            // Front Left
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Front Right
            new Translation2d(DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Back Left
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
    
            //Back Right
            new Translation2d(-DRIVETRAIN_TRACK_WIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
        );
    

    // Deadband
    public static final double kDeadband = 0.15;
    public static final double[] XY_Axis_inputBreakpoints =  {-1,  -0.9, -0.85, -0.7, -0.6, -0.5, -0.2,  -0.12, 0.12, 0.2,  0.5, 0.6, 0.7, 0.85, .9, 1};
    public static final double[] XY_Axis_outputTable = {-1.0, -.7, -0.6, -0.4, -0.3, -0.2, -0.05, 0, 0, 0.05,  0.2, 0.3, 0.4,  0.6, .7, 1.0};
    public static final double[] RotAxis_inputBreakpoints =  {-1, -.9, -0.85, -0.7, -0.6, -0.5, -0.2,  -0.12, 0.12, 0.2,  0.5, 0.6, 0.7, 0.85, .9, 1};
    public static final double[] RotAxis_outputTable = {-1.0, -.7, -0.6, -0.4, -0.3, -0.2, -0.05, 0, 0, 0.05,  0.2, 0.3, 0.4,  0.6, .7, 1.0};

}
