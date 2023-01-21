package frc.robot.Configuration;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.util.Units;

public final class Constants {
    
    // Usb Slots
    public static final int kDriverControllerUsbSlot = 0;
    public static final int kOperatorControllerUsbSlot = 1;

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

    // Swerve Encoder Offsets
    public static final double kFrontLeftEncoderOffset = -Math.toRadians(89.909 + 180);
    public static final double kFrontRightEncoderOffset = -Math.toRadians(238.746 + 180);
    public static final double kBackLeftEncoderOffset = -Math.toRadians(22.324 + 180);
    public static final double kBackRightEncoderOffset = -Math.toRadians(145.321 + 180);

    // Swerve Properties
    public static final double MAX_VOLTAGE = 12.0;
    public static final double DRIVE_ENCODER_CONVERSION_FACTOR = SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    public static final double DRIVE_ENCODER_COUNTS_PER_REVOLUTION = 2048 * 467;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
    SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
    private static final double DRIVETRAIN_TRACK_WIDTH_METERS = Units.inchesToMeters(19.5);
    private static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(19.5);
    public static final double MAX_ANGULAR_VELOCITY_PER_SECOND = 3 * Math.PI;
    

    // Deadband
    public static final double kDeadband = 0.15;
    public static final double[] XY_Axis_inputBreakpoints = {-1, -0.85, -0.6, -0.12, 0.12, 0.6, 0.85, 1};
    public static final double[] XY_Axis_outputTable = {-1.0, -0.6, -0.3, 0, 0, 0.3, 0.6, 1.0};
    public static final double[] RotAxis_inputBreakpoints = {-1, -0.9, -0.6, -0.12, 0.12, 0.6, 0.9, 1};
    public static final double[] RotAxis_outputTable = {-1.0, -0.5, -0.2, 0, 0, 0.2, 0.5, 1.0};
}
