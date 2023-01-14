package frc.robot.Configuration;

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
    public static final double kFrontLeftEncoderOffset = 0.0;
    public static final double kFrontRightEncoderOffset = 0.0;
    public static final double kBackLeftEncoderOffset = 0.0;
    public static final double kBackRightEncoderOffset = 0.0;

    // Deadband
    public static final double kDeadband = 0.15;
}
