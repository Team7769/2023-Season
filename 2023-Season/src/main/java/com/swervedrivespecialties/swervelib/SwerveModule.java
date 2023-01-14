package com.swervedrivespecialties.swervelib;

public interface SwerveModule {
    double getPosition();
    
    double getDriveVelocity();

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);
}
