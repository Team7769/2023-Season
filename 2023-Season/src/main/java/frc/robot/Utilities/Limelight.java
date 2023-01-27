package frc.robot.Utilities;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Configuration.Constants;

public class Limelight {
    private NetworkTableEntry _validTargets;
    private NetworkTableEntry _camMode;
    private NetworkTableEntry _ledMode;
    private NetworkTableEntry _tagId;

    public static Limelight _instance;

    public static Limelight getInstance() {
        if (_instance == null) {
            _instance = new Limelight();
        }

        return _instance;
    }

    public Limelight() {
        _validTargets = NetworkTableInstance.getDefault()
            .getTable(Constants.kTableName)
            .getEntry(Constants.kValidTargetKey);

        _ledMode = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kLEDModeKey);
        _camMode = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kCamModeKey);
        _tagId = NetworkTableInstance.getDefault().getTable(Constants.kTableName).getEntry(Constants.kTargetTagId);
    }

    public boolean hasTarget() {
        return _validTargets.getDouble(0) >= 1;
    }

    public boolean isAimbot() {
        return _camMode.getDouble(0) >= 1;
    }

    public double getTargetId() {
        return NetworkTableInstance.getDefault()
        .getTable(Constants.kTableName)
        .getEntry(Constants.kTargetTagId)
        .getDouble(0);
    }

    public void setAimbot() {
        _ledMode.setDouble(Constants.kLEDOn);
        _camMode.setDouble(Constants.kCamModeVisionProcessing);
    }

    public void setDashcam() {
        _ledMode.setDouble(Constants.kLEDOff);
        _camMode.setDouble(Constants.kCamModeDriver);
    }

}
