package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

import frc.robot.Configuration.Constants;

public class PlacerDowner extends Subsystem {
    private static PlacerDowner _instance = null;

    private CANSparkMax _placerDownerMotor;
    private RelativeEncoder _placerDownerEncoder;
    private PIDController _placerDownerPID;
    
    PlacerDowner() {
        _placerDownerMotor = new CANSparkMax(Constants.kPlacerDownerMotorDeviceId, MotorType.kBrushless);
        _placerDownerMotor.setIdleMode(IdleMode.kBrake);
        _placerDownerEncoder = _placerDownerMotor.getEncoder();

        _placerDownerPID = new PIDController(Constants.kPlacerDownerKp, Constants.kPlacerDownerKi, Constants.kPlacerDownerKd);
        _placerDownerPID.setTolerance(0.005);
    }

    public static PlacerDowner getInstance() {
        if ( _instance == null ) {
            _instance = new PlacerDowner();
        }

        return _instance;
    }

    public void liftUp() {
    }

    public void placeDown() {

    }

}
