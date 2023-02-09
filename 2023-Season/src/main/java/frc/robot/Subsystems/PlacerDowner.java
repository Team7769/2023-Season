package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Enums.PlacerDownerState;

public class PlacerDowner extends Subsystem {
    private static PlacerDowner _instance = null;

    private CANSparkMax _placerDownerMotor;
    private CANSparkMax _placerDownerElevator;
    private RelativeEncoder _placerDownerElevatorEncoder;
    private PIDController _placerDownerPID;
    private DoubleSolenoid _tilter;
    private DoubleSolenoid _pivoter;

    private PlacerDownerState _currentState = PlacerDownerState.STOW;
    private final double _placerDownerSpeed = 0.25;
    
    PlacerDowner() {
        _placerDownerMotor = new CANSparkMax(Constants.kPlacerDownerMotorDeviceId, MotorType.kBrushless);
        _placerDownerMotor.setIdleMode(IdleMode.kBrake);
        _placerDownerMotor.setSmartCurrentLimit(20);
        _placerDownerMotor.burnFlash();

        //_tilter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kTilterForward, Constants.kTilterReverse);
        //_pivoter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kPivoterForward, Constants.kPivoterReverse);

        // _placerDownerMotor = new CANSparkMax(Constants.kPlacerDownerMotorDeviceId, MotorType.kBrushless);
        // _placerDownerMotor.setIdleMode(IdleMode.kBrake);
        // _placerDownerEncoder = _placerDownerMotor.getEncoder();

        // _placerDownerPID = new PIDController(Constants.kPlacerDownerKp, Constants.kPlacerDownerKi, Constants.kPlacerDownerKd);
        // _placerDownerPID.setTolerance(0.005);
    }

    public static PlacerDowner getInstance() {
        if ( _instance == null ) {
            _instance = new PlacerDowner();
        }

        return _instance;
    }

    @Override
    public void logTelemetry() {
        SmartDashboard.putString("placerDownerState", _currentState.name());
    }

    private void intake() {
        _placerDownerMotor.set(_placerDownerSpeed);
    }
    
    private void eject() {
        _placerDownerMotor.set(-_placerDownerSpeed);
    }

    private void stop() {
        _placerDownerMotor.set(0);
    }
    
    private void deploy() {
        _tilter.set(Value.kForward);
        _pivoter.set(Value.kForward);
    }

    public void retract() {
        _tilter.set(Value.kReverse);
        _pivoter.set(Value.kReverse);
    }

    public void setElevatorSetpoint() {
        
    }

    public void handleCurrentState() {
        switch (_currentState) {
            case INTAKE:
                intake();
                break;
            case EJECT:
                eject();
                break;
            case DEPLOY:
                deploy();
                break;
            case STOW:
                retract();
                break;
            case STOP:
            default:
                stop();
                break;
        }
    }

    public void setWantedState(PlacerDownerState wantedState) {
        _currentState = wantedState;
    }
}
