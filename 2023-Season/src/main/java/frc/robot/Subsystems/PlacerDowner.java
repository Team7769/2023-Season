package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Configuration.ElevatorPosition;
import frc.robot.Enums.PlacerDownerState;

public class PlacerDowner extends Subsystem {
    private static PlacerDowner _instance = null;

    private CANSparkMax _placerDownerMotor;
    private CANSparkMax _placerDownerElevator;
    private RelativeEncoder _placerDownerElevatorEncoder;
    private PIDController _placerDownerPID;
    private DoubleSolenoid _tilter;
    private DoubleSolenoid _pivoter;
    private ElevatorFeedforward _feedForward;

    private SparkMaxPIDController _elevatorController;

    private PlacerDownerState _currentState = PlacerDownerState.STOW;
    private double _setpoint = ElevatorPosition.DIGIORNO;
    private final double _placerDownerSpeed = 0.25;

    private final double kSmartMotionP = 0.0;
    private final double kSmartMotionI = 0.0;
    private final double kSmartMotionD = 0.0;
    private final double kSmartMotionFF = 0.0;
    private final double kSmartMotionIz = 0.0;
    private final double kSmartMotionMaxOutput = 1.0;
    private final double kSmartMotionMinOutput = -1.0;
    private final double kSmartMotionMaxVel = 2000;
    private final double kSmartMotionMaxAccel = 1500;
    private final double kSmartMotionAllowedError = 100;
    
    PlacerDowner() {
        _placerDownerMotor = new CANSparkMax(Constants.kPlacerDownerMotorDeviceId, MotorType.kBrushless);
        _placerDownerMotor.setIdleMode(IdleMode.kBrake);
        _placerDownerMotor.setSmartCurrentLimit(20);
        _placerDownerMotor.burnFlash();

        //_tilter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kTilterForward, Constants.kTilterReverse);
        //_pivoter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kPivoterForward, Constants.kPivoterReverse);

        _placerDownerElevator = new CANSparkMax(Constants.kPlacerDownerElevatorMotorDeviceId, MotorType.kBrushless);
        _placerDownerElevator.setSmartCurrentLimit(20);
        _placerDownerElevator.setSecondaryCurrentLimit(40);
        _placerDownerElevator.setIdleMode(IdleMode.kBrake);
        _placerDownerElevatorEncoder = _placerDownerMotor.getEncoder();

        // WPILib Controller
        _placerDownerPID = new PIDController(Constants.kPlacerDownerKp, Constants.kPlacerDownerKi, Constants.kPlacerDownerKd);
        _placerDownerPID.setTolerance(100);

        _feedForward = new ElevatorFeedforward(Constants.kPlacerDownerFeedforwardKs, Constants.kPlacerDownerFeedforwardKg, Constants.kPlacerDownerFeedforwardKv);
        
        // Spark MAX Smart Motion Parameters
        _elevatorController = _placerDownerElevator.getPIDController();
        _elevatorController.setP(kSmartMotionP);
        _elevatorController.setI(kSmartMotionI);
        _elevatorController.setD(kSmartMotionD);
        _elevatorController.setIZone(kSmartMotionIz);
        _elevatorController.setFF(kSmartMotionFF);
        _elevatorController.setOutputRange(kSmartMotionMinOutput, kSmartMotionMaxOutput);
        _elevatorController.setSmartMotionMaxVelocity(kSmartMotionMaxVel, 0);
        _elevatorController.setSmartMotionMaxAccel(kSmartMotionMaxAccel, 0);
        _elevatorController.setSmartMotionAllowedClosedLoopError(kSmartMotionAllowedError, 0);
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
        _elevatorController.setReference(_setpoint, com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
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

    public void holdPosition() {
        // WPILib

        // SparkMAX
        _elevatorController.setReference(_setpoint, com.revrobotics.CANSparkMax.ControlType.kSmartMotion);
    }

    public void setElevatorSetpoint(double position) {
        switch (_currentState) {
            case HOLD_POSITION:
                setSetpoint(position);
            default:
                break;
        }
    }

    private void setSetpoint(double position) {
        _setpoint = position;
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
            case HOLD_POSITION:
                holdPosition();
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

    public void setHoldPosition() {

    }
}
