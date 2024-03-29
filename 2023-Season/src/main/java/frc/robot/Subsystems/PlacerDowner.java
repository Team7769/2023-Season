package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxLimitSwitch.Type;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Configuration.ElevatorPosition;
import frc.robot.Enums.PlacerDownerState;

public class PlacerDowner extends Subsystem {
    private static PlacerDowner _instance = null;

    private CANSparkMax _theClaw;
    private CANSparkMax _placerDownerElevator;
    private RelativeEncoder _placerDownerElevatorEncoder;
    private ProfiledPIDController _placerDownerPID;
    private ElevatorFeedforward _feedForward;
    private DoubleSolenoid _tilter;
    private DoubleSolenoid _pivoter;
    private DigitalInput _elevatorBottomLimit;
    private DigitalInput _tiltLimit;

    private SparkMaxPIDController _elevatorController;

    private PlacerDownerState _currentState = PlacerDownerState.STOW;
    private double _setpoint = ElevatorPosition.DIGIORNO;
    private double _manualElevatorSpeed = 0.0;
    private double _manualClawSpeed = 0.0;
    private Value _manualPivotValue = Value.kOff;
    private Value _manualTiltValue = Value.kOff;

    private final double _placerDownerSpeed = 0.25;
    private final double _placerDownerEjectSpeed = .35;
    private final double _placerDownerHoldSpeed = 0.10;

    private final double kSmartMotionP = 0.015;
    private final double kSmartMotionI = 0.0;
    private final double kSmartMotionD = 0.001;
    private final double kSmartMotionFF = 0.0;
    private final double kSmartMotionIz = 0.0;
    private final double kSmartMotionMaxOutput = 1.00;
    private final double kSmartMotionMinOutput = -1.00;
    private final double kSmartMotionMaxVel = 225;
    private final double kSmartMotionMaxAccel = 150;
    private final double kAllowedError = 3;

    private final TrapezoidProfile.Constraints _constraints = new TrapezoidProfile.Constraints(kSmartMotionMaxVel, kSmartMotionMaxAccel);
    private TrapezoidProfile.State _goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State _profileSetpoint = new TrapezoidProfile.State();

    private boolean hasReset = false;

    PlacerDowner() {
        _theClaw = new CANSparkMax(Constants.kTheClawDeviceId, MotorType.kBrushless);
        _theClaw.setIdleMode(IdleMode.kBrake);
        _theClaw.setInverted(true);
        _theClaw.setSmartCurrentLimit(20, 100);

        _tilter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kTilterForward, Constants.kTilterReverse);
        _pivoter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kPivoterForward, Constants.kPivoterReverse);

        _elevatorBottomLimit = new DigitalInput(1);
        _tiltLimit = new DigitalInput(2);
        _placerDownerElevator = new CANSparkMax(Constants.kPlacerDownerElevatorMotorDeviceId, MotorType.kBrushless);
        _placerDownerElevator.setSmartCurrentLimit(20, 100);
        _placerDownerElevator.setIdleMode(IdleMode.kBrake);
        _placerDownerElevator.setInverted(false);
        _placerDownerElevatorEncoder = _placerDownerElevator.getEncoder();

        // WPILib Controller
        _placerDownerPID = new ProfiledPIDController(Constants.kPlacerDownerKp, Constants.kPlacerDownerKi,
                Constants.kPlacerDownerKd, new TrapezoidProfile.Constraints(2000, 1500));
        _feedForward = new ElevatorFeedforward(Constants.kPlacerDownerFeedforwardKs,
                Constants.kPlacerDownerFeedforwardKg, Constants.kPlacerDownerFeedforwardKv);

        // Spark MAX Smart Motion Parameters
        _elevatorController = _placerDownerElevator.getPIDController();
        _elevatorController.setP(kSmartMotionP);
        _elevatorController.setI(kSmartMotionI);
        _elevatorController.setD(kSmartMotionD);
        _elevatorController.setIZone(kSmartMotionIz);
        _elevatorController.setFF(kSmartMotionFF);
        _elevatorController.setOutputRange(kSmartMotionMinOutput, kSmartMotionMaxOutput);

        //_limitSwitch = _hoodMotor.getForwardLimitSwitch(Type.kNormallyOpen); (Not hood motor but dont know what motor yet)
    }

    public static PlacerDowner getInstance() {
        if (_instance == null) {
            _instance = new PlacerDowner();
        }

        return _instance;
    }

    @Override
    public void logTelemetry() {
        SmartDashboard.putString("placerDownerState", _currentState.name());
        SmartDashboard.putBoolean("placerDownerElevatorAtSetpoint", atSetpoint());
        SmartDashboard.putNumber("placerDownerP", _elevatorController.getP());
        SmartDashboard.putNumber("placerDownerI", _elevatorController.getI());
        SmartDashboard.putNumber("placerDownerD", _elevatorController.getD());
        SmartDashboard.putNumber("placerDownerIZone", _elevatorController.getIZone());
        SmartDashboard.putNumber("placerDownerFF", _elevatorController.getFF());
        SmartDashboard.putNumber("placerDownerOutputRangeMax", _elevatorController.getOutputMax());
        SmartDashboard.putNumber("placerDownerOutputRangeMin", _elevatorController.getOutputMin());
        SmartDashboard.putNumber("placerDownerCurrentPosition", _placerDownerElevatorEncoder.getPosition());
        SmartDashboard.putNumber("placerDownerMotorVoltage", _theClaw.getAppliedOutput());
        SmartDashboard.putNumber("placerDownerMotorTemperature", _theClaw.getMotorTemperature());
        SmartDashboard.putNumber("placerDownerMotorOutputCurrent", _theClaw.getOutputCurrent());
        SmartDashboard.putString("placerDownerTilterCurrentState", _tilter.get().name());
        SmartDashboard.putString("placerDownerPivoterCurrentState", _pivoter.get().name());
        SmartDashboard.putBoolean("placerDownerLimitSwitchBlocked", !_elevatorBottomLimit.get());
        SmartDashboard.putNumber("placerDownerCurrentSetpoint", _setpoint);
        SmartDashboard.putNumber("placerDownerGoal", _goal.position);
        SmartDashboard.putNumber("placerDownerElevatorVoltage", _placerDownerElevator.getAppliedOutput());
        SmartDashboard.putNumber("placerDownerElevatorTemperature", _placerDownerElevator.getMotorTemperature());
        SmartDashboard.putNumber("placerDownerElevatorOutputCurrent", _placerDownerElevator.getOutputCurrent());
        SmartDashboard.putBoolean("placerDownerSteve", isSteve());

    }

    public void handleElevatorReset() {
        if ( hasReset ) return;

        if(!_elevatorBottomLimit.get()){
            _placerDownerElevatorEncoder.setPosition(0);
            hasReset = true;
        }
    }

    public boolean isSteve() {
        return (!_tiltLimit.get());
    }

    public boolean transferComplete(){
        return (atSetpoint() && _setpoint == ElevatorPosition.PIZZA_DELIVERY);    
    }

    public void allowReset() {
        hasReset = false;
    }

    private void intake() {
        setElevatorSetpoint(ElevatorPosition.PIZZA_DELIVERY);
        _theClaw.set(_placerDownerSpeed);
        handleElevatorPosition();
        _pivoter.set(Value.kForward);

        if (atSetpoint()) {
            setWantedState(PlacerDownerState.HOLD_POSITION);
        }
    }

    private void eject() {
        _theClaw.set(-_placerDownerEjectSpeed);
    }

    private void stop() {
        _theClaw.set(0);
    }

    private void deploy() {
        _tilter.set(Value.kReverse);
        _pivoter.set(Value.kReverse);

        if (isSteve()) {
            handleElevatorPosition();
        }
    }

    private void retract() {
        handleElevatorPosition();

        if (_placerDownerElevatorEncoder.getPosition() <= 50) {
            _tilter.set(Value.kForward);
            _pivoter.set(Value.kForward);

            setWantedState(PlacerDownerState.HOLD_POSITION);
        }        
    }

    private void holdPosition() {
        handleElevatorPosition();
        _theClaw.set(_placerDownerHoldSpeed);
    }

    private void reset() {
        setElevatorSetpoint(ElevatorPosition.DIGIORNO);
        setWantedState(PlacerDownerState.STOW);
    }

    private void resetWithoutTilt(){
        _pivoter.set(Value.kForward);
        setElevatorSetpoint(ElevatorPosition.DIGIORNO);
        setWantedState(PlacerDownerState.HOLD_POSITION);
    }

    public void setManualElevatorSpeed(double speed) {
        if (Math.abs(speed) <= 0.10) {
            speed = 0.0;
        }

        _manualElevatorSpeed = speed;
    }
    public void setManualIntake() {
        _manualClawSpeed = _placerDownerSpeed;
    }
    public void setManualEject() {
        _manualClawSpeed = -_placerDownerSpeed;
    }
    public void setManualStop() {
        _manualClawSpeed = 0.0;
    }
    public void setTiltValue(Value value) {
        _manualTiltValue = value;
    }
    public void setPivotValue(Value value) {
        _manualPivotValue = value;
    }

    private void yeehaw() {
        _placerDownerElevator.set(_manualElevatorSpeed);
        _theClaw.set(_manualClawSpeed);
        _pivoter.set(_manualPivotValue);
        _tilter.set(_manualTiltValue);
    }

    private void flipoff() {
        handleElevatorPosition();
        if (atSetpoint()) {
            _pivoter.set(Value.kReverse);
            setWantedState(PlacerDownerState.HOLD_POSITION);
        }
    }

    public boolean isScoring() {
        switch (_currentState) {
            case DEPLOY:
                if (isSteve()) {
                    return true;
                } else {
                    return false;
                }
            case LOW_SCORE:
                return true;
            default:
                return false;
        }
    }

    public void setElevatorSetpoint(double position) {
        // if (_tilter.get() != Value.kForward) {
        //     position = _setpoint;
        // }

        switch (_currentState) {
            case RESET:
            case INTAKE:
            case HOLD_POSITION:
            case DEPLOY:
            case STOW:
            case RESET_WITHOUT_TILT:
                setSetpoint(position);
            default:
                break;
        }
    }

    public boolean atSetpoint() {
        var currentPosition = _placerDownerElevatorEncoder.getPosition();
        var error = _setpoint - currentPosition;

        return Math.abs(error) <= kAllowedError;
    }

    private void handleElevatorPosition() {
        // WPILib
        // _placerDownerPID.setGoal(_setpoint);
        // var pidOutput =
        // _placerDownerPID.calculate(_placerDownerElevatorEncoder.getPosition());
        // var feedforward =
        // _feedForward.calculate(_placerDownerPID.getSetpoint().velocity);
        // _placerDownerElevator.setVoltage(pidOutput + feedforward);

        // SparkMAX
        var profile = new TrapezoidProfile(_constraints, _goal, _profileSetpoint);

        _profileSetpoint = profile.calculate(0.02);
        _elevatorController.setReference(_profileSetpoint.position, com.revrobotics.CANSparkMax.ControlType.kPosition, 0,
                _feedForward.calculate(_profileSetpoint.velocity));
    }

    private void setSetpoint(double position) {
        // WPILib
        _setpoint = position;

        // SparkMAX
        _goal = new TrapezoidProfile.State(position, 0);
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
            case RESET:
                reset();
                break;
            case YEEHAW:
                yeehaw();
                break;
            case LOW_SCORE:
                flipoff();
                break;
            case RESET_WITHOUT_TILT:
                resetWithoutTilt();
                break;
            case STOP:
            default:
                stop();
                break;
        }
    }

    public void setWantedState(PlacerDownerState wantedState) {
        switch (_currentState) {
            case INTAKE:
                if (wantedState == PlacerDownerState.INTAKE) {
                    return;
                }
                break;
            case DEPLOY:
                if (wantedState == PlacerDownerState.DEPLOY) {
                    return;
                }
                break;
            default:
                break;
        }
        
        _currentState = wantedState;
    }
}
