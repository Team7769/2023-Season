package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Constants;
import frc.robot.Enums.PickerUpperState;
import frc.robot.Lib.Photoeye;

public class PickerUpper extends Subsystem {
    private static PickerUpper _instance = null;

    private CANSparkMax _leftMotor;
    private CANSparkMax _rightMotor;
    private PickerUpperState _currentState;
    private DoubleSolenoid _boxer;
    private DoubleSolenoid _flexer;
    private Timer _boxItTimer;
    private Photoeye _collectorSensor;

    private final double _collectSpeed = 0.65;
    private final double _ejectSpeed = -0.65;
    private final double _deliverySpeed = -0.25;

    private double _manualSpeed = 0.0;
    private Value _manualFlex = Value.kOff;
    private Value _manualBox = Value.kOff;

    PickerUpper() {
        _leftMotor = new CANSparkMax(Constants.kPickerUpperLeftMotorDeviceId, MotorType.kBrushless);
        _leftMotor.setSmartCurrentLimit(5, 100);
        _rightMotor = new CANSparkMax(Constants.kPickerUpperRightMotorDeviceId, MotorType.kBrushless);
        _rightMotor.setSmartCurrentLimit(5, 100);

        _leftMotor.setIdleMode(IdleMode.kBrake);
        _leftMotor.setInverted(false);

        _rightMotor.setIdleMode(IdleMode.kBrake);
        _rightMotor.setInverted(true);

        _boxer = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kBoxerForward, Constants.kBoxerReverse);
        _flexer = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kFlexerForward, Constants.kFlexerReverse);

        _collectorSensor = new Photoeye(Constants.kCollectorPort);

        _boxItTimer = new Timer();
        _currentState = PickerUpperState.PIZZAS_READY;
    }

    public static PickerUpper getInstance() {
        if ( _instance == null ) {
            _instance = new PickerUpper();
        }

        return _instance;
    }
    @Override
    public void logTelemetry() { 
        SmartDashboard.putString("pickerUpperBoxerCurrentState", _boxer.get().name()); 
        SmartDashboard.putString("pickerUpperFlexerCurrentState", _flexer.get().name());
        SmartDashboard.putBoolean("pickerUpperCollectorSensorIsBlocked", _collectorSensor.isBlocked());
        SmartDashboard.putString("pickerUpperCurrentState", _currentState.name());
        SmartDashboard.putNumber("pickerUpperLeftMotorCurrentSpeed", _leftMotor.getAppliedOutput());
        SmartDashboard.putNumber("pickerUpperRightMotorCurrentSpeed", _rightMotor.getAppliedOutput());
        SmartDashboard.putNumber("pickerUpperLeftMotorCurrentTemperature", _leftMotor.getMotorTemperature());
        SmartDashboard.putNumber("pickerUpperRightMotorCurrentTemperature", _rightMotor.getMotorTemperature());
        SmartDashboard.putNumber("pickerUpperLeftMotorOutputCurrent", _leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("pickerUpperRightMotorOutputCurrent", _rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("pickerUppersTimer", _boxItTimer.get());
        
    }
    private void open() {
        _boxer.set(Value.kReverse);
    }

    private void close() {
        _boxer.set(Value.kForward);
    }

    private void collect() {
        _leftMotor.set(_collectSpeed);
        _rightMotor.set(_collectSpeed);
        down();
    }

    private void eject() {
        _leftMotor.set(_ejectSpeed);
        _rightMotor.set(_ejectSpeed);
        down();
    }

    private void up() {
        _flexer.set(Value.kReverse);
    }

    private void down() {
        _flexer.set(Value.kForward);
    }

    private void stop() {
        up();
        _leftMotor.set(0);
        _rightMotor.set(0);
    }

    public void shakeNBake() {
        open();
        collect();

        if (_collectorSensor.isBlocked()){
            setWantedState(PickerUpperState.BOX_IT);
        }
    }

    public void wrongOrder() {
        open();
        eject();
    }

    public void boxIt() {
        if (_boxItTimer.hasElapsed(2)) {
            setWantedState(PickerUpperState.PIZZAS_READY);
        } else if (_boxItTimer.hasElapsed(0.5)) {
            up();
        } else {
            close();
        }

        _leftMotor.set(_collectSpeed);
        _rightMotor.set(_collectSpeed);
    }

    public void delivery() {
        if (_boxItTimer.hasElapsed(2.5)) {
            setWantedState(PickerUpperState.WERE_CLOSED);
            _boxItTimer.stop();
        } else if (_boxItTimer.hasElapsed(2.25)) {
            _leftMotor.set(_deliverySpeed);
            _rightMotor.set(_deliverySpeed);
            open();
        } 

        up();
    }

    public void pizzasReady(){
        stop();
    }

    public void setManualCollect() {
        _manualSpeed = _collectSpeed;
        _manualBox = Value.kReverse;
    }
    public void setManualEject() {
        _manualSpeed = _ejectSpeed;
        _manualBox = Value.kReverse;
    }
    public void setManualStop() {
        _manualSpeed = 0.0;
        _manualBox = Value.kForward;
    }
    public void setManualFlex(Value value) {
        _manualFlex = value;
    }

    private void yeehaw() {
        _leftMotor.set(_manualSpeed);
        _rightMotor.set(_manualSpeed);
        _flexer.set(_manualFlex);
        _boxer.set(_manualBox);
    }
    
    // To be added when we get sensor  
    public boolean isPizzaReady() {
       //return _collectorSensor.isBlocked() && _currentState == PickerUpperState.PIZZAS_READY;
       return _currentState == PickerUpperState.PIZZAS_READY || _currentState == PickerUpperState.DELIVERY;
    }

    public boolean isBoxing() {
        return _currentState == PickerUpperState.BOX_IT;
    }

    public void handleCurrentState() {
        switch (_currentState) {
            case SHAKE_N_BAKE:
                shakeNBake();
                break;
            case WRONG_ORDER:
                wrongOrder();
                break;
            case BOX_IT:
                boxIt();
                break;
            case PIZZAS_READY:
                pizzasReady();
                break;
            case DELIVERY:
                delivery();
                break;
            case YEEHAW:
                yeehaw();
                break;
            case WERE_CLOSED:
                stop();
                break;
            default:
                break;
        }
    }

    public void setWantedState(PickerUpperState state) {
        if (_currentState == PickerUpperState.BOX_IT && state == PickerUpperState.BOX_IT) {
            return;
        }

        switch (state) {
            case BOX_IT:
                _boxItTimer.reset();
                _boxItTimer.start();
                break;
            case PIZZAS_READY:
            case DELIVERY:
                break;
            default:
                if (isBoxing()){
                    return;
                }
                break;
        }

        _currentState = state;
    }
}
