package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.Configuration.Constants;
import frc.robot.Enums.PickerUpperState;

public class PickerUpper extends Subsystem {
    private static PickerUpper _instance = null;

    private CANSparkMax _leftMotor;
    private CANSparkMax _rightMotor;
    private PickerUpperState _currentState;
    private DoubleSolenoid _boxer;
    private DoubleSolenoid _flexer;
    private Timer _boxItTimer;
    //private boolean pizzaReady;

    private final double _collectSpeed = 0.5;
    private final double _ejectSpeed = -0.5;
    

    PickerUpper() {
        _leftMotor = new CANSparkMax(Constants.kPickerUpperLeftMotorDeviceId, MotorType.kBrushless);
        _rightMotor = new CANSparkMax(Constants.kPickerUpperRightMotorDeviceId, MotorType.kBrushless);

        _leftMotor.setInverted(true);
        _leftMotor.setIdleMode(IdleMode.kBrake);

        _rightMotor.setIdleMode(IdleMode.kBrake);
        _rightMotor.follow(_leftMotor);

        _boxer = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kBoxerForward, Constants.kBoxerReverse);
        _flexer = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.kFlexerForward, Constants.kFlexerReverse);

        _boxItTimer = new Timer();
        //pizzaReady = false;
    }

    public static PickerUpper getInstance() {
        if ( _instance == null ) {
            _instance = new PickerUpper();
        }

        return _instance;
    }

    private void open() {
        _boxer.set(Value.kForward);
    }

    private void close() {
        _boxer.set(Value.kReverse);
    }

    private void collect() {
        _leftMotor.set(_collectSpeed);
        _rightMotor.set(_collectSpeed);
    }

    private void eject() {
        _leftMotor.set(_ejectSpeed);
        _rightMotor.set(_ejectSpeed);
    }

    private void up() {
        _flexer.set(Value.kForward);
    }

    private void down() {
        _flexer.set(Value.kReverse);
    }

    private void stop() {
        _leftMotor.set(0);
        _rightMotor.set(0);
    }

    public void shakeNBake() {
        open();
        collect();

        // If sensor == true set next state BoxIt
    }

    public void boxIt() {
        close();
        if (_boxItTimer.hasElapsed(0.5)) {
            up();
            _boxItTimer.stop();
            setWantedState(PickerUpperState.PIZZAS_READY);
        }
    }
    
   // To be added when we get sensor  
   // public boolean pizzaReady() {
   //     return sensor==true and state==;
   // }

    public void handleCurrentState() {
        switch (_currentState) {
            case SHAKE_N_BAKE:
                shakeNBake();
                break;
            case WRONG_ORDER:
                open();
                eject();
                break;
            case BOX_IT:
                boxIt();
                break;
            case PIZZAS_READY:
                stop();
                break;
            default:
                break;
        }
    }

    public void setWantedState(PickerUpperState state) {
        switch (state) {
            case BOX_IT:
                _boxItTimer.reset();
                _boxItTimer.start();
                break;
            default:
                break;
        }

        _currentState = state;
    }
}
