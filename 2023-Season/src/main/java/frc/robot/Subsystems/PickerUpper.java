package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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
    private boolean pizzaReady;

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

        pizzaReady = false;
    }

    public static PickerUpper getInstance() {
        if ( _instance == null ) {
            _instance = new PickerUpper();
        }

        return _instance;
    }

    public void open() {
        _boxer.set(Value.kForward);
    }

    public void close() {
        _boxer.set(Value.kReverse);
    }

    public void collect() {
        _leftMotor.set(_collectSpeed);
        _rightMotor.set(_collectSpeed);
    }

    public void eject() {
        _leftMotor.set(_ejectSpeed);
        _rightMotor.set(_ejectSpeed);
    }

    public void up() {
        _flexer.set(Value.kForward);
    }

    public void down() {
        _flexer.set(Value.kReverse);
    }

    public void stop() {
        _leftMotor.set(0);
        _rightMotor.set(0);
    }
    
   // To be added when we get sensor  
   // public void pizzaReady() {
   //     if(sensor=true){
   //         pizzaReady = true;
   //     }
   // }

    public void handleCurrentState() {
        switch (_currentState) {
            case SHAKE_N_BAKE:
                open();
                collect();
                break;
            case WRONG_ORDER:
                open();
                eject();
                break;
            case BOX_IT:
                close();
                stop();
                break;
            case PIZZA_STOLEN:
                stop();
                break;
            default:
                break;
        }
    }
}
