package frc.robot.Subsystems;

import frc.robot.Enums.GamePieceManagerState;

public class GamePieceManager extends Subsystem {
    private static GamePieceManager _instance;
    private static PickerUpper _pickerUpper;
    private static PlacerDowner _placerDowner;

    private GamePieceManagerState _currentState;
    private GamePieceManagerState _previousState;
    
    GamePieceManager() {
        _pickerUpper = PickerUpper.getInstance();
        _placerDowner = PlacerDowner.getInstance();
        _currentState = GamePieceManagerState.HOLDING;
    }

    public static GamePieceManager getInstance() {
        if (_instance == null) {
            _instance = new GamePieceManager();
        }

        return _instance;
    }

    @Override
    public void logTelemetry() {

    }

    @Override
    public void readDashboardData() {

    }

    public GamePieceManagerState getCurrentState() {
        return _currentState;
    }

    public void setState(GamePieceManagerState state) {
        var invalidState = false;

        switch (_currentState) {
            case COLLECTING:
                invalidState = state != GamePieceManagerState.TRANSFER;
                break;
            case TRANSFER:
                invalidState = (state != GamePieceManagerState.COLLECTING && state != GamePieceManagerState.HOLDING);
                break;
            case HOLDING:
                invalidState = (state != GamePieceManagerState.SCORING && state != GamePieceManagerState.COLLECTING);
                break;
            case SCORING:
                invalidState = (state != GamePieceManagerState.COLLECTING && state != GamePieceManagerState.HOLDING);
                break;
            default:
                invalidState = true;
                break;
        }

        if (invalidState) {
            System.out.println("GamePieceManager: Error - Tried moving to state " + state + " from " + _currentState);
        } else {
            _previousState = _currentState;
            _currentState = state;
        }
    }

    public void handle() {
        switch (_currentState) {
            case COLLECTING:
                // Intake
                // Eject
                // Clamp
                // Drop
                // Raise
                break;
            case TRANSFER:
                // Raise Elevator
                // Lower Elevator
                // Intake
                break;
            case HOLDING:
                // Hold piece
                // Eject
                break;
            case SCORING:
                // Lower Elevator
                // Raise Elevator
                // Release
                // Deploy
                // Retract
                break;
            default:
                break;
        }
    }
}
