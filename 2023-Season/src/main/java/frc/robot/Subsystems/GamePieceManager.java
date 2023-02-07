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
        _currentState = GamePieceManagerState.IDLE;
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
            case IDLE:
                invalidState = state != GamePieceManagerState.COLLECTING;
                break;
            case COLLECTING:
                invalidState = state != GamePieceManagerState.TRANSFER;
                break;
            case TRANSFER:
                invalidState = state != GamePieceManagerState.PREPARE_TO_SCORE;
                break;
            case PREPARE_TO_SCORE:
                invalidState = (state != GamePieceManagerState.DEPLOY && state != GamePieceManagerState.EJECT);
                break;
            case DEPLOY:
                invalidState = (state != GamePieceManagerState.SCORING && state != GamePieceManagerState.RESET);
                break;
            case SCORING:
                invalidState = state != GamePieceManagerState.RESET;
                break;
            case RESET:
                invalidState = state != GamePieceManagerState.IDLE;
                break;
            case EJECT:
                invalidState = state != GamePieceManagerState.IDLE;
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
            case IDLE:
                break;
            case COLLECTING:
                break;
            case TRANSFER:
                break;
            case PREPARE_TO_SCORE:
                break;
            case DEPLOY:
                break;
            case SCORING:
                break;
            case RESET:
                break;
            case EJECT:
                break;
            default:
                break;
        }
    }
}
