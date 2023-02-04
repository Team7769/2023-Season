package frc.robot.Subsystems;

public class PickerUpper extends Subsystem {
    private static PickerUpper _instance = null;

    PickerUpper() {

    }

    public static PickerUpper getInstance() {
        if ( _instance == null ) {
            _instance = new PickerUpper();
        }

        return _instance;
    }

    public void open() {

    }

    public void close() {

    }

    public void collect() {
        
    }

}
