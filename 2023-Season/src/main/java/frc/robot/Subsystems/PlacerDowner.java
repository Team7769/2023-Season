package frc.robot.Subsystems;

public class PlacerDowner extends Subsystem {
    private static PlacerDowner _instance = null;

    PlacerDowner() {

    }

    public static PlacerDowner getInstance() {
        if ( _instance == null ) {
            _instance = new PlacerDowner();
        }

        return _instance;
    }

    public void placeDown() {
        
    }

}
