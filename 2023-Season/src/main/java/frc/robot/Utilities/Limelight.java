package frc.robot.Utilities;

public class Limelight {

    public static Limelight _instance;
    
    public static Limelight getInstance()
    {
        if (_instance == null)
        {
            _instance = new Limelight();
        }

        return _instance;
    }

}
