package frc.robot.Utilities;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class LEDController {
    private static Spark _bellyPanBlinkin;
    private static Spark _elevatorBlinkin;
    public static LEDController _instance;

    public final double kBlueHeartBeat = -.23;
    public final double kRedHeartBeat = -.25;
    public final double color1HeartBeat = .07;
    public final double color2HeartBeat = .27;
    public final double bpmParty = -.67;
    public final double kYellow = .69;
    public final double bpmCustom = 0.43;
    public final double sinelonCustom = 0.55;
    public final double colorWavesParty = -0.43;
    public final double kRainbowGlitter = -0.89;
    public final double kFireMedium = -0.59;
    public final double kGoldStrobe = -0.07;
    public final double kBlueStrobe = -0.09;
    public final double kBlueShot = -0.83;
    public final double kConfetti = -0.87;
    public final double kViolet = 0.91;
    public final double kBlueViolet = 0.89;
    public final double kHotPink = 0.57;
    public final double kGold = 0.67;
    public final double kStrobeRed = -0.11;
    public final double kStrobeBlue = -0.09;

    public static LEDController GetInstance()
    {
        if (_instance == null)
        {
            _instance = new LEDController();
        }
        return _instance;
    }

    public void setLowerLED(double value)
    {
        _bellyPanBlinkin.set(value);
    }

    public void setUpperLED(double value)
    {
        _elevatorBlinkin.set(value);
    }

    public void setTeleopIdle()
    {
        _elevatorBlinkin.set(sinelonCustom);
    }

}
