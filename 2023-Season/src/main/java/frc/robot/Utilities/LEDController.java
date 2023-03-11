package frc.robot.Utilities;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LEDController {
    private static Spark _bellyPanBlinkin;
    private static Spark _elevatorBlinkin;
    private Alliance _alliance;
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
    public final double kRainbowPalette = 0.99;
    public final double kRainbowPartyPalette = -0.97;
    public final double kRainbowOceanPalette = -0.95;
    public final double kRainbowLavePalette = -0.93;
    public final double kRainbowForestPalette = -0.91;
    public final double kRainbowWithGlitter = -0.89;
    public final double kShotRed = -0.85;
    public final double kShotBlue = -0.83;
    public final double kShotWhite = -0.81;
    public final double kSinelonRainbowPalette = -0.79;
    public final double kSinelonPartyPalette = -0.77;
    public final double kSinelonOceanPalette = -0.75;
    public final double kSinelonLavaPalette = -0.73;
    public final double kSinelonForestPalette = -0.71;
    public final double kBeatsPerMinuteRainbowPalette = -0.69;
    public final double kBeatsPerMinutePartyPalette = -0.67;
    public final double kBeatsPerMinuteOceanPalette = -0.65;
    public final double kBeatsPerMinuteLavaPalette = -0.63;
    public final double kBeatsPerMinuteForestPalette = -0.61;
    public final double kFireLarge = -0.57;
    public final double kTwinklesRainbowPalette = -0.55;
    public final double kTwinklesPartyPalette = -0.53;
    public final double kTwinklesOceanPalette = -0.51;
    public final double kTwinklesLavaPalette = -0.49;
    public final double kTwinklesForestPalette = -0.47;
    public final double kColorWavesRainbowPalette = -0.45;
    public final double kColorWavesPartyPalette = -0.43;
    public final double kColorWavesOceanPalette = -0.41;
    public final double kColorWavesLavaPalette = -0.39;
    public final double kColorWavesForestPalette = -0.37;
    public final double kLarsonScannerRed = -0.35;
    public final double kLarsonScannerGray = -0.33;
    public final double kLightChaseRed = -0.31;
    public final double kLightChaseBlue = -0.29;
    public final double kLightChaseGray = -0.27;
    public final double kColor1Strobe = 0.15;
    public final double kColor2Strobe = 0.35;


    public LEDController() {
        _bellyPanBlinkin = new Spark(1);
        _elevatorBlinkin = new Spark(0);
        _alliance = Alliance.Invalid;
    }


    public static LEDController GetInstance()
    {
        if (_instance == null)
        {
            _instance = new LEDController();
        }
        return _instance;
    }

    public void setAlliance() {
        // Only set alliance if the 
        if (_alliance == Alliance.Invalid) {
            _alliance = DriverStation.getAlliance();
        }
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

    public void changeToAlliance() 
    {
        setUpperLED(
            _alliance == Alliance.Red ? kStrobeRed : kStrobeBlue
        );
    }

    public void eject() 
    {
        setUpperLED(kFireMedium);
        setLowerLED(kFireMedium);
    }

    public void ejectHeld()
    {
        setUpperLED(kHotPink);
        setLowerLED(kHotPink);
    }

    public void startHit()
    {
        setUpperLED(kGold);
    }

    public void backHit()
    {
        setUpperLED(kViolet);
    }

    public void yHit()
    {
        setUpperLED(kColor1Strobe);
        setLowerLED(kColor2Strobe);
    }

    public void xHit()
    {
        setLowerLED(kColor1Strobe);
        setUpperLED(kColor2Strobe);
    }

    public void lowHit() {
        setUpperLED(kBeatsPerMinuteOceanPalette);
        setLowerLED(kBeatsPerMinuteOceanPalette);
    }

    public void humanPlayerPickup() {
        setLowerLED(kColorWavesPartyPalette);
    }

    public void teleopInit()
    {
        setLowerLED(bpmCustom);
        setUpperLED(kHotPink);
    }

}
