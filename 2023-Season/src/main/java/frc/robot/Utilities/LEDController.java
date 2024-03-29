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
    public final double kHeartbeatRed = -0.25;
    public final double kHeartbeatBlue = -0.23;
    public final double kHeartbeatWhite = -0.21;
    public final double kHeartbeatGray = -0.19;
    public final double kBreathRed = -0.17;
    public final double kBreathBlue = -0.15;
    public final double kBreathGray = -0.13;
    public final double kStrobeGold = -0.07;
    public final double kStrobeWhite = -0.05;
    public final double kEndtoEndBlendtoBlack = -0.03;
    public final double kLarsonScanner = -0.01;
    public final double kLightChase = 0.01;
    public final double kHeartbeatSlow = 0.03;
    public final double kHeartbeatMedium = 0.05;
    public final double kHeadbeatFast = 0.07;
    public final double kBreathSlow = 0.09;
    public final double kBreathFast = 0.11;
    public final double kShot = 0.13;
    public final double kStrobe = 0.15;
    public final double kSparkleColor1onColor2 = 0.37;
    public final double kSparkleColor2onColor1 = 0.39;
    public final double kColorGradientColor1and2 = 0.41;
    public final double kBeatsPerMinuteColor1on2 = 0.43;
    public final double kEndToEndBlendColor1to2 = 0.45;
    public final double kEndtoEndBlend = 0.47;
    public final double kColor1andColor2NoBlending = 0.49;
    public final double kTwinklesColor1and2 = 0.53;
    public final double kColorWavesColor1and2 = 0.55;
    public final double kDarkRed = 0.59;
    public final double kRed = 0.61;
    public final double kRedOrange = 0.63;
    public final double kOrange = 0.65;
    public final double kLawnGreen = 0.71;
    public final double kLime = 0.73;
    public final double kDarkGreen = 0.75;
    public final double kGreen = 0.77;
    public final double kBlueGreen = 0.79;
    public final double kAqua = 0.81;
    public final double kSkyBlue = 0.83;
    public final double kDarkBlue = 0.85;
    public final double kBlue = 0.87;
    public final double kWhite = 0.93;
    public final double kGray = 0.95;
    public final double kDarkGray = 0.97;
    public final double kBlack = 0.99;


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
            _alliance == Alliance.Red ? kRed : kBlue
        );
    }

    public void eject() 
    {
        setUpperLED(kFireMedium);
        setLowerLED(kFireMedium);
    }

    public void ejectHeld()
    {
        setUpperLED(kRainbowPartyPalette);
        setLowerLED(kColorWavesPartyPalette);
    }

    public void startHit()
    {
        setUpperLED(kGoldStrobe);
    }

    public void backHit()
    {
        setUpperLED(kViolet);
    }

    public void yHit()
    {
        setUpperLED(kRainbowOceanPalette);
        setLowerLED(kRainbowOceanPalette);
    }

    public void xHit()
    {
        setLowerLED(kRainbowForestPalette);
        setUpperLED(kRainbowForestPalette);
    }

    public void lowHit() {
        setUpperLED(kRainbowLavePalette);
        setLowerLED(kRainbowLavePalette);
    }

    public void humanPlayerPickup() {
        setLowerLED(kColorWavesPartyPalette);
    }

    public void teleopInit()
    {
        setLowerLED(bpmCustom);
        setUpperLED(kFireLarge);
    }

    public void anyHuman(){
        setUpperLED(kGreen);
    }

}
