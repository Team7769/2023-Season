// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.NotActiveException;
import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.AutomaticSimpleMatrixConvert;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Automode;
import frc.robot.Configuration.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Subsystem;
import frc.robot.Utilities.Limelight;
import frc.robot.Utilities.PathFollower;
import frc.robot.util.OneDimensionalLookup;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  private static Drivetrain _drivetrain;
  private static PathFollower _pathFollower;
  private XboxController _driverController;
  private int _selectedAutoMode;

  private static Limelight _limelight;

  private List<Subsystem> _subsystems;
  private int _autonomousCase = 0;
  private int _autoLoops = 0;
  private SendableChooser<Integer> _autoChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _drivetrain = Drivetrain.getInstance();
    _pathFollower = PathFollower.getInstance();
    _subsystems = new ArrayList<Subsystem>();
    _subsystems.add(_drivetrain);
    _limelight = Limelight.getInstance();

    PathPlannerServer.startServer(5811);
    _selectedAutoMode = 0;

    _autoChooser.setDefaultOption("Do Nothing", 0);
    _autoChooser.addOption("Loading Side Link + Balance", Automode.LOADING_SIDE_LINK_BALANCE);
    _autoChooser.addOption("Cable Side Two Cone + Balance", Automode.CABLE_SIDE_TWO_CONE_BALANCE);
    _autoChooser.addOption("Loading Side Link + Cone", Automode.LOADING_SIDE_LINK);
    _autoChooser.addOption("Cable Side Link + Balance", Automode.CABLE_SIDE_LINK_BALANCE);

    SmartDashboard.putData(_autoChooser);
  }

  @Override
  public void robotPeriodic() {
    for (var x : _subsystems) {
      x.logTelemetry();
      x.readDashboardData();
    }
    _pathFollower.logData();

    _drivetrain.updateOdomery();
  }

  @Override
  public void autonomousInit() {
    _selectedAutoMode = _autoChooser.getSelected();
    switch (_selectedAutoMode) {
      case Automode.DO_NOTHING:
        break;
      case Automode.LOADING_SIDE_LINK_BALANCE:
        _pathFollower.setLoadsideLinkBalance();
        break;
      case Automode.CABLE_SIDE_LINK_BALANCE:
        break;
      case Automode.LOADING_SIDE_LINK:
        break;
      case Automode.CABLE_SIDE_TWO_CONE_BALANCE:
        _pathFollower.setCableSideTwoConeBalance();
        break;
      default:
        break;
    }

    _drivetrain.resetOdometry();
    switch (_selectedAutoMode) {
      case Automode.LOADING_SIDE_LINK:
      case Automode.LOADING_SIDE_LINK_BALANCE:
      case Automode.CABLE_SIDE_LINK_BALANCE:
      case Automode.CABLE_SIDE_TWO_CONE_BALANCE:
        _drivetrain.initAutonPosition();
        break;
      default:
        break;
    }
  }

  @Override
  public void autonomousPeriodic() {
    switch (_selectedAutoMode) {
      case Automode.LOADING_SIDE_LINK_BALANCE:
        loadSideLinkBalance();
        break;
      case Automode.CABLE_SIDE_TWO_CONE_BALANCE:
        cableSideTwoConeBalanceAuto();
        break;
      default:
        break;
    }

    _autoLoops++;
  }

  public void cableSideTwoConeBalanceAuto() {
    switch (_autonomousCase) {
      case 0:
        _pathFollower.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        if (_autoLoops >= 50) {
          _pathFollower.startPath();
          _autonomousCase++;
        }
        break;
      case 3:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 4:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        if (_autoLoops >= 50) {
          _pathFollower.startPath();
          _autonomousCase++;
        }
        break;
      case 5:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autonomousCase++;
        }
        break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  public void testAuto() {
    switch (_autonomousCase) {
      case 0:
        _pathFollower.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _autonomousCase = 7769;
        }
        break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  public void loadSideLinkBalance() {
    switch (_autonomousCase) {
      case 0:
        _pathFollower.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 2:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        if (_autoLoops >= 50) {
          _pathFollower.startPath();
          _autonomousCase++;
        }
        break;
      case 3:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 4:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        if (_autoLoops >= 50) {
          _pathFollower.startPath();
          _autonomousCase++;
        }
        break;
      case 5:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 6:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        if (_autoLoops >= 50) {
          _pathFollower.startPath();
          _autonomousCase++;
        }
        break;
      case 7:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 8:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        if (_autoLoops >= 50) {
          _pathFollower.startPath();
          _autonomousCase++;
        }
        break;
      case 9:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autonomousCase++;
        }
        break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  public void blueSideTwoConeAuto() {
    switch (_autonomousCase) {
      case 0:
        _pathFollower.startPath();
        _autonomousCase++;
        break;
      case 1:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _autonomousCase = 7769;
          break;
        }
      default:
        // _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    teleopDrive();

    if (_driverController.getStartButtonPressed() && _driverController.getRightBumperPressed()) {
      _drivetrain.resetGyro();
    }

  }

  @Override
  public void disabledInit() {
    // _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
    _pathFollower.setTestAuto();
  }

  @Override
  public void disabledPeriodic() {
    _drivetrain.logPose();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  private void teleopDrive() {
    var translationX = -OneDimensionalLookup.interpLinear(Constants.XY_Axis_inputBreakpoints,
        Constants.XY_Axis_outputTable, _driverController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    var translationY = -OneDimensionalLookup.interpLinear(Constants.XY_Axis_inputBreakpoints,
        Constants.XY_Axis_outputTable, _driverController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    var rotationZ = -OneDimensionalLookup.interpLinear(Constants.RotAxis_inputBreakpoints,
        Constants.RotAxis_outputTable,
        _driverController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_PER_SECOND;

    if (_driverController.getAButtonPressed() || _driverController.getYButtonPressed() || _driverController.getXButtonPressed() || _driverController.getBButtonPressed()){
      _drivetrain.resetWallFacingController();
    }

    if (_driverController.getAButton()) {
      // Turn To The wall facing us
      rotationZ = _drivetrain.getWallRotationTarget(180);
    } else if (_driverController.getYButton()) {
      // Turn to the wall infront of us
      rotationZ = _drivetrain.getWallRotationTarget(0);
    } else if (_driverController.getXButton()) {
      // Turn to the wall left of us
      rotationZ = _drivetrain.getWallRotationTarget(90);
    } else if (_driverController.getBButton()) {
      // Turn to the wall right of us
      rotationZ = _drivetrain.getWallRotationTarget(-90);
    } else {
      rotationZ = rotationZ / 1.25;
    }

    if (_driverController.getLeftBumperPressed()) {
      // Robot orientated speed
      _drivetrain.robotOrientedDrive(translationX, translationY, rotationZ);
    } else {
      // Field orientated speed
      _drivetrain.fieldOrientedDrive(translationX / 1.25, translationY / 1.25, rotationZ);
    }

    SmartDashboard.putNumber("driveControllerTranslationX", translationX);
    SmartDashboard.putNumber("driveControllerTranslationY", translationY);
    SmartDashboard.putNumber("driveControllerRotationZ", rotationZ);
  }
}
