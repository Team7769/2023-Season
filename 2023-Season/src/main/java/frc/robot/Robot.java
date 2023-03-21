// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.NotActiveException;
import java.util.ArrayList;
import java.util.List;

import org.ejml.simple.AutomaticSimpleMatrixConvert;

import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configuration.Automode;
import frc.robot.Configuration.Constants;
import frc.robot.Configuration.ElevatorPosition;
import frc.robot.Enums.PlacerDownerState;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.GamePieceManager;
import frc.robot.Subsystems.PlacerDowner;
import frc.robot.Subsystems.PickerUpper;
import frc.robot.Enums.PickerUpperState;
import frc.robot.Subsystems.Subsystem;
import frc.robot.Utilities.LEDController;
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
  private static GamePieceManager _gamePieceManager;
  private static PlacerDowner _placerDowner;
  private static PickerUpper _pickerUpper;
  private static PathFollower _pathFollower;
  private static LEDController _ledController;
  private XboxController _driverController;
  private XboxController _operatorController;
  private int _selectedAutoMode;

  private static Limelight _limelight;

  private List<Subsystem> _subsystems;
  private int _autonomousCase = 0;
  private int _autoLoops = 0;
  private SendableChooser<Integer> _autoChooser = new SendableChooser<>();
  private boolean _ejectHeld = false;
  private Compressor _compressor; 
  @Override
  public void robotInit() {
    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _operatorController = new XboxController(Constants.kOperatorControllerUsbSlot);
    _drivetrain = Drivetrain.getInstance();
    _pathFollower = PathFollower.getInstance();
    _gamePieceManager = GamePieceManager.getInstance();
    _placerDowner = PlacerDowner.getInstance();
    _pickerUpper = PickerUpper.getInstance();
    _subsystems = new ArrayList<Subsystem>();
    _subsystems.add(_drivetrain);
    _subsystems.add(_pickerUpper);
    _subsystems.add(_placerDowner);
    _limelight = Limelight.getInstance();
     _compressor = new Compressor(1, PneumaticsModuleType.REVPH);
     _compressor.enableDigital();
    //PathPlannerServer.startServer(5811);
    _selectedAutoMode = 0;

    _ledController = LEDController.GetInstance();

    _autoChooser.setDefaultOption("Do Nothing", 0);
    // _autoChooser.addOption("Loading Side Link + Balance", Automode.LOADING_SIDE_LINK_BALANCE);
    // _autoChooser.addOption("Cable Side Two Cone + Balance", Automode.CABLE_SIDE_TWO_CONE_BALANCE);
    // _autoChooser.addOption("Cable Side Link + Balance", Automode.CABLE_SIDE_LINK_BALANCE);
    // _autoChooser.addOption("Loading Side Link", Automode.LOADING_SIDE_LINK_NOBALANCE);
    // _autoChooser.addOption("Loading Side Link + Cone", Automode.LOADING_SIDE_LINK_CONE_NOBALANCE);
    _autoChooser.addOption("Loading Side Pickup + Balance", Automode.LOADING_SIDE_PICKUP_BALANCE);
    _autoChooser.addOption("Loading Side Pickup + Score", Automode.LOADING_SIDE_PICKUP_SCORE);
    _autoChooser.addOption("Loading Side Pickup + Score 3 Pieces", Automode.LOADING_SIDE_PICKUP_SCORE_THREE);
    _autoChooser.addOption("Loading Side Pickup + Score Mid Link", Automode.LOADING_SIDE_PICKUP_SCORE_MID_LINK);
    _autoChooser.addOption("YEET", Automode.MIDDLE_YEET_BALANCE);

    SmartDashboard.putData(_autoChooser);
  }

  @Override
  public void robotPeriodic() {
    for (var x : _subsystems) {
      x.logTelemetry();
      //x.readDashboardData();
    }
    //_pathFollower.logData();

    _drivetrain.updateOdometry();
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
        _pathFollower.setCableSideLinkBalance();
        break;
      case Automode.CABLE_SIDE_TWO_CONE_BALANCE:
        _pathFollower.setCableSideTwoConeBalance();
        break;
      case Automode.LOADING_SIDE_LINK_NOBALANCE:
        _pathFollower.setLoadingSideLinkNoBalance();
        break;
      case Automode.LOADING_SIDE_LINK_CONE_NOBALANCE:
        _pathFollower.setLoadingSideLinkConeNoBalance();
        break;
      case Automode.LOADING_SIDE_PICKUP_BALANCE:
      case Automode.MIDDLE_YEET_BALANCE:
        _pathFollower.setLoadsidePickupBalance();
        break;
      case Automode.LOADING_SIDE_PICKUP_SCORE:
        _pathFollower.setLoadsidePickupScore();
        break;
      case Automode.LOADING_SIDE_PICKUP_SCORE_THREE:
        _pathFollower.setLoadsidePickupScoreThree();
        break;
      case Automode.LOADING_SIDE_PICKUP_SCORE_MID_LINK:
        _pathFollower.setLoadsidePickupScoreMidLink();
        break;
      default:
        break;
    }

    _drivetrain.resetOdometry();
    switch (_selectedAutoMode) {
      case Automode.LOADING_SIDE_LINK_BALANCE:
      case Automode.CABLE_SIDE_LINK_BALANCE:
      case Automode.CABLE_SIDE_TWO_CONE_BALANCE:
      case Automode.LOADING_SIDE_LINK_NOBALANCE:
      case Automode.LOADING_SIDE_LINK_CONE_NOBALANCE:
      case Automode.LOADING_SIDE_PICKUP_BALANCE:
      case Automode.MIDDLE_YEET_BALANCE:
      case Automode.LOADING_SIDE_PICKUP_SCORE:
      case Automode.LOADING_SIDE_PICKUP_SCORE_THREE:
      case Automode.LOADING_SIDE_PICKUP_SCORE_MID_LINK:
        _drivetrain.initAutonPosition();
        _placerDowner.setWantedState(PlacerDownerState.HOLD_POSITION);
        _placerDowner.setElevatorSetpoint(ElevatorPosition.PIZZA_DELIVERY);
        _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
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
      case Automode.CABLE_SIDE_LINK_BALANCE:
        cableSideLinkBalance();
        break;
      case Automode.LOADING_SIDE_LINK_NOBALANCE:
        loadingSideLinkNoBalance();
        break;
      case Automode.LOADING_SIDE_LINK_CONE_NOBALANCE:
        loadingSideLinkConeNoBalance();
        break;
      case Automode.LOADING_SIDE_PICKUP_BALANCE:
        loadSidePickupBalance();
        break;
      case Automode.LOADING_SIDE_PICKUP_SCORE:
        loadSidePickupScore();
        break;
      case Automode.LOADING_SIDE_PICKUP_SCORE_MID_LINK:
          loadSidePickupScoreThreeMid();
          break;
      case Automode.MIDDLE_YEET_BALANCE:
        yeet();
        break;
      default:
        break;
    }

    if (_pickerUpper.isPizzaReady()) {
      _placerDowner.setWantedState(PlacerDownerState.INTAKE);
      _pickerUpper.setWantedState(PickerUpperState.DELIVERY);
    } else if (_placerDowner.isScoring()) {
      _pickerUpper.setWantedState(PickerUpperState.RELEASE);
    }

    _pickerUpper.handleCurrentState();
    _placerDowner.handleCurrentState();
    incrementAutoLoopTimer();
  }

  public void incrementAutoLoopTimer() {
    _autoLoops++;
  }

  public void resetAutoLoopTimer() {
    _autoLoops = 0;
  }

  /**
   * Determines if a timeframe has elapsed.
   * 50 loops = 1 second.
   * @param seconds
   * @return
   */
  public boolean autoHasElapsed(double seconds) {
    return ((double)_autoLoops / 50) >= seconds;
  }

  public void nextAutoStep() {
    _autonomousCase++;
  }

  public void goToAutoStep(int step) {
    _autonomousCase = step;
  }

  public void loadSidePickupBalance() {
    switch (_autonomousCase) {
      case 0:
        // Init Elevator 
        _placerDowner.setElevatorSetpoint(ElevatorPosition.JETS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        _autonomousCase++;
        break;
      case 1:
        // If elevator is fully extended.
        if (_placerDowner.atSetpoint()) {

          // Wait 1 second then stop and eject.
          if (_autoLoops >= 50) {            
            _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
            _placerDowner.setWantedState(PlacerDownerState.EJECT);
            _autoLoops = 0;
            _autonomousCase++;
          } else if (_autoLoops >= 15 && _autoLoops < 50){
            // Wait .5 seconds, then drive for .5 seconds
            _drivetrain.fieldOrientedDrive(-0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);
          }
        } else {
          // Don't move until the elevator is extended.
          _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
          _autoLoops = 0;
        }
        break;
      case 2:
        // Stop
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);

        // Wait .5 seconds, then start the elevator reset.
        if (_autoLoops > 25) {
          _drivetrain.resetWallFacingController();
          _placerDowner.setWantedState(PlacerDownerState.RESET);
          _autoLoops = 0;
          _autonomousCase++;
        }
        break;
      case 3:
        
        if (_autoLoops > 100) {
          // Start path to cone/cube after 2 full seconds
          _autoLoops = 0;
          _pathFollower.startPath();
          _autonomousCase++;
        } else if (_autoLoops <= 25) {
          // Back away for .5 seconds.
          _drivetrain.fieldOrientedDrive(0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(180));
        } else {
          // Stop for 1.5 seconds
          _drivetrain.fieldOrientedDrive(0, 0, 0);
        }
        break;
      case 4:
        _drivetrain.followTrajectory();
        if (_autoLoops > 55 && _autoLoops <= 60) {
          // After around 1 second drop the collector
          _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
        }

        if (_pathFollower.isPathFinished()) {
          // Set next path, prepare for pickup
          _drivetrain.resetWallFacingController();
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 5:
        if (_autoLoops > 40) {
          // Start the path to the charge station.
          _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
          _pathFollower.startPath();
          _autonomousCase++;
          _autoLoops = 0;
        } else {
          // Drive slowly towards the gamepiece for 1.5 seconds
          _drivetrain.fieldOrientedDrive(0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(0));
        }
        break;
      case 6:
        // Follow path to charge station.
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          // Prepare to balance walk
          _drivetrain.resetWallFacingController();
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autonomousCase++;
          _autoLoops = 0;
        }
        break;
      case 7:
        // For at least 2 seconds, drive forward until the measured roll is considered balanced.
        if (_autoLoops <= 135) {
          _drivetrain.fieldOrientedDrive(-0.18 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(180));
        }
        else {
          // Otherwise stop and turn the wheels very slightly to lock position.
          var speed = _drivetrain.getBalanceSpeed();
          _drivetrain.fieldOrientedDrive(speed * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(180));

          if (_drivetrain.isLevel()) {
            _ledController.changeToAlliance();          
          }
        }
        break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  public void yeet() {
    switch (_autonomousCase) {
      case 0:
        // Init Elevator 
        _placerDowner.setElevatorSetpoint(ElevatorPosition.JETS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        _autoLoops = 0;
        _autonomousCase++;
        break;
      case 1:
        // If elevator is fully extended.
        if (_placerDowner.atSetpoint()) {
          // Wait 1 second then stop and eject.
          if (_autoLoops >= 50) {            
            _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
            _placerDowner.setWantedState(PlacerDownerState.EJECT);
            _autoLoops = 0;
            _autonomousCase++;
          } else if (_autoLoops >= 15 && _autoLoops < 50){
            // Wait .5 seconds, then drive for .5 seconds
            _drivetrain.fieldOrientedDrive(-0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);
          }
        } else {
          // Don't move until the elevator is extended.
          _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
          _autoLoops = 0;
        }
        break;
      case 2:
        // Stop
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);

        // Wait .5 seconds, then start the elevator reset.
        if (_autoLoops > 25) {
          _drivetrain.resetWallFacingController();
          _placerDowner.setWantedState(PlacerDownerState.RESET);
          _autoLoops = 0;
          _autonomousCase++;
        }
        break;
      case 3:
        if (_autoLoops > 100) {
          // Start path to cone/cube after 2 full seconds
          _autoLoops = 0;
          _drivetrain.resetWallFacingController();
          _autonomousCase++;
        } else if (_autoLoops <= 25) {
          // Back away for .5 seconds.
          _drivetrain.fieldOrientedDrive(0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(180));
        } else {
          // Stop for 1.5 seconds
          _drivetrain.fieldOrientedDrive(0, 0, 0);
        }
        break;
      case 4:
        _drivetrain.fieldOrientedDrive(0.25 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(180));

        if (_autoLoops > 200) {
          // Set next path, prepare for pickup
          _drivetrain.resetWallFacingController();
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _autonomousCase++;
        }
        break;
      case 5:
        // For at least 2 seconds, drive forward until the measured roll is considered balanced.
        if (_autoLoops <= 150) {
          _drivetrain.fieldOrientedDrive(-0.18 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(180));
        }
        else {
          // Otherwise stop and turn the wheels very slightly to lock position.
          var speed = _drivetrain.getBalanceSpeed();
          _drivetrain.fieldOrientedDrive(speed * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(180));

          if (_drivetrain.isLevel()) {
            _ledController.changeToAlliance();          
          }
        }

        break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  public void loadSidePickupScore() {
    switch (_autonomousCase) {
      case 0:
        // Init Elevator
        _placerDowner.setElevatorSetpoint(ElevatorPosition.JETS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        _autonomousCase++;
        break;
      case 1:
        // If elevator is fully extended.
        if (_placerDowner.atSetpoint()) {

          // Wait 1 second then stop and eject.
          if (_autoLoops >= 50) {
            _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
            _placerDowner.setWantedState(PlacerDownerState.EJECT);
            _autoLoops = 0;
            _autonomousCase++;
          } else if (_autoLoops >= 15 && _autoLoops < 50) {
            // Wait .5 seconds, then drive for .5 seconds
            _drivetrain.fieldOrientedDrive(-0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);
          }
        } else {
          // Don't move until the elevator is extended.
          _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
          _autoLoops = 0;
        }
        break;
      case 2:
        // Stop
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);

        // Wait .5 seconds, then start the elevator reset.
        if (_autoLoops > 25) {
          _drivetrain.resetWallFacingController();
          _placerDowner.setWantedState(PlacerDownerState.RESET);
          _autoLoops = 0;
          _autonomousCase++;
        }
        break;
      case 3:

        if (_autoLoops > 100) {
          // Start path to cone/cube after 2 full seconds
          _autoLoops = 0;
          _pathFollower.startPath();
          _autonomousCase++;
        } else if (_autoLoops <= 25) {
          // Back away for .5 seconds.
          _drivetrain.fieldOrientedDrive(0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0,
              _drivetrain.getWallRotationTarget(180));
        } else {
          // Stop for 1.5 seconds
          _drivetrain.fieldOrientedDrive(0, 0, 0);
        }
        break;
      case 4:
        _drivetrain.followTrajectory();
        if (_autoLoops > 55 && _autoLoops <= 60) {
          // After around 1 second drop the collector
          _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
        }

        if (_pathFollower.isPathFinished()) {
          // Set next path, prepare for pickup
          _drivetrain.resetWallFacingController();
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 5:
        if (_autoLoops > 40) {
          // Start the path back to the grid.
          _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
          _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
          _pathFollower.startPath();
          _autonomousCase++;
          _autoLoops = 0;
        } else {
          // Drive slowly towards the gamepiece for 1.5 seconds
          _drivetrain.fieldOrientedDrive(0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0,
              _drivetrain.getWallRotationTarget(0));
        }
        break;
      case 6:
        // Follow path to the grid.
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          // Prepare to score
          _drivetrain.resetWallFacingController();
          _drivetrain.robotOrientedDrive(0, 0, 0);
          //_autonomousCase++;
          _autonomousCase = 7769; // No faith
          _autoLoops = 0;
        }
        break;
      case 7:
        // Init Elevator
        _placerDowner.setElevatorSetpoint(ElevatorPosition.JETS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        _autonomousCase++;
        break;
      case 8:
        // If elevator is fully extended.
        if (_placerDowner.atSetpoint()) {

          // Wait 1 second then stop and eject.
          if (_autoLoops >= 50) {
            _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
            _placerDowner.setWantedState(PlacerDownerState.EJECT);
            _autoLoops = 0;
            _autonomousCase++;
          } else if (_autoLoops >= 15 && _autoLoops < 50) {
            // Wait .5 seconds, then drive for .5 seconds
            _drivetrain.fieldOrientedDrive(-0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);
          }
        } else {
          // Don't move until the elevator is extended.
          _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
          _autoLoops = 0;
        }
        break;
      case 9:
        // Stop
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);

        // Wait .5 seconds, then start the elevator reset.
        if (_autoLoops > 25) {
          _drivetrain.resetWallFacingController();
          _placerDowner.setWantedState(PlacerDownerState.RESET);
          _autoLoops = 0;
          _autonomousCase++;
        }
        break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  public void loadSidePickupScoreThree() {
    switch (_autonomousCase) {
      case 0:
        // Init Elevator
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        _placerDowner.setElevatorSetpoint(ElevatorPosition.BUDDYS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        nextAutoStep();
        break;
      case 1:
        // If elevator is fully extended, eject gamepiece
        if (_placerDowner.atSetpoint()) {
          _placerDowner.setWantedState(PlacerDownerState.EJECT);
          nextAutoStep();
        }

        // Don't move
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        break;
      case 2:
        // Stop
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        _placerDowner.setWantedState(PlacerDownerState.RESET);
        _pathFollower.startPath();
        resetAutoLoopTimer();
        nextAutoStep();
        break;
      case 3:
        _drivetrain.followTrajectory();
        if (autoHasElapsed(.5) && !autoHasElapsed(.55)) {
          // After half a second drop the collector
          _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
        }

        if (_pathFollower.isPathFinished()) {
          // Set next path, prepare for pickup
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _pathFollower.setNextPath();
          resetAutoLoopTimer();
          nextAutoStep();
        }
        break;
      case 4:
        // Transfer game piece
        // if (autoHasElapsed(2)) {
        //   // Start the path back to the grid.
        //   _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
        //   _pathFollower.startPath();
        //   resetAutoLoopTimer();
        //   nextAutoStep();
        // }
        
          // Start the path back to the grid.
          _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
          _pathFollower.startPath();
          resetAutoLoopTimer();
          nextAutoStep();

        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        break;
      case 5:
        // Follow path to the grid.
        _drivetrain.followTrajectory();
        
        // Bring elevator to scoring position.
        if (autoHasElapsed(1.5) && !autoHasElapsed(1.55)) {
          _placerDowner.setElevatorSetpoint(ElevatorPosition.SHIELDS);
          _placerDowner.setWantedState(PlacerDownerState.LOW_SCORE);
        }

        if (_pathFollower.isPathFinished()) {
          _pathFollower.setNextPath();
          _drivetrain.robotOrientedDrive(0, 0, 0);
          resetAutoLoopTimer();
          nextAutoStep();
        }
        break;
      case 6:
        // Score Cube
        if (_placerDowner.atSetpoint()) {
          _placerDowner.setWantedState(PlacerDownerState.EJECT);
          nextAutoStep();
        }

        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        break;
      case 7:
        // Reset
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        _placerDowner.setWantedState(PlacerDownerState.RESET);
        resetAutoLoopTimer();
        _pathFollower.startPath();
        nextAutoStep();
        break;
      case 8:
      _drivetrain.followTrajectory();
      if (autoHasElapsed(.5) && !autoHasElapsed(.55)) {
        _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
      }

      if (_pathFollower.isPathFinished()) {
        // Set next path, prepare for pickup
        _drivetrain.robotOrientedDrive(0, 0, 0);
        _pathFollower.setNextPath();
        nextAutoStep();
      }
      break;
    case 9:
        // Start the path back to the grid.
        _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        _pathFollower.startPath();
        resetAutoLoopTimer();
        nextAutoStep();
      break;
    case 10:
      // Follow path to the grid.
      _drivetrain.followTrajectory();
      if (autoHasElapsed(1.5) && !autoHasElapsed(1.55)) {
        _placerDowner.setElevatorSetpoint(ElevatorPosition.SHIELDS);
        _placerDowner.setWantedState(PlacerDownerState.LOW_SCORE);
      }

      if (_pathFollower.isPathFinished()) {
        // Prepare to score
        _drivetrain.robotOrientedDrive(0, 0, 0);
        nextAutoStep();
      }
      break;
    case 11:
      // If elevator is fully extended.
      if (_placerDowner.atSetpoint()) {
        _placerDowner.setWantedState(PlacerDownerState.EJECT);
        nextAutoStep();
      }
      _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
      break;
    case 12:
      // Stop
      _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
      _placerDowner.setWantedState(PlacerDownerState.RESET);
      nextAutoStep();
      break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  public void loadSidePickupScoreThreeMid() {
    switch (_autonomousCase) {
      case 0:
        // Init Elevator
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        _placerDowner.setElevatorSetpoint(ElevatorPosition.BUDDYS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        nextAutoStep();
        break;
      case 1:
        // If elevator is fully extended, eject gamepiece
        if (_placerDowner.atSetpoint()) {
          _placerDowner.setWantedState(PlacerDownerState.EJECT);
          resetAutoLoopTimer();
          nextAutoStep();
        }

        // Don't move
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        break;
      case 2:
        _placerDowner.setWantedState(PlacerDownerState.RESET);
        nextAutoStep();
        break;
      case 3:
        // Stop
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        if (autoHasElapsed(1.5)) {
          _pathFollower.startPath();
          resetAutoLoopTimer();
          nextAutoStep();
        }
        break;
      case 4:
        _drivetrain.followTrajectory();
        if (autoHasElapsed(.25) && !autoHasElapsed(.30)) {
          // After half a second drop the collector
          _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
        }

        if (_pathFollower.isPathFinished()) {
          // Set next path, prepare for pickup
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _pathFollower.setNextPath();
          resetAutoLoopTimer();
          _drivetrain.resetWallFacingController();
          nextAutoStep();
        }
        break;
      case 5:
        // Transfer game piece
        if (autoHasElapsed(.75)) {
          // Start the path back to the grid.
          _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
          _pathFollower.startPath();
          resetAutoLoopTimer();
          nextAutoStep();
        } else {
          _drivetrain.fieldOrientedDrive(0.1 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, _drivetrain.getWallRotationTarget(0));
        }

        
          // // Start the path back to the grid.
          // _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
          // _pathFollower.startPath();
          // resetAutoLoopTimer();
          // nextAutoStep();
        break;
      case 6:
        // Follow path to the grid.
        _drivetrain.followTrajectory();
        
        // Bring elevator to scoring position.
        if (autoHasElapsed(1.5) && !autoHasElapsed(1.55)) {
          _placerDowner.setElevatorSetpoint(ElevatorPosition.BUDDYS);
          _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        }

        if (_pathFollower.isPathFinished()) {
          _pathFollower.setNextPath();
          _drivetrain.robotOrientedDrive(0, 0, 0);
          resetAutoLoopTimer();
          nextAutoStep();
        }
        break;
      case 7:
        // Score Cube
        if (_placerDowner.atSetpoint()) {
          _placerDowner.setWantedState(PlacerDownerState.EJECT);
          nextAutoStep();
        }

        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        break;
      case 8:
        _placerDowner.setWantedState(PlacerDownerState.RESET);
        nextAutoStep();
        break;
      case 9:
        // Reset
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        
        if (autoHasElapsed(1.5)) {
          _pathFollower.startPath();
          resetAutoLoopTimer();
          nextAutoStep();
        }
        break;
      case 10:

      _drivetrain.followTrajectory();
      if (autoHasElapsed(1) && !autoHasElapsed(1.05)) {
        _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
      }

      if (_pathFollower.isPathFinished()) {
        // Set next path, prepare for pickup
        _drivetrain.robotOrientedDrive(0, 0, 0);
        _pathFollower.setNextPath();
        nextAutoStep();
      }
      break;
    case 11:
        // Start the path back to the grid.
        _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
        _pathFollower.startPath();
        resetAutoLoopTimer();
        nextAutoStep();
      break;
    case 12:
      // Follow path to the grid.
      _drivetrain.followTrajectory();
      // if (autoHasElapsed(1.5) && !autoHasElapsed(1.55)) {
      //   _placerDowner.setElevatorSetpoint(ElevatorPosition.SHIELDS);
      //   _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
      // }

      if (_pathFollower.isPathFinished()) {
        // Prepare to score
        _drivetrain.robotOrientedDrive(0, 0, 0);
        //nextAutoStep();
        goToAutoStep(7769);
      }
      break;
    case 13:
      // If elevator is fully extended.
      if (_placerDowner.atSetpoint()) {
        _placerDowner.setWantedState(PlacerDownerState.EJECT);
        nextAutoStep();
      }
      _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
      break;
    case 14:
      // Stop
      _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);
      _placerDowner.setWantedState(PlacerDownerState.RESET);
      nextAutoStep();
      break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  @Override
  public void teleopInit() {
    _ledController.teleopInit();
  }

  @Override
  public void teleopPeriodic() {
    teleopDrive();
    teleopGamePieceManagement();

    if (_driverController.getStartButtonPressed()) {
      _drivetrain.resetGyro();
    }
  }

  @Override
  public void disabledInit() {
    // _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
    _pathFollower.setTestAuto();
    _placerDowner.allowReset();
  }

  @Override
  public void disabledPeriodic() {
    // _drivetrain.logPose();
    _drivetrain.robotOrientedDrive(0, 0, 0);
    _placerDowner.handleElevatorReset();
    _ledController.setAlliance();
  }

  @Override
  public void testInit() {
    _placerDowner.setWantedState(PlacerDownerState.YEEHAW);
    _pickerUpper.setWantedState(PickerUpperState.YEEHAW);
  }

  @Override
  public void testPeriodic() {
    teleopDrive();
    testPeriodicPickerUpper();
    testPeriodicPlacerDowner();
  }

  private void testPeriodicPlacerDowner() {
    _placerDowner.setManualElevatorSpeed(-_operatorController.getLeftY());

    if (_operatorController.getLeftBumper()) {
      _placerDowner.setTiltValue(Value.kForward);
    } else if (_operatorController.getRightBumper()) {
      _placerDowner.setTiltValue(Value.kReverse);
    }

    if (_operatorController.getAButton()){
      _placerDowner.setManualIntake();
    } else if (_operatorController.getBButton()){
      _placerDowner.setManualEject();
    } else {
      _placerDowner.setManualStop();
    }

    if (Math.abs(_operatorController.getLeftTriggerAxis()) > 0.25) {
      _placerDowner.setPivotValue(Value.kForward);
    } else if (Math.abs(_operatorController.getRightTriggerAxis()) > 0.25) {
      _placerDowner.setPivotValue(Value.kReverse);
    }

    _placerDowner.handleCurrentState();
  }

  private void testPeriodicPickerUpper() {

    if (_driverController.getLeftBumper()) {
      _pickerUpper.setManualCollect();
    } else if (_driverController.getRightBumper()) {
      _pickerUpper.setManualEject();
    } else {
      _pickerUpper.setManualStop();
    }

    if (Math.abs(_driverController.getLeftTriggerAxis()) > 0.25) {
      _pickerUpper.setManualFlex(Value.kForward);
    } else if (Math.abs(_driverController.getRightTriggerAxis()) > 0.25) {
      _pickerUpper.setManualFlex(Value.kReverse);
    }

    _pickerUpper.handleCurrentState();
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  private void teleopGamePieceManagement() {
    if ( _operatorController.getStartButton() && _operatorController.getBackButton() ) {
      _ledController.anyHuman();
    }else if ( _operatorController.getBackButton() ) {
      _ledController.backHit();
    }
    else if (_operatorController.getStartButton()){
      _ledController.startHit();
    }

    if (_operatorController.getYButton()) {
      _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
      _placerDowner.setElevatorSetpoint(ElevatorPosition.JETS);

      _ledController.yHit();
    } else if (_operatorController.getXButton()) {
      _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
      _placerDowner.setElevatorSetpoint(ElevatorPosition.SHIELDS);
      _ledController.xHit();
    } else if (_operatorController.getBButton()) {
      _placerDowner.setWantedState(PlacerDownerState.STOW);
      _placerDowner.setElevatorSetpoint(ElevatorPosition.DIGIORNO);
    } else if (_operatorController.getAButton()) {
      // Human Player Pickup mode
      _placerDowner.setWantedState(PlacerDownerState.STOW);
      _pickerUpper.setWantedState(PickerUpperState.FRESH_FROM_THE_OVEN);
      _placerDowner.setElevatorSetpoint(ElevatorPosition.FRESH_FROM_THE_OVEN);
      _ledController.humanPlayerPickup();
    } else if (Math.abs(_operatorController.getRightTriggerAxis()) >= 0.25) {
      _placerDowner.setElevatorSetpoint(ElevatorPosition.DIGIORNO);
      _placerDowner.setWantedState(PlacerDownerState.LOW_SCORE);
      _ledController.lowHit();
    }
    

    var eject = Math.abs(_driverController.getRightTriggerAxis()) > 0.25;

    if (eject) {
      _placerDowner.setWantedState(PlacerDownerState.EJECT);

      _ledController.eject();
    } else if (_ejectHeld) {
      _placerDowner.setWantedState(PlacerDownerState.RESET);
      _ledController.ejectHeld();
    }

    _ejectHeld = eject;

    if (_operatorController.getLeftBumper()) {
      _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
    } 
    // else if (Math.abs(_driverController.getLeftTriggerAxis()) >= 0.25) {
    //   _pickerUpper.setWantedState(PickerUpperState.BOX_IT);
    // } 
    else if (_operatorController.getRightBumper()) {
      _pickerUpper.setWantedState(PickerUpperState.WRONG_ORDER);
    } else if (Math.abs(_operatorController.getLeftTriggerAxis()) >= 0.25){
      _placerDowner.setElevatorSetpoint(ElevatorPosition.DIGIORNO);
      _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
      _placerDowner.setWantedState(PlacerDownerState.HOLD_POSITION);
      
    } else if (_pickerUpper.isPizzaReady()) {
      _placerDowner.setWantedState(PlacerDownerState.INTAKE);
      _pickerUpper.setWantedState(PickerUpperState.DELIVERY);
    } else if (_placerDowner.isScoring()) {
      _pickerUpper.setWantedState(PickerUpperState.RELEASE);
    }else if (!_pickerUpper.isBusy()) {
      // IsBusy covers the boxing process or if we are trying Human Player
      _pickerUpper.setWantedState(PickerUpperState.WERE_CLOSED);
    }

    _pickerUpper.handleCurrentState();
    _placerDowner.handleCurrentState();
  }

  private void teleopDrive() {
    var translationX = -OneDimensionalLookup.interpLinear(Constants.XY_Axis_inputBreakpoints,
        Constants.XY_Axis_outputTable, _driverController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    var translationY = -OneDimensionalLookup.interpLinear(Constants.XY_Axis_inputBreakpoints,
        Constants.XY_Axis_outputTable, _driverController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    var rotationZ = -OneDimensionalLookup.interpLinear(Constants.RotAxis_inputBreakpoints,
        Constants.RotAxis_outputTable,
        _driverController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_PER_SECOND;

    if (_driverController.getAButtonPressed() || _driverController.getYButtonPressed() || _driverController.getXButtonPressed() || _driverController.getBButtonPressed() || _driverController.getLeftBumperPressed() || _driverController.getRightBumperPressed()){
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

    if (Math.abs(_driverController.getLeftTriggerAxis()) >= 0.25) {
      translationX /= 2;
      translationY /= 2;
    }

    // Strafing for scoring.
    if (_driverController.getLeftBumper()) {
      rotationZ = _drivetrain.getWallRotationTarget(180);
      translationX = 0;
      translationY = 0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    } else if (_driverController.getRightBumper()) {
      rotationZ = _drivetrain.getWallRotationTarget(180);
      translationX = 0;
      translationY = -0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    }

    // Field orientated speed
    _drivetrain.fieldOrientedDrive(translationX, translationY, rotationZ);

    SmartDashboard.putNumber("driveControllerTranslationX", translationX);
    SmartDashboard.putNumber("driveControllerTranslationY", translationY);
    SmartDashboard.putNumber("driveControllerRotationZ", rotationZ);
  }

  // Graveyard - R.I.P
  
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

  public void loadingSideLinkNoBalance()
  {
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
          _autonomousCase++;
        }
        break;
      default:
        _drivetrain.robotOrientedDrive(0.0, 0.0, 0.0);
        break;
    }
  }

  public void loadingSideLinkConeNoBalance()
  {
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
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 10:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        if (_autoLoops >= 50) {
          _pathFollower.startPath();
          _autonomousCase++;
        }
        break;
      case 11:
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

  public void cableSideLinkBalance()
  {
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

  public void loadSideLinkBalance() {
    switch (_autonomousCase) {
      case 0:
        _placerDowner.setElevatorSetpoint(ElevatorPosition.JETS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        _autonomousCase++;
        break;
      case 1:
        if (_placerDowner.atSetpoint()) {          
          _drivetrain.fieldOrientedDrive(-0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);

          if (_autoLoops > 25) {
            _placerDowner.setWantedState(PlacerDownerState.EJECT);
            _autoLoops = 0;
            _autonomousCase++;
          }
        } else {
          _autoLoops = 0;
        }
        break;
      case 2:
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);

        if (_autoLoops > 25) {
          _placerDowner.setWantedState(PlacerDownerState.RESET);
          _autoLoops = 0;
          _autonomousCase++;
        }
        break;
      case 3:
        if (_autoLoops > 100) {
          _autoLoops = 0;
          _pathFollower.startPath();
          _autonomousCase++;
        } else {
          _drivetrain.fieldOrientedDrive(0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);
        }
        break;
      case 4:
        _drivetrain.followTrajectory();
        if (_autoLoops > 75 && _autoLoops <= 80) {
          _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
        }

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 5:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        _pathFollower.startPath();
        _autonomousCase++;
        break;
      case 6:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 7:
        _placerDowner.setElevatorSetpoint(ElevatorPosition.JETS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        _autonomousCase++;
        break;
      case 8:
        if (_placerDowner.atSetpoint()) {          
          _drivetrain.fieldOrientedDrive(-0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);

          if (_autoLoops > 25) {
            _placerDowner.setWantedState(PlacerDownerState.EJECT);
            _autoLoops = 0;
            _autonomousCase++;
          }
        } else {
          _autoLoops = 0;
        }
        break;
      case 9:
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);

        if (_autoLoops > 25) {
          // No time.
          _autonomousCase = 7769;
          _placerDowner.setWantedState(PlacerDownerState.RESET);
          _autoLoops = 0;
          _autonomousCase++;
        }
        break;
      case 10:
        if (_autoLoops > 100) {
          _autoLoops = 0;
          _pathFollower.startPath();
          _autonomousCase++;
        } else {
          _drivetrain.fieldOrientedDrive(0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);
        }
        break;
      case 11:
        _drivetrain.followTrajectory();
        if (_autoLoops > 75 && _autoLoops <= 80) {
          _pickerUpper.setWantedState(PickerUpperState.SHAKE_N_BAKE);
        }

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 12:
        _drivetrain.robotOrientedDrive(0, 0, 0);
        _pathFollower.startPath();
        _autonomousCase++;
        break;
      case 13:
        _drivetrain.followTrajectory();

        if (_pathFollower.isPathFinished()) {
          _drivetrain.robotOrientedDrive(0, 0, 0);
          _autoLoops = 0;
          _pathFollower.setNextPath();
          _autonomousCase++;
        }
        break;
      case 14:
        _placerDowner.setElevatorSetpoint(ElevatorPosition.JETS);
        _placerDowner.setWantedState(PlacerDownerState.DEPLOY);
        _autonomousCase++;
        break;
      case 15:
        if (_placerDowner.atSetpoint()) {          
          _drivetrain.fieldOrientedDrive(-0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);

          if (_autoLoops > 25) {
            _placerDowner.setWantedState(PlacerDownerState.EJECT);
            _autoLoops = 0;
            _autonomousCase++;
          }
        } else {
          _autoLoops = 0;
        }
        break;
      case 16:
        _drivetrain.fieldOrientedDrive(0.0, 0.0, 0.0);

        if (_autoLoops > 25) {
          _placerDowner.setWantedState(PlacerDownerState.RESET);
          _autoLoops = 0;
          _autonomousCase++;
        }
        break;
      case 17:
        if (_autoLoops > 100) {
          _autoLoops = 0;
          _pathFollower.startPath();
          _autonomousCase++;
        } else {
          _drivetrain.fieldOrientedDrive(0.15 * Constants.MAX_VELOCITY_METERS_PER_SECOND, 0.0, 0.0);
        }
        break;
      case 18:
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
}
