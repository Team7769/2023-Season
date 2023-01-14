// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Configuration.Constants;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Subsystem;
import frc.robot.util.OneDimensionalLookup;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

   private static Drivetrain _drivetrain;
   private XboxController _driverController;

   private List<Subsystem> _subsystems;

  @Override
  public void robotInit() {
    _driverController = new XboxController(Constants.kDriverControllerUsbSlot);
    _drivetrain = Drivetrain.getInstance();
    _subsystems.add(_drivetrain);
  }

  @Override
  public void robotPeriodic() {
    for (var x : _subsystems) {
      x.logTelemetry();
      x.readDashboardData();
    }

    _drivetrain.updateOdomery();
  }

  @Override
  public void autonomousInit() {
    _drivetrain.resetOdometry();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
    teleopDrive();
  }

  @Override
  public void disabledInit() {
    _drivetrain.drive(0.0, 0.0, 0.0);
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  private void teleopDrive() {
    //var translationX = _driverController.getLeftX();
    //var translationY = _driverController.getLeftY();
    //var rotationZ = _driverController.getRightX();

    var translationX = OneDimensionalLookup.interpLinear(Constants.XY_Axis_inputBreakpoints, Constants.XY_Axis_outputTable, _driverController.getLeftY()) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    var translationY = OneDimensionalLookup.interpLinear(Constants.XY_Axis_inputBreakpoints, Constants.XY_Axis_outputTable, _driverController.getLeftX()) * Constants.MAX_VELOCITY_METERS_PER_SECOND;
    var rotationZ = OneDimensionalLookup.interpLinear(Constants.RotAxis_inputBreakpoints, Constants.RotAxis_outputTable, _driverController.getRightX()) * Constants.MAX_ANGULAR_VELOCITY_PER_SECOND;
    // if (Math.abs(translationX) <= Constants.kDeadband && Math.abs(translationY) <= Constants.kDeadband)
    // {
    //   translationX = 0;
    //   translationY = 0;
    // }
    // if (Math.abs(rotationZ) <= Constants.kDeadband) {
    //   rotationZ = 0;
    // }

    _drivetrain.teleopDrive(translationX, translationY, rotationZ);
  }
}
