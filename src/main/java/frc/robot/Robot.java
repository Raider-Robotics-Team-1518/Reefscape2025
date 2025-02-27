// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

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
  private Command autonomousCommand;

  public static RobotContainer robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putNumber("ARM ANGLE", Utils.round2prec(RobotContainer.fmu.get_arm_position(), 1));
    final double[] moduleAngles = RobotContainer.swerveDrive.getAllAbsModuleAngles();
    SmartDashboard.putNumber("SwerveModules/FrntL-Angle", moduleAngles[0]);
    SmartDashboard.putNumber("SwerveModules/RearL-Angle", moduleAngles[1]);
    SmartDashboard.putNumber("SwerveModules/RearR-Angle", moduleAngles[2]);
    SmartDashboard.putNumber("SwerveModules/FrntR-Angle", moduleAngles[3]);
    SmartDashboard.putNumber("Target/H-Offset", (int) RobotContainer.limeLight1.getTargetOffsetHorizontal());
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    Optional<Alliance> allianceColorOpt = DriverStation.getAlliance();
    if (allianceColorOpt.isPresent()) {
      Alliance allianceColor = allianceColorOpt.get();
      if (allianceColor == Alliance.Blue) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      } else if (allianceColor == Alliance.Red) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      }
    }

    autonomousCommand = RobotContainer.autoChooser.getSelected();
    if (autonomousCommand != null) {
      robotContainer.swerveDrive.resetGyro(); // gives warning about accessing a static
      autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    Optional<Alliance> allianceColorOpt = DriverStation.getAlliance();
    if (allianceColorOpt.isPresent()) {
      Alliance allianceColor = allianceColorOpt.get();
      if (allianceColor == Alliance.Blue) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
      } else if (allianceColor == Alliance.Red) {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
      }
    }

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Target/H-Offset", (int) RobotContainer.limeLight1.getTargetOffsetHorizontal());
    SmartDashboard.putNumber("ClimbEncoder", RobotContainer.fmu.climb_pos);

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
