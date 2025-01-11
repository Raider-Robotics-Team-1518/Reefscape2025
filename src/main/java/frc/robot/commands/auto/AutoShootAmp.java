// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoShootAmp extends Command {
  /** Creates a new AutoShootSpeaker. */
  private double speed = Constants.MotorSpeeds.shooterSpeedForAmp;
  private Timer timer;
  private boolean isDone = false;

  public AutoShootAmp() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();  // no-op if already running
    RobotContainer.fmu.setIntakeSpeed(Constants.MotorSpeeds.intakeReverse);
    if (timer.hasElapsed(Constants.Timings.driveIntakeBackwardInSeconds)) {
      RobotContainer.fmu.setShooterSpeed(speed);
    }
    if (timer.hasElapsed(Constants.Timings.bumpDelayInSeconds)) {
      RobotContainer.fmu.bumpIntake();
    }
    if (timer.hasElapsed(1.5d)) {
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    RobotContainer.fmu.setIntakeSpeed(0);
    RobotContainer.fmu.setShooterSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
