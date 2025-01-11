// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fmu;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends Command {
  /** Creates a new Shooter. */
  private double speed = Constants.MotorSpeeds.intakeSpeed;
  private Timer timer;

  public Shooter(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
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
    if (timer.hasElapsed(2.5d)) {
      isFinished();
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
    return false;
  }
}
