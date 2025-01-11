// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fmu;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class ShooterIntake extends Command {
  /** Creates a new ShooterIntake. */
  private double speed = Constants.MotorSpeeds.intakeSpeed;

  public ShooterIntake(double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speed > 0 && !RobotContainer.fmu.isNoteLoaded()) {
      RobotContainer.fmu.setIntakeSpeed(speed);
    }
    else if (speed < 0) {
      RobotContainer.fmu.setIntakeSpeed(speed);

    }
    else {
      RobotContainer.fmu.stopIntake();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.fmu.setIntakeSpeed(0);
    if (!interrupted && RobotContainer.fmu.isNoteLoaded()) {
      // TODO: move arm to optimal angle for speaker
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
