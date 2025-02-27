// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.fmu.MoveArmToAngle;

public class AutoIntake extends Command {
  /** Creates a new AutoIntake. */
  private double speed = Constants.MotorSpeeds.intakeSpeed;
  private boolean isDone = false;

  public AutoIntake() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new MoveArmToAngle(Constants.Limits.armMaxAngle);
    if (!RobotContainer.fmu.isNoteLoaded()) {
      RobotContainer.fmu.setIntakeSpeed(speed);
    } else {
      isDone = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // I think we don't want to stop the intake here so that it will
    // run until we call the AutoStopIntake command
    // RobotContainer.fmu.setIntakeSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
