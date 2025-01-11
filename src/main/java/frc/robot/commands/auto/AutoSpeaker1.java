// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.fmu.FullAimingSpeaker;

public class AutoSpeaker1 extends Command {
  /** Creates a new AutoAimArm. */
  private Timer timer;
  public AutoSpeaker1() {
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer.start();
    
    if(timer.hasElapsed(2.50d)){
      RobotContainer.swerveDrive.stopAllModules();
      // new StopDriveTrain().andThen(new FullAimingSpeaker()).andThen(new AutoShootSpeaker()).end(false);
    } else {
          // RobotContainer.swerveDrive.driveFieldRelative(0.40d, 0, 0, false);
          RobotContainer.swerveDrive.driveRobotCentric(0.4d,0,0, false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
