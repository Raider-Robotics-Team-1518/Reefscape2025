/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * This command is designed so that a driver can drive 
 * the swerve drive based around the robot's orientation.
 * Forward on the stick will cause the robot to drive 
 * forward. left and right on the stick will cause the 
 * robot to move to its left or right. This command does
 * not end of its own accord so it must be interupted to 
 * end.
 * 
 * Also allows for use of the DPAD in interpretation of angles
 */
public class DriveRobotCentricDPAD extends Command {
  private boolean veloMode;
  /**
   * Creates a new DriveRobotCentric.
   */
  public DriveRobotCentricDPAD(boolean veloMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.swerveDrive);
    this.veloMode = veloMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //RobotContainer.setDriverRumble(0.25, 0.25);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.isFieldRelative = false;
    //pull primary stick values, and put to awaySpeed and lateralSpeed doubles
    double forwardSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftY);
    forwardSpeed *= 0.5;
    double strafeSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftX);
    strafeSpeed *= 0.5;
    //check if secondary sticks are being used
    /*if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightY))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightX))>.1){
      //if secondary sticks used, replace with secondary sticks witha slow factor
      forwardSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightY)*.375;
      forwardSpeed *= 0.5;
      strafeSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightX)*.375;
      strafeSpeed *= 0.5;
    }*/
    //create rotation speed from gamepad triggers
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger) - Robot.robotContainer.getDriverAxis(Axis.kRightTrigger);

    /* DPAD Specific Rotations */
    if(Math.abs(rotSpeed) <= 0.1 && Robot.robotContainer.getDriverDPad() != -1){
        //double targetAngle = Math.toRadians(Robot.robotContainer.getDriverDPad());
        RobotContainer.swerveDrive.setGyro(RobotContainer.swerveDrive.getGyroInDeg()-Robot.robotContainer.getDriverDPad());
    }

    RobotContainer.swerveDrive.driveRobotCentric(
      forwardSpeed *Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_LINEAR ,
      strafeSpeed *Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_LINEAR ,
      rotSpeed*-Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_ROTATIONAL,
      veloMode,
      false
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //RobotContainer.setDriverRumble(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
