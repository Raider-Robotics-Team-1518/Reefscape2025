// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController.Axis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * This command is designed so that a driver can drive 
 * the swerve drive based around a fixed orientation.
 * Forward on the stick should cause the robot to away 
 * from the driver. If this is true, then left and right 
 * on the stick will cause the robot to move to the 
 * driver's left and right, respectively. This command 
 * does not end of its own accord so it must be interrupted 
 * to end.
 * 
 * UNLIKE DriveFieldCentric this command uses a PIDController 
 * to maintain the robot's rotational orientation when the 
 * robot is not instructed to rotate by the rotational 
 * input. 
 */

public class DriveFieldRelativeAdvanced extends Command {
  private double currentAngle = 0;
  private boolean wasDriverControl;
  private boolean veloMode;

  /** Creates a new DriveFieldCentricAdvanced. */
  public DriveFieldRelativeAdvanced(boolean veloMode) {
    addRequirements(RobotContainer.swerveDrive);
    this.veloMode = veloMode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentAngle = RobotContainer.swerveDrive.getGyroInRad();
    wasDriverControl = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.swerveDrive.isFieldRelative = true;
    //pull primary stick values, and put to awaySpeed and lateralSpeed doubles
    double awaySpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftY)*0.5;
    double lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftX)*0.5;
    //check if secondary sticks are being used
    /*if(Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightY))>.1 ||
      Math.abs(Robot.robotContainer.getDriverAxis(Axis.kRightX))>.1){
      //if secondary sticks used, replace with secondary sticks witha slow factor
      awaySpeed = Robot.robotContainer.getDriverAxis(Axis.kRightY)*.375;
      lateralSpeed = Robot.robotContainer.getDriverAxis(Axis.kRightX)*.375;
    }*/
    //create rotation speed from gamepad triggers
    double rotSpeed = Robot.robotContainer.getDriverAxis(Axis.kLeftTrigger) - Robot.robotContainer.getDriverAxis(Axis.kRightTrigger);

    //use DPad to turn to specific angles. left over from GRR 340 2021
    /*if(Robot.robotContainer.getDriverDPad() == 0){
       currentAngle = Math.round(RobotContainer.swerveDrive.getGyroInRad()/Constants.TWO_PI) * Constants.TWO_PI;
    } else if(Robot.robotContainer.getDriverDPad() == 90){
      currentAngle = Math.round(RobotContainer.swerveDrive.getGyroInRad()/Constants.TWO_PI) * Constants.TWO_PI - 1.178;
    }*/
    
    //test if the absolute rotational input is greater than .1
    if (Math.abs(rotSpeed) > .1){
      //if the test is true, just copy the DriveFieldCentric execute method
      RobotContainer.swerveDrive.driveFieldRelative(
        awaySpeed*-Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_LINEAR,
        lateralSpeed*-Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_LINEAR,
        rotSpeed*-Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_ROTATIONAL ,
        veloMode
      );
      //for when rotation speed is zero, update the current angle
      currentAngle = RobotContainer.swerveDrive.getGyroInRad();
      wasDriverControl = true;

    }
    else {
      if(wasDriverControl && Math.abs(RobotContainer.swerveDrive.getRotationalVelocity()) > 90.0){
        RobotContainer.swerveDrive.driveFieldRelative(
          awaySpeed*-Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_LINEAR,
          lateralSpeed*-Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_LINEAR,
          0,
          veloMode
        );
        currentAngle = RobotContainer.swerveDrive.getGyroInRad();
      }else{
        //if the test is false, still use driveFieldCentric(), but for last parameter use PIDController accessor function
        RobotContainer.swerveDrive.driveFieldRelative(
          awaySpeed*-Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_LINEAR,
          lateralSpeed*-Constants.DriveTrainScaling.DRIVER_SPEED_SCALE_LINEAR,
          RobotContainer.swerveDrive.getCounterRotationPIDOut(currentAngle),
          veloMode
        );
        wasDriverControl = false;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
