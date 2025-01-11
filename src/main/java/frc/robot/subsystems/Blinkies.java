// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimeLight;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import frc.robot.Utils;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.Results;

public class Blinkies extends SubsystemBase {
  public LEDState lightState = LEDState.DEFAULT;
  public final DigitalOutput dOutput1 = new DigitalOutput(6);
  public final DigitalOutput dOutput2 = new DigitalOutput(7);
  public final DigitalOutput dOutput3 = new DigitalOutput(8);
  public final DigitalOutput dOutput4 = new DigitalOutput(9);
  private LimelightTarget_Fiducial llAprilTag = new LimelightHelpers.LimelightTarget_Fiducial();
  private LimeLight limelight = new LimeLight();
  private LimelightHelpers llHelpers = new LimelightHelpers();

  public Blinkies() {
    limelight.init("limelight");
  }

  @Override
  public void periodic() {
    int fID = (int) llHelpers.getFiducialID("limelight");
    SmartDashboard.putNumber("tx", llAprilTag.tx);
    SmartDashboard.putNumber("fID", fID);

    if (RobotState.isDisabled()) {
      setLEDState(LEDState.DEFAULT);
    } else {
      int targetSpeakerAprilTag = 0;
      int targetAmpAprilTag = 0;
      int targetSourceAprilTag = 0;
      Optional<Alliance> allianceColorOpt = DriverStation.getAlliance();
      if (allianceColorOpt.isPresent()) {
        Alliance allianceColor = allianceColorOpt.get();
        if (allianceColor == Alliance.Red) {
            targetSpeakerAprilTag = Constants.AprilTagIds.redSpeakerLeft;
            targetAmpAprilTag = Constants.AprilTagIds.redAmp;
            targetSourceAprilTag = Constants.AprilTagIds.redSourceLeft;
            setLEDState(LEDState.RED);
        } else if (allianceColor == Alliance.Blue) {
          targetSpeakerAprilTag = Constants.AprilTagIds.blueSpeakerLeft;
          targetAmpAprilTag = Constants.AprilTagIds.blueAmp;
          targetSourceAprilTag = Constants.AprilTagIds.blueSourceLeft;
          setLEDState(LEDState.BLUE);
        }

        SmartDashboard.putString("alliance color", allianceColor.toString());
        SmartDashboard.putNumber("targetSpeakerTag", targetSpeakerAprilTag);
        if (fID > 0) {
          double distance = limelight.getDistanceToTarget(fID);
          RobotContainer.optimalArmAngle = limelight.getOptimalArmAngle(distance);
          SmartDashboard.putNumber("Distance", Utils.round2prec(distance, 1));
          SmartDashboard.putNumber("Optimal Arm Angle", Utils.round2prec(RobotContainer.optimalArmAngle, 1));

          if (distance > 0) {
            if (fID == targetSpeakerAprilTag && distance < Constants.FieldPositions.maxDistanceToSpeaker) {
              setLEDState(LEDState.GREEN);
              // SmartDashboard.putString("LED Color", "green");
            } else if (fID == targetAmpAprilTag && distance < Constants.FieldPositions.maxDistanceToAmp) {
              setLEDState(LEDState.PURPLE);
              // SmartDashboard.putString("LED Color", "purple");
            } else if (fID == targetSourceAprilTag && distance < Constants.FieldPositions.maxDistanceToSource) {
              setLEDState(LEDState.YELLOW);
              // SmartDashboard.putString("LED Color", "yellow");
            } else {
              // SmartDashboard.putString("LED Color", "black");
            }

          // } else {
            // SmartDashboard.putString("LED Color", "white");
          }
        // } else {
          // SmartDashboard.putString("LED Color", "pink");
        }
      }

      /*
       * } else if (note is loaded) {
       * setLEDState(LEDState.ORANGE)
       * }
       */
    }
  }

  public void setLEDState(LEDState state) {
    dOutput1.set(state.do1);
    dOutput2.set(state.do2);
    dOutput3.set(state.do3);
    this.lightState = state;
  }

  public enum LEDState {
    // the state of the DIO ports must match the code on the LED controller
    // which is looking for a pattern of highs & lows to determine the
    // color or pattern to show
    OFF(false, false, false), // 0
    RED(true, false, false), // 1
    BLUE(false, true, false), // 2
    GREEN(true, true, false), // 3
    ORANGE(false, false, true), // 4
    YELLOW(true, false, true), // 5
    PURPLE(false, true, true), // 6
    DEFAULT(true, true, true); // 7

    private final boolean do1, do2, do3;

    private LEDState(boolean do1, boolean do2, boolean do3) {
      this.do1 = do1;
      this.do2 = do2;
      this.do3 = do3;
    }
  }

}
