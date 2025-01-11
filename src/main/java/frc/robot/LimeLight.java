/*
 * RaiderRobotics class for interacting with the LimeLight camera
 * 
 * The Limelight pipeline in use determines whether we're looking for
 * an AprilTag or retroreflective marker. See 
 * https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltags
 * for information on setting up an AprilTag pipeline.
 */


package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
  LimeLight wrapper class
*/

public class LimeLight {
  private static NetworkTableInstance table = null;
  private static String tableName = "limelight";

  public static enum LightMode {
    pipeline, off, blink, on
  }
  public static enum CameraMode {
    vision, driver
  }

 
  public final void init(String table_name) {
    if (table == null) {
      table = NetworkTableInstance.getDefault();
    }
    tableName = table_name;
  }

  private static NetworkTableEntry getTableEntry(String key) {
    return table.getTable(tableName).getEntry(key);
  }
//   private static void setTableEntry(String key, int value) {
//     NetworkTableEntry tableEntry = getTableEntry(key);
//     tableEntry.setNumber(value);
//   }

  /* PUBLIC METHODS */

  /**
    Determines if target is visible
    @return true or false
  */
  public boolean isTargetVisible() {
    return getTableEntry("tv").getDouble(0) == 1;
  }

  /**
    Horizontal offset from crosshair to target (LimeLight tx value)
    @return double (+/-27 degrees)
  */
  public double getTargetOffsetHorizontal() {
    return getTableEntry("tx").getDouble(0.00);
  }

  /**
    Vertical offset from crosshair to target (LimeLight ty value)
    @return double (+/-20.5 degrees)
  */
  public double getTargetOffsetVertical() {
    return getTableEntry("ty").getDouble(0.00);
  }

  /**
   * Distance to the target in inches
   * @return double (inches)
   */
  public double getDistanceToTarget(int aprilTagId) {
    double angleToGoalDegrees = Constants.FieldPositions.limelightMountingAngle + getTargetOffsetVertical();
    double angleToGoalRadians = angleToGoalDegrees * 0.01745327778; // (3.14159 / 180.0);

    double goalHeightInches = 0;
    switch (aprilTagId) {
        case Constants.AprilTagIds.redSpeakerLeft:
        case Constants.AprilTagIds.blueSpeakerLeft:
            goalHeightInches = Constants.FieldPositions.speakerTagHeight;
            break;
        case Constants.AprilTagIds.redAmp:
        case Constants.AprilTagIds.blueAmp:
            goalHeightInches = Constants.FieldPositions.ampTagHeight;
            break;
        case Constants.AprilTagIds.redSourceLeft:
        case Constants.AprilTagIds.blueSourceLeft:
            goalHeightInches = Constants.FieldPositions.sourceTagHeight;
            break;
    }
    double distance = (goalHeightInches - Constants.FieldPositions.limelightMountingHeight) / Math.tan(angleToGoalRadians);
    return distance;
  }

  public double getOptimalArmAngle(double distance) {
    /*
     * Computed formula:
     *  from Mr. Barnes and students' spreadsheet
     *  181 -6.7 * distance + 0.27 * Math.pow(distance, 2)
     */
    final double distanceInFeet = distance / 12;
    return 178 -6.7 * distanceInFeet + 0.27 * Math.pow(distanceInFeet, 2);
  }

  /**
    Target area as percentage of camera FOV (LimeLight ta value)
    @return double (0-100)
  */
  public double getTargetArea() {
    return getTableEntry("ta").getDouble(0.00);
  }

  /**
    Target skew (LimeLight ts value)
    @return double (-90 to 0 degrees)
  */
  public double getTargetSkew() {
    return getTableEntry("ts").getDouble(0.00);
  }

  /**
    Pipeline latency (LimeLight tl value)
    @return double (milliseconds)
  */
  public double getPipelineLatency() {
    return getTableEntry("tl").getDouble(0.00);
  }

  /**
    Total latency (LimeLight tl value + 11 ms for image capture)
    @return double (milliseconds)
  */
  public double getLatency() {
    Double latency = getTableEntry("tl").getDouble(0.00);
    if (latency == 0) {
      return latency;
    }
    return latency + 11.0;
  }

  /**
    Height of the rough bounding box (LimeLight tvert value)
    @return double (0-320 pixels)
  */
  public double getBoundingBoxHeight() {
    return getTableEntry("tvert").getDouble(0.00);
  }

  /**
    Width of the rough bounding box (LimeLight thor value)
    @return double (0-320 pixels)
  */
  public double getBoundingBoxWidth() {
    return getTableEntry("thor").getDouble(0.00);
  }


  /* ***** SETTERS ***** */

  /**
    Set LED ring mode
    @param mode LightMode
    @example
      LimeLight targetingLimeLight = new LimeLight('targeting_limelight');
      targetingLimeLight.setLedMode(LimeLight.LightMode.on);
  */
  public static void setLedMode(LightMode mode) {
    getTableEntry("ledMode").setNumber(mode.ordinal());
  }

  /**
    Set camera mode
    @param mode CameraMode
    @example
      LimeLight targetingLimeLight = new LimeLight('targeting_limelight');
      targetingLimeLight.setCameraMode(LimeLight.CameraMode.vision);
  */
  public static void setCameraMode(CameraMode mode) {
    getTableEntry("camMode").setNumber(mode.ordinal());
  }

  /**
    Set pipeline (integer from 0 to 9)
    @param number int
    @example
      LimeLight targetingLimeLight = new LimeLight('targeting_limelight');
      targetingLimeLight.setPipeline(2);
  */
  public static void setPipeline(int number) {
    getTableEntry("pipeline").setNumber(number);
  }

}