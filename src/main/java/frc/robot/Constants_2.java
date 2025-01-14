// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Preferences;
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {
  /*
   * Raider Robotics Team 1518
   */

  public static boolean setupState = false;

  /*
   * Static Motor Set Speeds
   */

  /* Teleoperated */

  /* Autonomous */

  /*
   * Preference Names
   */

  /*
   * Non-Swerve PID
   */

  /* Field Constants */

  /* Center of Field Offsets */
  public static final double FIELD_CENTER_X = 8.270875d;
  public static final double FIELD_CENTER_Y = 4.00685d;

  // Measured in Meters
  public static final double DIST_TO_UPPER_NODE_CONE = 1.01d;
  public static final double DIST_TO_MIDDLE_NODE_CONE = 0.58d;

  public static final double DIST_TO_UPPER_NODE_CUBE = 1.01d;
  public static final double DIST_TO_MIDDLE_NODE_CUBE = 0.58d;

  public static final double DIST_TO_LOWER_NODE = 0.29d;

  /* Robot Dimensions */
  public static final double ROBOT_SIZE = 0.78105d;
  public static final double TOWER_HEIGHT_TO_PIVOT = 1.1684; // 46 inches
  public static final double TELESCOPE_LENGTH_RETRACTED = 0.9144;

  /* Path Planner Event Map */
  public static HashMap<String, Command> autonomousEventMap = new HashMap<>();

  /*
   * Greater Rochester Robotics
   */
  /* Factors of PI */
  public static final double PI_OVER_TWO = Math.PI * 0.5;
  public static final double THREE_PI_OVER_TWO = 3 * PI_OVER_TWO;
  public static final double TWO_PI = 2 * Math.PI;
  public static final Rotation2d ROTATE_BY_PI = Rotation2d.fromDegrees(180);

  /* Swerve Module Positions */
  public static final class SwerveModulePosition{

    /* 2023 Coordinates */
    /*public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(0.4826, 0.4826);// These are in meters
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-0.4826, 0.4826);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-0.4826, -0.4826);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(0.4826, -0.4826);
    public static final double DRIVE_BASE_RADIUS = 0.4826; // in meters, distance from center to furthest module */
    
    /* 2024 Coordinates - Location of Gyro */
    public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(0.23, 0.15);// These are in meters
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-0.26, 0.15);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-0.26, -0.59);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(0.23, -0.59);
    public static final double DRIVE_BASE_RADIUS = 0.4650; // in meters, distance from center to furthest module
    
    /* 2024 Coordinates - From Center of Robot */
    /* public static final Translation2d FRONT_LEFT_POSITION = new Translation2d(0.245, 0.37);// These are in meters
    public static final Translation2d REAR_LEFT_POSITION = new Translation2d(-0.245, 0.37);
    public static final Translation2d REAR_RIGHT_POSITION = new Translation2d(-0.245, -0.37);
    public static final Translation2d FRONT_RIGHT_POSITION = new Translation2d(0.245, -0.37);
    public static final double DRIVE_BASE_RADIUS = 0.4650; // in meters, distance from center to furthest module */
  }
  /* Swerve Module Drive Motor Constants */
  public static final double DRIVE_ENC_TO_METERS_FACTOR = 0.00001903342;
  // !! newest calculation (not tested) !!
  // encoderPos*DRIVE_ENC_TO_METERS_FACTOR=distanceTraveled
  // encoderPos*(1/2048)*(1/8.14)*(2*3.14159265*0.1016)=distanceTraveled
  // (1/2048)*(1/8.14)*(2*3.14159265*0.1016) = 0.00003829298477
  // YEAH: (1/2048)*(1/8.14)*(pi*0.101)=0.00001903342
  // *2 = 0.00003806684
  // (Calculated using Full Precision calculator, up to 100 decimal places)

  public static final double MINIMUM_DRIVE_SPEED = 0.01;// the slowest the wheels can turn, in m/s
  public static final double MINIMUM_DRIVE_DUTY_CYCLE = 0.05;// the slowest the wheels can turn, in duty cycle output
  public static final double MOTOR_MAXIMUM_VELOCITY = 4.62; // 4.62 default
  public static final double PATH_MAXIMUM_VELOCITY = 2.75d;
  public static final double MAXIMUM_ACCELERATION = 1.25d;
  public static final double PATH_MAXIMUM_ACCELERATION = 1.25;

  public static final double MAXIMUM_VOLTAGE = 12.0;// this is used in compensating for drops in battery voltage

  /* Swerve Move Wheel PIDF constants */
  /* PID Constants for rotation of the swerve module */
  /*
   * Input: ControlPercent Speed
   * Output: Translation Position (Pose2d X or Y)
   */
  public static final double SWERVE_DRIVE_P_VALUE = 0.02928;
  public static final double SWERVE_DRIVE_I_VALUE = 0.0d;
  public static final double SWERVE_DRIVE_D_VALUE = 0.007322; // 0.00089375
  public static final double SWERVE_DRIVE_FF_VALUE = 1023 / (MOTOR_MAXIMUM_VELOCITY / DRIVE_ENC_TO_METERS_FACTOR);

  public static final class AutoConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(SWERVE_DRIVE_P_VALUE, SWERVE_DRIVE_I_VALUE, SWERVE_DRIVE_D_VALUE);
    public static final PIDConstants ANGLE_PID = new PIDConstants(SWERVE_ROT_P_VALUE, SWERVE_ROT_I_VALUE, SWERVE_ROT_D_VALUE); // real values for us
  }

  public static final double SWERVE_DRIVE_P_VELOCITY = 0.0256135252274057; // 0.2928
  public static final double SWERVE_DRIVE_I_VELOCITY = 0.42515784024188247;
  public static final double SWERVE_DRIVE_D_VELOCITY = 0.0; // 0.00089375
  // public static final double SWERVE_DRIVE_FF_VALUE = 0.0d;

  /* Swerve Module Rotation constants */
  public static final double RAD_TO_ENC_CONV_FACTOR = (512 * 8.14) / (Math.PI / 2);
  // rob old (tested, broken): (1outputRev/(2*3.1415 radians)) * (12.8 motorRev /
  // 1 outputRev) * (4096 u / 1 motorRev) = 8344.5488
  // !! newest calculation (tested, weird) !!
  // (1/(2*pi))*2048*8.14 = 2653.2274929
  // 512/(pi/2)=325.949323452
  // (512*8.14)/(Math.PI/2)=2653.2274929

  /* PID Constants for rotation of the swerve module */
  /*
   * Input: Motor ControlPercent Speed
   * Output: Encoder Position (In Radians)
   */
  public static final double SWERVE_ROT_P_VALUE = 0.092241671475395803; // 0.02*4 rob
  public static final double SWERVE_ROT_I_VALUE = 0.0;
  public static final double SWERVE_ROT_D_VALUE = 0.006725014426187409; // .05*4 rob
  public static final double SWERVE_ROT_I_ZONE_VALUE = 0;
  public static final double SWERVE_ROT_FF_VALUE = 0.0;

  public static final double SWERVE_MODULE_TOLERANCE = 0.1;
  public static final double ROTATIONAL_VELOCITY_TOLERANCE = 1.0;

  /* Robot Rotation PID controller constants */
  public static final double ROBOT_SPIN_PID_TOLERANCE = Math.toRadians(0.5);
  public static final double MINIMUM_ROTATIONAL_OUTPUT = 0.10;

  /* Constant for turn to angle functions */
  public static final double ROBOT_SPIN_P = 1.05;// tuned for drive/climber bot
  public static final double ROBOT_SPIN_I = 0.0;
  public static final double ROBOT_SPIN_D = 0.01;

  /* Constants to stop incidental rotation for motion */
  public static final double ROBOT_COUNTER_SPIN_P = 1.1;
  public static final double ROBOT_COUNTER_SPIN_I = 0.0;
  public static final double ROBOT_COUNTER_SPIN_D = 0.001;

  /* Driver Scaling Constants */
  public static final class DriveTrainScaling {
    public static final double DRIVER_SPEED_SCALE_LINEAR = 0.90; //  0.375;
    public static final double DRIVER_SPEED_SCALE_LINEAR_LATERAL = 0.90;
    public static final double DRIVER_SPEED_SCALE_ROTATIONAL = .75;
    public static final double DRIVE_SPEED_SCALE_FACTOR = 0.2; // scale down for "slow mode" (right stick)

  }

  /* IDENTIFICATION NUMBERS FOR DEVICES */

  public static final int DRIVER_CONTROLLER_PORT = 0;
  public static final int CODRIVER_CONTROLLER_PORT = 1;
  

  /* CTRE motor and sensors */

  public static final int FRONT_LEFT_MOVE_MOTOR = 1;// drive module 0
  public static final int FRONT_LEFT_ROTATE_MOTOR = 2;// drive module 0
  public static final int FRONT_LEFT_ROTATE_SENSOR = 15;// drive module 0

  public static final int REAR_LEFT_MOVE_MOTOR = 3;// drive module 1
  public static final int REAR_LEFT_ROTATE_MOTOR = 4;// drive module 1
  public static final int REAR_LEFT_ROTATE_SENSOR = 12;// drive module 1

  public static final int REAR_RIGHT_MOVE_MOTOR = 5;// drive module 2
  public static final int REAR_RIGHT_ROTATE_MOTOR = 6;// drive module 2
  public static final int REAR_RIGHT_ROTATE_SENSOR = 13;// drive module 2

  public static final int FRONT_RIGHT_MOVE_MOTOR = 7;// drive module 3
  public static final int FRONT_RIGHT_ROTATE_MOTOR = 8;// drive module 3
  public static final int FRONT_RIGHT_ROTATE_SENSOR = 14;// drive module 3

  /* Rev Robotics SparkMAXs */
  public static final int LEAD_INTAKE_MOTOR = 21;
  public static final int FOLLOW_INTAKE_MOTOR = 22;
  public static final int LEAD_SHOOTER_MOTOR = 23;
  public static final int FOLLOW_SHOOTER_MOTOR = 24;
  public static final int LEAD_ARM_MOTOR = 25;
  public static final int FOLLOW_ARM_MOTOR = 26;
  public static final int CLIMB_MOTOR = 27;

  /*
   * Magnetic Offsets for CANCoders on Swerve Modules
   * These have been configured in Phoenix TunerX for each CANCoder
   */
  public static final class CANCoderOffsets {
    public static final double FRONT_LEFT_OFFSET = -44.7d;
    public static final double REAR_LEFT_OFFSET = -0.3d;
    public static final double REAR_RIGHT_OFFSET = 62.1;
    public static final double FRONT_RIGHT_OFFSET = -7.5d;
  }

  /* Robot operational parameters */
  public static final class MotorSpeeds {
    public static final double intakeSpeed = 0.45d;
    public static final double intakeBumpSpeed = 1.0d;
    public static final double intakeReverse = -0.20d;
    public static final double shooterSpeedForSpeaker = 1.0d;
    public static final double shooterSpeedForAmp = 0.5d;
    public static final double armPowerUp = 0.85d;
    public static final double armPowerDn = 0.25d;
    public static final double climbPower = 0.75d;
  }

  public static final class Timings {
    public static final double driveIntakeBackwardInSeconds = 0.03d;
    public static final double bumpDelayInSeconds = 0.40d;
    public static final double resetColorSensorDelay = 2.0d;
  }

  public static final class ColorValues {
    // For HSL, define the min & max of HUE that is orange
    // see https://hslpicker.com/#ff6a00
    public static final float orangeHueMin = 30.0f;
    public static final float orageHueMax = 80.0f;
    // For RGB, see https://rgbcolorcode.com/color/FFAA00
    public static final double red = 0.568d;
    public static final double blue = 0.073d;
    public static final double green = 0.354d;
  }

  public static final class AprilTagIds {
    /*
     * Blue Source (right to left) – ID 1, 2
     * Red Speaker (right to left) – ID 3, 4
     * Red Amp – ID 5
     * Blue Amp – ID 6
     * Blue Speaker (right to left) – ID 7, 8
     * Red Source (right to left) – ID 9,10
     * Red Stage (counter-clockwise starting at Stage Left) – ID 11, 12, 13
     * Blue Stage (counter-clockwise starting at Center Stage) – ID 14, 15, 1
     */
    public static final int redSpeakerLeft = 4;
    public static final int blueSpeakerLeft = 7;
    public static final int redAmp = 5;
    public static final int blueAmp = 6;
    public static final int redSourceLeft = 10;
    public static final int redSourceRight = 9;
    public static final int blueSourceLeft = 2;
    public static final int blueSourceRight = 1;
  }

  public static final class FieldPositions {
    // inches, distance from the center of the Limelight lens to the floor
    public static final double limelightMountingHeight = 8;
    // degrees, measured from vertical, with tilted up being positive
    public static final double limelightMountingAngle = 24;
    // inches
    public static final double speakerTagHeight = 54.675; // 53.88;
    // inches
    public static final double ampTagHeight = 48.125;
    // inches
    public static final double sourceTagHeight = 48.125;
    // inches
    public static final double sourceTagSpacing = 19.0375;
    // inches
    public static final double maxDistanceToSpeaker = 100;
    // inches
    public static final double maxDistanceToAmp = 100;
    // inches
    public static final double maxDistanceToSource = 100;
  }

  public static final class Tolerances {
    public static final double armAimingTolerance = 1.00d; // angle, how close to the desired angle do we have to be
  }
  
  public static final class Limits {
    public static final double armMinAngle = 92;
    public static final double armMaxAngle = 192;
    public static final double armLoadAngle = 175;
    public static final double armSourceAngle = 110;
    public static final double armDefaultSpkrAngle = 158;
    public static final double armDefaultAmpAngle = 92;
    public static final double climbMax = -290.0d;
    public static final double climbMin = -0.0d;
  }

  /* Solenoid Channels */

  /* Digital Input */

  public static void updateDouble(String key, double newValue) {
    try {
      Preferences.remove(key);
    } catch (Exception exception) {
    }
    Preferences.setDouble(key, newValue);
  }

  public static double getDouble(String key) {
    if (Preferences.containsKey(key)) {
      return Preferences.getDouble(key, -69);
    }
    return -69;
  }

}
