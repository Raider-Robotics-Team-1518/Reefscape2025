// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
// import com.revrobotics.CANSparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.AbsoluteEncoder;
//import com.revrobotics.EncoderType;
// import com.revrobotics.CANSparkMax.IdleMode;
// com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;
import frc.robot.Utils;



public class FieldManipulationUnit extends SubsystemBase {
  /** Creates a new Shooter. */
  // private CANSparkMax lead_shooter_motor;
  // private CANSparkMax follow_shooter_motor;
  private TalonFX lead_shooter_motor;
  private TalonFX follow_shooter_motor;
  private SparkMax lead_intake_motor;
  private SparkMax follow_intake_motor;
  private SparkMax climb_motor;
  private TalonFX lead_arm_motor;
  private TalonFX follow_arm_motor;
  private DutyCycleEncoder arm_position;
  private boolean override_note_is_loaded;
  private final I2C.Port i2cPort = I2C.Port.kMXP;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  public double climb_pos = 0.0d;
  public boolean climbStop = false; // Used on dashboard to turn RED when at max or min position

  public FieldManipulationUnit () {
    lead_shooter_motor = new TalonFX(Constants.LEAD_SHOOTER_MOTOR); // new CANSparkMax(Constants.LEAD_SHOOTER_MOTOR, MotorType.kBrushless);
    lead_shooter_motor.setInverted(true);
    // lead_shooter_motor.setControl(new CoastOut());
    // lead_shooter_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    follow_shooter_motor = new TalonFX(Constants.FOLLOW_SHOOTER_MOTOR); // new CANSparkMax(Constants.FOLLOW_SHOOTER_MOTOR, MotorType.kBrushless);
    follow_shooter_motor.setInverted(true);
    // follow_shooter_motor.setControl(new CoastOut());
    // follow_shooter_motor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    //follow_shooter_motor.follow(lead_shooter_motor); // Removed to set differential speeds

    lead_intake_motor = new SparkMax(Constants.LEAD_INTAKE_MOTOR, MotorType.kBrushless);
    follow_intake_motor = new SparkMax(Constants.FOLLOW_INTAKE_MOTOR, MotorType.kBrushless);
    // follow_intake_motor.follow(lead_intake_motor);
    follow_intake_motor.isFollower();

    lead_arm_motor = new TalonFX(Constants.LEAD_ARM_MOTOR);
    lead_arm_motor.setInverted(true);
    lead_arm_motor.setControl(new StaticBrake());
    follow_arm_motor = new TalonFX(Constants.FOLLOW_ARM_MOTOR);
    follow_arm_motor.setControl(new StaticBrake());
    follow_arm_motor.setControl(new Follower(Constants.LEAD_ARM_MOTOR, true));
    arm_position = new DutyCycleEncoder(5);
    // arm_position.setDistancePerRotation(1.0d);
    

    climb_motor = new SparkMax(Constants.CLIMB_MOTOR, MotorType.kBrushless);
    
    override_note_is_loaded = false;

  }

  public void setIntakeSpeed(double speed) {
    lead_intake_motor.set(speed);
  }

  public void stopIntake() {
    lead_intake_motor.set(0);
  }

  public void bumpIntake() {
    // run intake slowly to push Note into shooter
    lead_intake_motor.set(Constants.MotorSpeeds.intakeBumpSpeed);
    Timer.delay(0.5d);
    lead_intake_motor.set(0);
    override_note_is_loaded = true;
//    Timer.delay(Constants.Timings.resetColorSensorDelay);
    override_note_is_loaded = false;
  }

  public void setShooterSpeed(double speed) {
    lead_shooter_motor.set(speed * 0.85d);
    follow_shooter_motor.set(speed * 1.0d);
  }

  public void stop_shooterCommand() {
    lead_shooter_motor.set(0);
  }

  public void move_arm(double power){
    lead_arm_motor.set(power);
  }

  public void stop_arm(){
    lead_arm_motor.set(0);
  }

  public void move_climb(double power){
    climb_pos = climb_motor.getEncoder().getPosition();
    if((climb_pos >= Constants.Limits.climbMax) && (Math.signum(power) < 0)) {
      climb_motor.set(power);
    } else {
      if((climb_pos <= Constants.Limits.climbMin) && (Math.signum(power) > 0)) {
        climb_motor.set(power);
      } else {
          climb_motor.set(0.0);
      }
    }
  }

  public void override_move_climb(double power) {
    climb_motor.set(power);
  }

  public void climbResetEncoder() {
    climb_motor.getEncoder().setPosition(0);
  }

  public void stop_climb(){
    climb_pos = climb_motor.getEncoder().getPosition();
    climb_motor.set(0);
  }

  public double get_arm_position(){
   // return arm_position.getAbsolutePosition() * 360;
  return 0;
  }

  public boolean isFinished() {
    return true;
  }

  public boolean isNoteLoaded() {
    Color detectedColor = m_colorSensor.getColor(); // returns a struct of doubles
    double r = detectedColor.red;
    double b = detectedColor.blue;
    double g = detectedColor.green;

    // calculate with Hue Saturation Value (HSV)
    float hue = Utils.getHue((float)r, (float)g, (float)b);
    // SmartDashboard.putNumber("Hue", (double)hue);

    if (hue > Constants.ColorValues.orangeHueMin && hue < Constants.ColorValues.orageHueMax) {
      SmartDashboard.putBoolean("Note Loaded", true);
      return true;
    }
    else {
      SmartDashboard.putBoolean("Note Loaded", false);
      return false;
    }

    // alternatively, calculate using the RGB values
    // orange has lots of red, a fair bit of blue, and not much green
    // tune these amounts in the Constants file
/*    if (r > Constants.ColorValues.red && b > Constants.ColorValues.blue && g < Constants.ColorValues.green) {
      return true;
    }
*/
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    // COMMENT OUT TO DISABLE THE COLOR SENSOR
    if (!override_note_is_loaded) {
      if (isNoteLoaded()) {
        stopIntake();
      }
    }
    // Dashboard light for climb max/min position
    if (climb_pos <= Constants.Limits.climbMax || climb_pos >= Constants.Limits.climbMin) {
      climbStop = true;
    } else {
      climbStop = false;
    }
    SmartDashboard.putBoolean("Climb Stop", climbStop);
  }
}
