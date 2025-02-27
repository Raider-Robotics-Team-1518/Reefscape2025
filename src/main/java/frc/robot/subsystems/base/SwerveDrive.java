// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.base;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import frc.robot.Constants;
import frc.robot.subsystems.swervelib.SwerveModule;

/**
 * This subsystem contains all SwerveModule objects, and runs all drive functions.
 * 
 * More on swerve found here:
 * https://docs.google.com/presentation/d/1feVl0L5lgIKSZhKCheWgWhkOydIu-ibgdp7oqA0yqAQ/edit?usp=sharing
 */
public class SwerveDrive extends SubsystemBase {

  private static SwerveModule swerveModules[];
  private static SwerveModule frontLeft, rearLeft, rearRight, frontRight;
  public boolean isFieldRelative = false;
  public AHRS imu;
  public SwerveDriveKinematics driveKinematics;
  public SwerveDriveOdometry driveOdometry;
  private PIDController robotSpinController;
  private PIDController robotCounterSpinController;
  private boolean hasPoseBeenSet = false;

  /**
   * This enumeration clarifies the numbering of the swerve module for new users.
   * frontLeft  | 0
   * rearLeft   | 1
   * rearRight  | 2
   * frontRight | 3
   */
  public enum SwerveModNum{
    frontLeft(0) , rearLeft(1) , rearRight(2) , frontRight(3);
    public int num;
    private SwerveModNum(int number){
      num = number;
    }
    public int getNumber() {
			return num;
		}
  }
  
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {

        // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
  RobotConfig config = null;

  try {
    config = RobotConfig.fromGUISettings();
  } catch (Exception e) {
    // Handle exception as needed
    e.printStackTrace();
  }

    // Constructs the swerve modules 
    frontLeft = new SwerveModule(Constants.FRONT_LEFT_MOVE_MOTOR, Constants.FRONT_LEFT_ROTATE_MOTOR, Constants.FRONT_LEFT_ROTATE_SENSOR, false);
    rearLeft = new SwerveModule(Constants.REAR_LEFT_MOVE_MOTOR, Constants.REAR_LEFT_ROTATE_MOTOR, Constants.REAR_LEFT_ROTATE_SENSOR, false);
    rearRight = new SwerveModule(Constants.REAR_RIGHT_MOVE_MOTOR, Constants.REAR_RIGHT_ROTATE_MOTOR, Constants.REAR_RIGHT_ROTATE_SENSOR, false);
    frontRight = new SwerveModule(Constants.FRONT_RIGHT_MOVE_MOTOR, Constants.FRONT_RIGHT_ROTATE_MOTOR, Constants.FRONT_RIGHT_ROTATE_SENSOR, false);
    
     //This may seem repetitive, but it makes clear which module is which.
    swerveModules = new SwerveModule[]{
      frontLeft,
      rearLeft,
      rearRight,
      frontRight
    };
    
    //Create kinematics object, which converts between ChassisSpeeds and ModuleStates
    driveKinematics = new SwerveDriveKinematics(
      Constants.SwerveModulePosition.FRONT_LEFT_POSITION, Constants.SwerveModulePosition.REAR_LEFT_POSITION, 
      Constants.SwerveModulePosition.REAR_RIGHT_POSITION, Constants.SwerveModulePosition.FRONT_RIGHT_POSITION);

    // Constructs IMU object
    imu = new AHRS(SerialPort.Port.kUSB);//Must use params, won't work without
    
    //construct the odometry class.
    driveOdometry = new SwerveDriveOdometry(driveKinematics, getGyroRotation2d(), getSwerveModulePositions());

    //construct the wpilib PIDcontroller for rotation.
    robotSpinController = new PIDController(Constants.ROBOT_SPIN_P, Constants.ROBOT_SPIN_I, Constants.ROBOT_SPIN_D);
    robotSpinController.setTolerance(Constants.ROBOT_SPIN_PID_TOLERANCE);

    //construct the wpilib PIDcontroller for counter rotation.
    robotCounterSpinController = new PIDController(Constants.ROBOT_COUNTER_SPIN_P, Constants.ROBOT_COUNTER_SPIN_I, Constants.ROBOT_COUNTER_SPIN_D);
    robotCounterSpinController.setTolerance(Constants.ROBOT_SPIN_PID_TOLERANCE);

    hasPoseBeenSet = false;

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getCurPose2d, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotCentric(speeds, false, false), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    Constants.AutoConstants.TRANSLATION_PID, // Translation PID constants
                    Constants.AutoConstants.ANGLE_PID // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Gyro", this.getGyroInDeg());
    SmartDashboard.putNumber("XPos", getCurPose2d().getX());
    SmartDashboard.putNumber("YPos", getCurPose2d().getY());
    SmartDashboard.putBoolean("FieldRelativeEnabled", isFieldRelative);


    //run odometry update on the odometry object
    driveOdometry.update(getGyroRotation2d(), getSwerveModulePositions());

  }

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] states = driveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.0d);
    for (int i = 0; i < states.length; i++) {
        swerveModules[i].setModuleState(states[i], false);
    }
  }

  /*
   * Get states used for kinematics/odometry/AutoBuilder
   */
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4]; // 4 swerve drive modules
    for (int i = 0; i < 4; i++) {
      states[i] = swerveModules[i].getModuleState();
    }
    return states;
  }
  public ChassisSpeeds getChassisSpeeds() {
    return driveKinematics.toChassisSpeeds(getStates());
  }

  public void setModuleStates(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.0d);
    //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
    for (int i = 0; i < states.length; i++) {
        swerveModules[i].setModuleState(states[i], false);
    }
  }

  public void setModuleStatesVelocity(SwerveModuleState[] states) {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MOTOR_MAXIMUM_VELOCITY);
    //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
    for (int i = 0; i < states.length; i++) {
        swerveModules[i].setModuleState(states[i], true);
    }
  }

  public void setModulesVelocityToDutyCycle(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      states[i].speedMetersPerSecond = states[i].speedMetersPerSecond/Constants.MOTOR_MAXIMUM_VELOCITY;
    }
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 1.0d);
    //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
    for (int i = 0; i < states.length; i++) {
        swerveModules[i].setModuleState(states[i], false);
    }
  }

  public void simpleDriveRotationControlPercent(double rotSpeed, int modNumber) {
      swerveModules[modNumber].setRotationMotor(rotSpeed);
  }

  public void simpleDriveControlPercent(double speed) {
    driveRobotCentric(
        speed,
        0.0d,
        0.0d,
        false,
        false
    );


}

  /* =================== Module Drive Methods =================== */

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param chassisSpeeds an object  
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void driveRobotCentric(ChassisSpeeds chassisSpeeds, boolean isVeloMode, boolean rotationOnlyMode){
    //instantiate an array of SwerveModuleStates, set equal to the output of toSwerveModuleStates() 
    SwerveModuleState[] targetStates = driveKinematics.toSwerveModuleStates(chassisSpeeds);
    //use SwerveDriveKinematic.desaturateWheelSpeeds(), max speed is 1 if percentOutput, MaxVelovcity if velocity mode
    SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, isVeloMode? Constants.MOTOR_MAXIMUM_VELOCITY : 1.0);
    // if(Math.abs(chassisSpeeds.vxMetersPerSecond) < 0.05 && Math.abs(chassisSpeeds.vyMetersPerSecond) < 0.05 && Math.abs(chassisSpeeds.omegaRadiansPerSecond) > .01){
    if(rotationOnlyMode){
      //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
      for (int i = 0; i < targetStates.length; i++) {
        swerveModules[i].setModuleStateRot(targetStates[i], isVeloMode);
      } 
    }else{
      //pass along SwerveModuleStates to SwerveModules and pass along boolean isVeloMode
      for (int i = 0; i < targetStates.length; i++) {
          swerveModules[i].setModuleState(targetStates[i], isVeloMode);
      }
    }
  }

  /**
   * Drives the robot based on speeds from the robot's orientation.
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not 
   * moving for percentVoltage mode and between the Max Velocity 
   * and -Max Velocity with 0 not moving in Velocity mode
   * @param forwardSpeed the movement forward and backward
   * @param strafeSpeed the movement side to side
   * @param rotSpeed the speed of rotation
   * @param isVeloMode true if velocity mode, false if percent output mode
   * @param rotationOnlyMode special boolean that allows override of module rotation, for when the robot only needs to rotate
   */
  public void driveRobotCentric(double forwardSpeed, double strafeSpeed, double rotSpeed, boolean isVeloMode, boolean rotationOnlyMode){
    //convert forwardSpeed, strafeSpeed and rotSpeed to a chassisSpeeds object, pass to driveRobotCentric
    driveRobotCentric(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotSpeed), isVeloMode, rotationOnlyMode);
  }

  /**
   * Drive the robot so that all directions are independent of the robots orientation (rotation)
   * all speed should be in range of -1.0 to 1.0 with 0.0 being not moving in that direction
   * 
   * @param awaySpeed from field relative, aka a fix direction,
   *                  away from or toward the driver, a speed
   *                  valued between -1.0 and 1.0, where 1.0
   *                  is to away from the driver 
   * @param lateralSpeed from field relative, aka a fix direction
   *                     regardless of robot rotation, a speed
   *                     valued between -1.0 and 1.0, where 1.0
   *                     is to the left 
   * @param rotSpeed rotational speed of the robot
   *                 -1.0 to 1.0 where 0.0 is not rotating
   * @param isVeloMode true if velocity mode, false if percent output mode
   */
  public void driveFieldRelative(double awaySpeed, double lateralSpeed, double rotSpeed, boolean isVeloMode){
    //convert awaySpeed, lateralSpeed and rotSpeed to ChassisSpeeds with fromFieldRelativeSpeeds pass to driveRobotCentric
    driveRobotCentric(ChassisSpeeds.fromFieldRelativeSpeeds(awaySpeed, lateralSpeed, rotSpeed, getGyroRotation2d()), isVeloMode, false);
  }

/**
   * This function is meant to drive one module at a time for testing purposes.
   * @param moduleNumber which of the four modules(0-3) we are using
   * @param moveSpeed move speed -1.0 to 1.0, where 0.0 is stopped
   * @param rotatePos a position between -PI and PI where we want the module to be
   * @param isVeloMode changes between velocity mode and dutyCycle mode
   */
  public void driveOneModule(int moduleNumber,double moveSpeed, double rotatePos, boolean isVeloMode){
    //test that moduleNumber is between 0-3, return if not(return;)
    if (moduleNumber > 3 && moduleNumber < 0){
      // System.out.println("Module " + moduleNumber + " is out of bounds.");
      return;
    }else if(rotatePos < -Math.PI || rotatePos > Math.PI){
      // System.out.println("Input angle out of range.");
      return;
    }

    SwerveModuleState oneSwerveState = new SwerveModuleState(moveSpeed, new Rotation2d(rotatePos));
    //code to drive one module in a testing form
    swerveModules[moduleNumber].setModuleState( oneSwerveState , isVeloMode );

  }

  /**
   * Stops all module motion, then lets all the modules spin freely.
   */
  public void stopAllModules(){
    //run a for loop to call each mmodule
    for (int i=0; i<4; i++){
      //use the stopAll method, which stops both the drive and rotation motor.
      swerveModules[i].stopAll();
    }
  }

  /* =================== Robot Pose Methods =================== */
  
  /**
   * Pull the current Position from the odometry 
   * class, this should be in meters.
   * 
   * @return a Pose2d representing the current position
   */
  public Pose2d getCurPose2d(){
    return driveOdometry.getPoseMeters();
  }

  public void resetPose() {
    driveOdometry.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), getCurPose2d());
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    driveOdometry.resetPosition(initialHolonomicPose.getRotation(), getSwerveModulePositions(), getCurPose2d());
    // driveOdometry.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), getCurPose2d());
  }


  public void resetPoseAprilTags() {

  }

  /**
   * Sets current position in the odometry class
   * 
   * @param pose new current position
   */
  public void setCurPose2d(Pose2d pose) {
    driveOdometry.resetPosition(getGyroRotation2d(), getSwerveModulePositions(), pose);
    hasPoseBeenSet = true;
  }

  /**
   * Returns whether or not the position of the robo on the 
   * field is known because Pose2D has been set
   * @return
   */
  public boolean hasPoseBeenSet() {
    return hasPoseBeenSet;
  }

  /* =================== Gyro/IMU Methods =================== */
  
  /**
   * A function that allows the user to reset the gyro, this 
   * makes the current orientation of the robot 0 degrees on 
   * the gyro.
   */
  public void resetGyro(){
    //Resets the gyro(zero it)
    imu.reset();
  }

  /**
   * A function that allows the user to set the gyro to a 
   * specific angle. This will make the current orientation 
   * of the robot the input value. This must be in degrees 
   * for gyro.
   * @param newCurrentAngle value the gyro should now read in degrees.
   */
  public void setGyro(double newCurrentAngle){
    imu.reset();
    imu.setAngleAdjustment(newCurrentAngle);
  }

  
  /**
   * This calls the drive Gyro and returns the Rotation2d object.
   * This object contains angle in radians, as well as the sin 
   * and cos of that angle. This is an object that represents the
   * rotation of the robot.
   * @return a Rotation2d object
   */
  public Rotation2d getGyroRotation2d(){
    //return a newly constructed Rotation2d object, it takes the angle in radians as a constructor argument
    return Rotation2d.fromDegrees(getGyroInDeg());
    //note that counterclockwise rotation is positive
  }

  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in radians
   */
  public double getGyroInRad(){
    return Math.toRadians(getGyroInDeg());// Pull the gyro in degrees, convert and return in radians
    //note that counterclockwise rotation is positive
  }


  /**
   * This polls the onboard gyro, which, when the robot boots,
   * assumes and angle of zero, this needs to be positive when
   * turning left
   * @return the angle of the robot in degrees
   */
  public double getGyroInDeg(){
    return imu.getAngle()*-1;//Pull gyro in degrees
    //note counterclockwise rotation is positive
  }

  /**
   * Returns the speed of rotation of the robot, 
   * counterclockwise is positive.
   * @return degrees per second
   */
  public double getRotationalVelocity(){
    return imu.getRate()*-1;
  }

  /* =================== Pull From All Module Methods =================== */
  
  /**
   * Returns all values from the module's absolute 
   * encoders, and returns them in an array of 
   * doubles, as degrees, in module order.
   * 
   * @return array of doubles, in degrees
   */
  public double[] getAllAbsModuleAngles(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getAbsPosInDeg();
    }
    return moduleAngles;
  }

  public double getContinuousRotAngle(int modNumber) {
    return swerveModules[modNumber].getRotationSensorPosContinuous()/Constants.RAD_TO_ENC_CONV_FACTOR;
  }

  public double getSensorRotPos(int modNumber) {
    return swerveModules[modNumber].getRotationSensorPosContinuous();
  }

  public void resetContinuousRotPos(int modNumber) {
    swerveModules[modNumber].resetRotationSensorPosition();
  }
  

  /**
   * Returns all values from the module's absolute 
   * encoders, and returns them in an array of 
   * doubles, as RADIANS, in module order.
   * 
   * @return array of doubles, in radians
   */
  public double[] getAllAbsModuleAnglesRad(){
    double[] moduleAngles = new double[4];
    for(int i=0; i<4; i++){
      moduleAngles[i]=swerveModules[i].getPosInRad();
    }
    return moduleAngles;
  }

  /**
   * Returns all values from the rotational motor's 
   * reletive encoders in an array of doubles. This 
   * array is in order of module number.
   * 
   * @return array of doubles, representing tick count.
   */
  public double[] getAllModuleRelEnc(){
    double[] moduleRelEnc = new double[4];
    for(int i=0; i<4; i++){
      moduleRelEnc[i]=swerveModules[i].getRelEncCount();
    }
    return moduleRelEnc;
  }

  /**
   * Returns the collective distance as seen by the 
   * drive motor's encoder, for each module.
   * 
   * @return an array of doubles in meters
   */
  public double[] getAllModuleDistance(){
    double[] moduleDistances = new double[4];
    for(int i=0; i<4; i++){
      moduleDistances[i]=swerveModules[i].getDriveDistance();
    }
    return moduleDistances;
  }

  /**
   *  Gets all the drive velocities.
   * 
   * @return An array of velocities.
   */
  public double[] getAllModuleVelocity(){
    double[] moduleVelocities = new double[4];
    for(int i=0; i<4; i++){
      moduleVelocities[i]=swerveModules[i].getDriveVelocity();
    }
    return moduleVelocities;
  }

  /**
   * Gets all the SwerveModulePosition 
   * of all modules.
   * 
   * @return An array of SwerveModulePositions.
   */
  public SwerveModulePosition[] getSwerveModulePositions(){
    //instatiate and construct a 4 large SwerveModuleState array
    SwerveModulePosition[] modulePositions =  new SwerveModulePosition[4];
    //get the current SwerveModuleStates from all modules in array
    for (int i = 0; i < modulePositions.length; i++) {
      modulePositions[i] = swerveModules[i].getModulePosition();
    }

    return modulePositions;
  }
  
  /**  
   * method to configure all modules DriveMotor PIDF
   * these are the PIDF on the TalonFX. This is for
   * testing
   */
  public void setDrivePIDF(double P, double I, double D, double F){
    for (int i=0; i<4; i++){
      swerveModules[i].setDriveMotorPIDF(P, I, D, F);
    }
  }

  
  /**
   * Method for taking the current position of all modules,
   * and making that position the absolute zero of each 
   * modules position respectively.
   */
  public void zeroAllModulePosSensors(){
    //a for loop so cycle through all modules
    for (int i=0; i<4; i++){
      //call the zero position method
      swerveModules[i].zeroAbsPositionSensor();
    }
  }

  public void zeroModulePosSensor(int modNumber){
      swerveModules[modNumber].zeroAbsPositionSensor();
  }

  /**
   * 
   * @param target an angle in radians
   * @return a value to give the rotational input, -1.0 to 1.0
   */
  public double getRobotRotationPIDOut(double target){
    double currentGyroPos = getGyroInRad();
    double output = robotSpinController.calculate(currentGyroPos, target);
    // System.out.println("targetAngle:"+Math.toDegrees(target)+"   angle:"+Math.toDegrees(currentGyroPos)+"atSP:"+robotSpinController.atSetpoint()+"  pid output"+output);
    if(robotSpinController.atSetpoint()){
      return 0.0;
    } else {
      if (Math.abs(output) < Constants.MINIMUM_ROTATIONAL_OUTPUT){
        return Constants.MINIMUM_ROTATIONAL_OUTPUT*Math.signum(output);
      }else {
        return output;
      }
    }
  }

  public double getCounterRotationPIDOut(double target){
    double currentGyroPos = getGyroInRad();
    return robotCounterSpinController.calculate(currentGyroPos, target);
  }

}