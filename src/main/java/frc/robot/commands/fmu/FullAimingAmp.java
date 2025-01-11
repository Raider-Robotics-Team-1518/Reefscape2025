package frc.robot.commands.fmu;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class FullAimingAmp extends Command {
    private double powerUp = Constants.MotorSpeeds.armPowerUp;
    private boolean isTargetVisible = RobotContainer.limeLight1.isTargetVisible();
    private int targetID = (int) RobotContainer.llHelpers.getFiducialID("limelight");
    // private double powerDn = Constants.MotorSpeeds.armPowerDn;
    private double current_angle = RobotContainer.fmu.get_arm_position();
    private double set_angle = current_angle;
    private double horizOffset = RobotContainer.limeLight1.getTargetOffsetHorizontal() * Math.PI / 180.0d;
    private double robotPose = RobotContainer.swerveDrive.getGyroInRad();
    private boolean v_aligned = false;
    private boolean h_aligned = false;

    public FullAimingAmp() {
        addRequirements(RobotContainer.fmu);
        addRequirements(RobotContainer.swerveDrive);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Check to see if LimeLight has acquired target lock
        if (isTargetVisible) { // April Tag Visible
            set_angle = 96.0d;
            horizOffset = RobotContainer.limeLight1.getTargetOffsetHorizontal() * Math.PI / 180.0d;
            robotPose = RobotContainer.swerveDrive.getGyroInRad();

        } else {
            set_angle = Constants.Limits.armDefaultAmpAngle;
            horizOffset = 0;
            h_aligned = true;
        }
        // Check value of shoulder encoder
        current_angle = RobotContainer.fmu.get_arm_position();
        // Calculate power curve proportional
        powerUp = Math.abs(this.set_angle - current_angle) / 100;
        // Move arm up or down to default speaker angle
        if (Math.abs(this.set_angle - current_angle) > Constants.Tolerances.armAimingTolerance) {
            double v_sign = Math.signum(this.set_angle - current_angle);
            RobotContainer.fmu.move_arm(v_sign * (powerUp + 0.25d));
        } else {
            RobotContainer.fmu.stop_arm();
            v_aligned = true;
        }
    
        // Rotate Robot to center on April Tag
        if (Math.abs(horizOffset) > 0.05d ) { // Constants.Tolerances.armAimingTolerance) {
            //double h_sign = Math.signum(0 - horizOffset);
            //System.out.println("Turning to " + targetPose);
            //new DriveTurnToAngleInRad(targetPose);  // h_sign * (powerSteer + 0.2d)
            double targetPose = robotPose + horizOffset;
            double output = RobotContainer.swerveDrive.getRobotRotationPIDOut(targetPose);
            //System.out.println("targetPose " +targetPose +" robotPose " +robotPose + " h_offset " +horizOffset +" pid output " +output);
            RobotContainer.swerveDrive.driveRobotCentric(0, 0, -output, false, true);

        } else {
            RobotContainer.swerveDrive.stopAllModules();
            h_aligned = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //RobotContainer.fmu.stop_arm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (v_aligned && h_aligned) {
            return true;
        } else {
            return false;
        }
    }

}
