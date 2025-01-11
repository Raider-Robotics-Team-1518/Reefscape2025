package frc.robot.commands.fmu;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class MoveArmToAngle extends Command {
    private double powerUp = Constants.MotorSpeeds.armPowerUp;
    // private double powerDn = Constants.MotorSpeeds.armPowerDn;
    private double current_angle = RobotContainer.fmu.get_arm_position();
    private double set_angle = current_angle;
    private boolean isDone = false;
    
    public MoveArmToAngle(double set_angle) {
        addRequirements(RobotContainer.fmu);
        this.set_angle = set_angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
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
            isDone = true;
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

        return isDone;
    }

}
