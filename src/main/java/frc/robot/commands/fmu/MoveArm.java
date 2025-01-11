package frc.robot.commands.fmu;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Utils;

public class MoveArm extends Command {
    private double powerUp = Constants.MotorSpeeds.armPowerUp;
    private double powerDn = Constants.MotorSpeeds.armPowerDn;

    public MoveArm() {
        addRequirements(RobotContainer.fmu);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(Robot.robotContainer.getCoDriverAxis(Axis.kLeftY) > 0.0d) {
            if (RobotContainer.fmu.get_arm_position() < Constants.Limits.armMaxAngle) {
                RobotContainer.fmu.move_arm(Robot.robotContainer.getCoDriverAxis(Axis.kLeftY) * powerUp);
            }
            else {
                RobotContainer.fmu.stop_arm();
            }
        }
        else {
            if (RobotContainer.fmu.get_arm_position() > Constants.Limits.armMinAngle) {
                RobotContainer.fmu.move_arm(Robot.robotContainer.getCoDriverAxis(Axis.kLeftY) * powerDn);
            }
            else {
                RobotContainer.fmu.stop_arm();
            }
        }

        SmartDashboard.putNumber("ARM ANGLE", Utils.round2prec(RobotContainer.fmu.get_arm_position(), 1));

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //RobotContainer.fmu.stop_arm();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {

        return false;
    }

}
