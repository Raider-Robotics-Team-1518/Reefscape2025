package frc.robot.commands.struct;

import java.sql.Driver;
import java.util.List;

import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.base.SwerveDrive;

public class Autos {

    public static Command getFullAuto(String pathName) {
        return null;
    }

    // Basic tests
    public static Command autoDriveStraight() {
        return getFullAuto("DriveStraight");
    }

    public static Command autoHorseshoeTest() {
        return getFullAuto("DriveHorseshoeTest");
    }

    public static Command autoSpinSlide() {
        return getFullAuto("180SpinSlide");
    }



}
