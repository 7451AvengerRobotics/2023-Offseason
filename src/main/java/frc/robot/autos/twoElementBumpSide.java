package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class twoElementBumpSide extends SequentialCommandGroup {
    public twoElementBumpSide(SwerveDrive s_Swerve) {
        PathPlannerTrajectory main = PathPlanner.loadPath("Offseason Path", 3,2);

        addRequirements(s_Swerve);
        addCommands(new ParallelCommandGroup(
            s_Swerve.followTrajectoryCommand(main, true).alongWith(new InstantCommand())
        ));
    }
}
