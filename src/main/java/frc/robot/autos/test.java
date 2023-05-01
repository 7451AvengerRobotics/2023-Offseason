package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class test extends SequentialCommandGroup {
    public test(SwerveDrive s_Swerve) {

        addRequirements(s_Swerve);
        setTestPath();
        addCommands(new ParallelCommandGroup(
            s_Swerve.sAutoBuilder("Test", AutoConstants.testPath)
        ));
    }

    public void setTestPath() {
        AutoConstants.testPath.put("Start", new InstantCommand());
        AutoConstants.testPath.put("Stop", new InstantCommand());
    }
}
