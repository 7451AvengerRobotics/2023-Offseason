package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class twoElementBumpSide extends SequentialCommandGroup {
    public twoElementBumpSide(SwerveDrive s_Swerve) {

        addRequirements(s_Swerve);
        setOffseasonPath();
        addCommands(new ParallelCommandGroup(
            s_Swerve.sAutoBuilder("Offseason Path", AutoConstants.offseasonPath)
        ));
    }

    public void setOffseasonPath() {
        AutoConstants.offseasonPath.put("Start", new InstantCommand());
        AutoConstants.offseasonPath.put("Stop", new InstantCommand());
    }
}
