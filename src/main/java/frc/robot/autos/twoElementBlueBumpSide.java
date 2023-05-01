package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class twoElementBlueBumpSide extends SequentialCommandGroup {
    public twoElementBlueBumpSide(SwerveDrive s_Swerve) {

        addRequirements(s_Swerve);
        setOffseasonPathBlueBump();
        addCommands(new ParallelCommandGroup(
            s_Swerve.sAutoBuilder("Offseason Path Blue Bump", AutoConstants.offseasonPathBlueBump)
        ));
    }

    public void setOffseasonPathBlueBump() {
        AutoConstants.offseasonPathBlueBump.put("Start", new InstantCommand());
        AutoConstants.offseasonPathBlueBump.put("Stop", new InstantCommand());
    }
}
