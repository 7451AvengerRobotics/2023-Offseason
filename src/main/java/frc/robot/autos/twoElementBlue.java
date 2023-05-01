package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class twoElementBlue extends SequentialCommandGroup {
    public twoElementBlue(SwerveDrive s_Swerve) {

        addRequirements(s_Swerve);
        setOffseasonPathBlue();
        addCommands(new ParallelCommandGroup(
            s_Swerve.sAutoBuilder("Offseason Path Blue", AutoConstants.offseasonPathBlue)
        ));
    }

    public void setOffseasonPathBlue() {
        AutoConstants.offseasonPathBlue.put("Start", new InstantCommand());
        AutoConstants.offseasonPathBlue.put("Stop", new InstantCommand());
    }
}
