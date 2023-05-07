package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class ChargeAuto extends SequentialCommandGroup {
    public ChargeAuto(SwerveDrive s_Swerve) {

        addRequirements(s_Swerve);
        setOffseasonPathBlue();
        addCommands(new ParallelCommandGroup(
            s_Swerve.sAutoBuilder("Charge Auto", AutoConstants.chargePath)
        ));
    }

    public void setOffseasonPathBlue() {
        AutoConstants.chargePath.put("Start", new InstantCommand());
        AutoConstants.chargePath.put("Stop", new InstantCommand());
    }
}
