package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve.SwerveDrive;


public class TeleopSwerve extends CommandBase {    
    private SwerveDrive s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier m_halfSpeed;
    private BooleanSupplier m_quarterSpeed;
    private BooleanSupplier bothSpeed;

    public TeleopSwerve(SwerveDrive s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, 
                        BooleanSupplier halfSpeed, BooleanSupplier quarterSpeed, BooleanSupplier bothSpeed) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.m_halfSpeed = halfSpeed;
        this.m_quarterSpeed = quarterSpeed;
        this.bothSpeed = bothSpeed;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if(m_quarterSpeed.getAsBoolean()){
            // xVal = xVal*0.25;
            // yVal =yVal*0.25;
            // if(!rotateWithButton){
            //     rotationVal = rotationVal *0.25;
            // }
            rotationVal = rotationVal *0.25;
        }
        else if (bothSpeed.getAsBoolean()){
            strafeVal = strafeVal*0.36;
            translationVal =translationVal*0.36;
            rotationVal = rotationVal *0.26;

        }
        else if(m_halfSpeed.getAsBoolean()){
            strafeVal = strafeVal*0.4;
            translationVal =translationVal*0.4;
        }
        else{
            strafeVal = strafeVal*1.0;
            translationVal =translationVal*1.0;
            rotationVal = rotationVal *1.0;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed), 
            rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
            true, 
            true
        );
    }
}