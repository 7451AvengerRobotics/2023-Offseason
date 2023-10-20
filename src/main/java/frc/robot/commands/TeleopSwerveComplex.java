package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve.SwerveDrive;


public class TeleopSwerveComplex extends CommandBase {    
    private SwerveDrive s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private PIDController m_thetaController;
    private BooleanSupplier m_90, m_180, m_270, m_0;

    public TeleopSwerveComplex(SwerveDrive s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, 
                        DoubleSupplier rotationSup, BooleanSupplier zero, BooleanSupplier ninety, 
                        BooleanSupplier oneEighty, BooleanSupplier twoSeventy) {
        
        m_0 = zero;
        m_180 = oneEighty;
        m_90 = ninety;
        m_270 = twoSeventy;
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
    }

    @Override 
    public void initialize(){
        m_thetaController = new PIDController(
        Constants.SwerveConstants.GAINS_ANGLE_SNAP.kP, 
        Constants.SwerveConstants.GAINS_ANGLE_SNAP.kI, 
        Constants.SwerveConstants.GAINS_ANGLE_SNAP.kD
        );
        
        m_thetaController.enableContinuousInput(-180, 180);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

        boolean rotateWithButton = m_0.getAsBoolean() || m_90.getAsBoolean() || m_180.getAsBoolean() || m_270.getAsBoolean();
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        if(rotateWithButton){
            if(m_0.getAsBoolean()){
                m_thetaController.setSetpoint(0.0);
            }
            else if(m_90.getAsBoolean()){
                m_thetaController.setSetpoint(-90.0);
            }
            else if(m_180.getAsBoolean()){
                m_thetaController.setSetpoint(180.0);
            }
            else if(m_270.getAsBoolean()){
                m_thetaController.setSetpoint(90.0);
            }
            rotationVal = m_thetaController.calculate((MathUtil.inputModulus(s_Swerve.getPose().getRotation().getDegrees(), -180, 180)), m_thetaController.getSetpoint());
            rotationVal = MathUtil.clamp(rotationVal, -Constants.SwerveConstants.maxAngularVelocity * 0.075, Constants.SwerveConstants.maxAngularVelocity * 0.075);
        }

        else if (!rotateWithButton){
            rotationVal = (MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband));
;
        }

        /* Drive */
        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.SwerveConstants.maxSpeed), 
            rotationVal * Constants.SwerveConstants.maxAngularVelocity, 
            true, 
            false
        );
    }
}