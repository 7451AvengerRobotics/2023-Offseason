package frc.robot.subsystems.Swerve;

import frc.lib.util.GamePiece;
import frc.lib.util.GamePiece.GamePieceType;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SwerveConstants.Mod0;
import frc.robot.constants.Constants.SwerveConstants.Mod1;
import frc.robot.constants.Constants.SwerveConstants.Mod2;
import frc.robot.constants.Constants.SwerveConstants.Mod3;
import frc.robot.vision.Limelight;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.HashMap;
import java.util.List;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    private Field2d field;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    private Limelight limelight;
    public static HashMap<Command, String> autoMap = new HashMap<>();


    public SwerveDrive() {
        gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
        gyro.configFactoryDefault();
        // limelight = new Limelight(this);
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
            new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
            new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
            new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getModulePositions());
        field = new Field2d();
        SmartDashboard.putData("Field", field);
    }

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.setYaw(0);
    }


    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    public Pose2d getVisionPose(){
        return limelight.getCurrentPose();
    }

    public Pose2d getNearestGridPose(){
        //Check the cameras to see where we are
        double currentY = this.getVisionPose().getY();
        double closestY = 0;

        if(GamePiece.getGamePiece() == GamePieceType.Cube){
            //set the default starting position for Y
            if(DriverStation.getAlliance() == Alliance.Blue){
                 closestY = Constants.FieldCoordinates.BLUEcubeYcoord[0];
                for(int i = 1; i < Constants.FieldCoordinates.BLUEcubeYcoord.length; i++){
                    if(Math.abs(Constants.FieldCoordinates.BLUEcubeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                        System.out.println("Closest Y Coord: " + closestY);
                        closestY = Constants.FieldCoordinates.BLUEcubeYcoord[i];
                    }
                }
                return new Pose2d(new Translation2d(1.82, closestY), Rotation2d.fromDegrees(180));
            }
            if(DriverStation.getAlliance() == Alliance.Red){
                closestY =  Constants.FieldCoordinates.REDcubeYcoord[0];
                for(int i = 1; i<  Constants.FieldCoordinates.REDcubeYcoord.length; i++){
                    if(Math.abs( Constants.FieldCoordinates.REDcubeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                        closestY =  Constants.FieldCoordinates.REDcubeYcoord[i];
                    }
                }
                return new Pose2d(new Translation2d(1.82, closestY), Rotation2d.fromDegrees(180));
            }           
        }
        if(GamePiece.getGamePiece() == GamePieceType.Cone){
            //set the default starting position for Y
            if(DriverStation.getAlliance() == Alliance.Blue){
                closestY = Constants.FieldCoordinates.BLUEconeYcoord[0];
               for(int i = 1; i <  Constants.FieldCoordinates.BLUEconeYcoord.length; i++){
                   if(Math.abs( Constants.FieldCoordinates.BLUEconeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                       System.out.println("Closest Y Coord: " + closestY);
                       closestY =  Constants.FieldCoordinates.BLUEconeYcoord[i];
                   }
               }
               return new Pose2d(new Translation2d(1.82, closestY), Rotation2d.fromDegrees(180));
           }
           if(DriverStation.getAlliance() == Alliance.Red){
            closestY =  Constants.FieldCoordinates.REDconeYcoord[0];
            for(int i = 1; i< Constants.FieldCoordinates.REDconeYcoord.length; i++){
                if(Math.abs( Constants.FieldCoordinates.REDconeYcoord[i] - currentY) < Math.abs(closestY - currentY)){
                    closestY =  Constants.FieldCoordinates.REDconeYcoord[i];
                }
            }
            return new Pose2d(new Translation2d(1.82, closestY), Rotation2d.fromDegrees(180));
        }     
     }
     //Defaults to the middle-ish of the grid but this shouldn't happen
     return new Pose2d(new Translation2d(2.3, 3.0), Rotation2d.fromDegrees(180));
 }

 public void drive(ChassisSpeeds speeds){
    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(swerveModuleStates);
}


 public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    double dt = TimedRobot.kDefaultPeriod;

    ChassisSpeeds chassisSpeeds = fieldRelative
    ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, swerveOdometry.getPoseMeters().getRotation())
    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

    Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * dt, chassisSpeeds.vyMetersPerSecond * dt,
        Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * dt));
    Twist2d twist_vel = new Pose2d().log(robot_pose_vel);
    ChassisSpeeds updated = new ChassisSpeeds(twist_vel.dx / dt, twist_vel.dy / dt, twist_vel.dtheta / dt);

    SwerveModuleState[] swerveModuleStates = Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(updated);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for(SwerveModule mod : mSwerveMods){
        mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void stopDrive(){
        drive(new Translation2d(0, 0), 0, false, true);
    }

    public void setYaw(double yaw){
        gyro.setYaw(yaw);
    }

    public Rotation2d getYaw() {
        return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public double getPitch(){
        return gyro.getPitch();
    }

    public double getRoll(){
        return gyro.getRoll();
    }


    @Override
    public void periodic(){
        swerveOdometry.update(getYaw(), getModulePositions());  
        field.setRobotPose(getPose());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
        }

        SmartDashboard.putString("Alliance Color", DriverStation.getAlliance().name());
        // SmartDashboard.putNumber("Swerve Estimated Pos", limelight.getCurrentPose().getY());

        swerveOdometry.update(getYaw(), getModulePositions()); 

        Pose2d fieldRalativePos = getPose();
        
        if(DriverStation.getAlliance() == Alliance.Red)
            field.setRobotPose(fieldRalativePos.relativeTo(Constants.FieldCoordinates.RedOrigin));

        if(DriverStation.getAlliance() == Alliance.Blue)
            field.setRobotPose(fieldRalativePos.relativeTo(Constants.FieldCoordinates.BlueOrigin));

        SmartDashboard.putData("Field", field);

    }

    public SequentialCommandGroup followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        PIDController thetaController = new PIDController(0.5, 0, 0);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        return new SequentialCommandGroup(
        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if(isFirstPath){
              this.resetOdometry(traj.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            traj, 
            this::getPose, // Pose supplier
            Constants.SwerveConstants.swerveKinematics, // SwerveDriveKinematics
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            thetaController, // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            this::setModuleStates, // Module states consumer
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this // Requires this drive subsystem
        )
        .andThen(() -> stopDrive())
    );
     }

     public Command sAutoBuilder(String pathName, HashMap<String, Command> eventMap) {
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            this::getPose, // Pose2d supplier
            this::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.SwerveConstants.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            this::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            this);
        
        List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName,
            PathPlanner.getConstraintsFromPath(pathName));
        final Command auto = autoBuilder.fullAuto(pathToFollow);
        autoMap.put(auto, pathName);
        return auto;
    }

    public void XLock(){
        SwerveModuleState[] desiredStates = new SwerveModuleState[4];
        desiredStates[0] = new SwerveModuleState(0, new Rotation2d(Mod0.angleOffset.getDegrees() + 45));
        desiredStates[1] = new SwerveModuleState(0, new Rotation2d(Mod1.angleOffset.getDegrees() - 45));
        desiredStates[2] = new SwerveModuleState(0, new Rotation2d(Mod2.angleOffset.getDegrees() - 45));
        desiredStates[3] = new SwerveModuleState(0, new Rotation2d(Mod3.angleOffset.getDegrees() + 45));
        setModuleStates(desiredStates);
      }

}