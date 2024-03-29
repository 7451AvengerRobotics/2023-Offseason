package frc.robot.constants;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.Gains;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.2;
    public static final boolean tuningMode = false;

    public static final class FieldCoordinates {

    public static Pose2d BlueOrigin = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
    public static Pose2d RedOrigin = new Pose2d(new Translation2d(16.5, 8.0), Rotation2d.fromDegrees(180));
    public static final double[] BLUEcubeYcoord = {1.06, 2.76, 4.45};
    public static final double[] BLUEconeYcoord = {0.53, 1.62, 2.22, 3.35, 3.87, 4.98};
    public static final double[] REDcubeYcoord = {3.57, 5.27, 6.94};
    public static final double[] REDconeYcoord = {3.02, 4.15, 4.71, 5.83, 6.41, 7.52};

    public static final double[] REDsubstationcoord = {1.9, 0.52};
    public static final double[] BLUEsubstationcoord = {6.2, 7.45};

    }
    
    public static final Pose2d REDsubStation = new Pose2d(new Translation2d(15.4, 1.8), Rotation2d.fromDegrees(0));
    public static final Pose2d BLUEsubStation = new Pose2d(new Translation2d(15.4, 6.2), Rotation2d.fromDegrees(0));


    public static final class SwerveConstants {
        public static final int pigeonID = 14;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.73);
        public static final double wheelBase = Units.inchesToMeters(21.73); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; 
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        public static final double driveKS = (0.32 / 12);
        public static final double driveKV = (1.51 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.5; 
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; 

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Brake;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 10;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(99.580078125);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 11;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(146.42578125);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
            
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(329.94140625);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(206.3671875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final Gains GAINS_ANGLE_SNAP = new Gains(0.02, 0.0, 0.0, 0.0, 50);
    
        public static final Gains GAINS_BALANCE = new Gains(0.045, 0.0, 0.0, 0.0, 50);
    
        public static final double SNAP_TOLLERANCE = 2.0;
        public static final double BALANCE_TOLLERANCE = 0.001;
        
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1.1;
        public static final double kPYController = 1.3;
        public static final double kPThetaController = 0.7;
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final HashMap<String, Command> offseasonPathBlueBump = new HashMap<>();
        public static final HashMap<String, Command> offseasonPathBlue = new HashMap<>();
        public static final HashMap<String, Command> testPath = new HashMap<>();
        public static final HashMap<String, Command> chargePath = new HashMap<>();
    }
    
    public static final class LimelightConstants{
        public static final String left = "limelight-newleft";
        public static final String right = "limelight-newrigh";
        public static final String top = "limelight-vtwotop";
    }

    public class ButtonConstants {
        
        public static final int CONTROLLER_PORT = 0;
        public static final int BUTTON_PANEL_PORT = 1;
        /*
         * Button Panel Mapping
         * ---------------------------
         * |          | 11 | 12|     | 
         * |    ^    | 1 | 2 | 3 | 4 |
         * |   < >   | 5 | 6 | 7 | 8 |
         * |    v    | 9 | 10 |      |   
         * ---------------------------
         */
      
        // All Constants are subject to change once we figure out what each one does in the old code.
        //12 is not connected so it is 1-11 
        /* Actual Buttons */
      
        public static final int HighCube = 2;
        public static final int MidCone = 5;
        public static final int MidCube = 6;
      
        public static final int ClawIntake = 3;
        public static final int ClawOuttake = 4;
      
        public static final int TurretLeft = 9;
        public static final int TurretRight = 10;
      
        public static final int Ground = 1;
        public static final int CLAW_TOGGLE = 8;
      
        public static final int ResetEncoder = 11;
      
        public static final int lockSolenoid = 7;
        /* Test Buttons */ 
      }   
}

