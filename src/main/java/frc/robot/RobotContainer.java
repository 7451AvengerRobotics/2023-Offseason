package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.AutoConstants;
import frc.robot.subsystems.Swerve.SwerveDrive;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final PS4Controller controller = new PS4Controller(Constants.ButtonConstants.CONTROLLER_PORT) ;
    private final Joystick buttonPanel = new Joystick(Constants.ButtonConstants.BUTTON_PANEL_PORT);
    private final JoystickButton robotCentric = new JoystickButton(controller, PS4Controller.Button.kL1.value);
    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    //TODO: Auto Allign will throw an error at the moment if we use it because we cannot determine game piece. We can use controller feedback or we can use other sensors,

    /* Subsystems */
    private final SwerveDrive s_Swerve = new SwerveDrive();

    public static HashMap<Command, String> autoMap = new HashMap<>();

    public Command sAutoBuilder(String pathName, HashMap<String, Command> eventMap) {
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            s_Swerve::getPose, // Pose2d supplier
            s_Swerve::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
            Constants.SwerveConstants.swerveKinematics, // SwerveDriveKinematics
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            s_Swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            s_Swerve);
        
        List<PathPlannerTrajectory> pathToFollow = PathPlanner.loadPathGroup(pathName,
            PathPlanner.getConstraintsFromPath(pathName));
        final Command auto = autoBuilder.fullAuto(pathToFollow);
        autoMap.put(auto, pathName);
        return auto;
    }

    public static SendableChooser<Command> chooser = new SendableChooser<>();
    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -controller.getRawAxis(translationAxis), 
                () -> -controller.getRawAxis(strafeAxis), 
                () -> -controller.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        setOffseasonPath();

        Shuffleboard.getTab("AUTON").add(chooser).withSize(3, 1);
        Command instantCmd = new InstantCommand();
        chooser.setDefaultOption("Nothing", instantCmd);
        chooser.addOption("Offseason Path", sAutoBuilder("Offseason Path", AutoConstants.offseasonPath));
        chooser.addOption("Example Path", new exampleAuto(s_Swerve));
        autoMap.put(instantCmd, "nothing");

        // Configure the button bindings
        configureButtonBindings();
    }

    public void setOffseasonPath() {
        AutoConstants.offseasonPath.put("Start", new InstantCommand());
        AutoConstants.offseasonPath.put("Stop", new InstantCommand());
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return chooser.getSelected();
    }
}
