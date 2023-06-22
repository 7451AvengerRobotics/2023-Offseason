package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.constants.Constants;
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

    public static SendableChooser<Command> chooser = new SendableChooser<>();
    


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -(controller.getRawAxis(translationAxis))*0.5, 
                () -> -(controller.getRawAxis(strafeAxis))*0.5, 
                () -> -(controller.getRawAxis(rotationAxis))*0.5, 
                () -> robotCentric.getAsBoolean()
            )
        );


        Shuffleboard.getTab("Auton").add(chooser).withSize(3, 1);
        Command instantCmd = new InstantCommand();
        chooser.setDefaultOption("Nothing", instantCmd);
        // chooser.addOption("Offseason Path", sAutoBuilder("Offseason Path", AutoConstants.offseasonPath));
        chooser.addOption("Example Path", new exampleAuto(s_Swerve));
        chooser.addOption("1st Offseason Path test", new twoElementBlueBumpSide(s_Swerve));
        chooser.addOption("2nd Offseason Path test", new twoElementBlue(s_Swerve));
        chooser.addOption("Balance Path", new ChargeAuto(s_Swerve));
        chooser.addOption("test path", new test(s_Swerve));
        autoMap.put(instantCmd, "nothing");
        // Configure the button bindings
        configureButtonBindings();
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
