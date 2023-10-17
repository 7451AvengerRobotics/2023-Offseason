package frc.robot;

import java.util.HashMap;
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
import frc.robot.commands.SimpleCommands.SolenoidLock;
import frc.robot.commands.SimpleCommands.SolenoidUnlock;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmExtendCommand;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmRetractCommand;
import frc.robot.commands.SimpleCommands.ArmCommands.ArmToggleCommand;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawIntake;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawOuttake;
import frc.robot.commands.SimpleCommands.ClawCommands.ClawToggle;
import frc.robot.commands.SimpleCommands.VirtualFourBar.VirtualFourBarCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.subsystems.other.Arm;
import frc.robot.subsystems.other.Claw;
import frc.robot.subsystems.other.VirtualFourBar;

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
    //private final JoystickButton robotCentric = new JoystickButton(controller, PS4Controller.Button.kL1.value);
    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    //TODO: Auto Allign will throw an error at the moment if we use it because we cannot determine game piece. We can use controller feedback or we can use other sensors,

    /* Subsystems */
    private final SwerveDrive s_Swerve = new SwerveDrive();  
    private final Arm arm;
    private final Claw claw;
    private final VirtualFourBar bar;
    public static HashMap<Command, String> autoMap = new HashMap<>();
    public static SendableChooser<Command> chooser = new SendableChooser<>();
     /* Subsystems */


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        /* Initializing Subsystems */
        arm = new Arm();
        claw = new Claw();
        bar = new VirtualFourBar();

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -(controller.getRawAxis(translationAxis))*-0.8, 
                () -> -(controller.getRawAxis(strafeAxis))*-0.8, 
                () -> -(controller.getRawAxis(rotationAxis))*-0.8
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
        configureBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureBindings() {

        /* Button Mapping */
 
 
     /* Actual Buttons */
 
    //  JoystickButton midCone = new JoystickButton(buttonPanel, ButtonConstants.MidCone);
    //  JoystickButton midCube = new JoystickButton(buttonPanel, ButtonConstants.MidCube);
 
    //  JoystickButton highCube = new JoystickButton(buttonPanel, ButtonConstants.HighCube);
    //  JoystickButton grabObject = new JoystickButton(buttonPanel, ButtonConstants.Ground);
    //  JoystickButton resetBar = new JoystickButton(buttonPanel, ButtonConstants.ResetEncoder);
 
    //  JoystickButton clawIntake = new JoystickButton(buttonPanel, ButtonConstants.ClawIntake);
    //  JoystickButton clawOuttake = new JoystickButton(buttonPanel, ButtonConstants.ClawOuttake);
    //  JoystickButton clawToggle = new JoystickButton(buttonPanel, ButtonConstants.CLAW_TOGGLE);
 
    //  JoystickButton turretRight = new JoystickButton(buttonPanel, ButtonConstants.TurretLeft);
    //  JoystickButton turretLeft = new JoystickButton(buttonPanel, ButtonConstants.TurretRight);
 
    //  JoystickButton lockSolenoid = new JoystickButton(buttonPanel, ButtonConstants.lockSolenoid);
 
     /* Actual Buttons */
 
 
 
 
     /*Actual Command Mapping */
    // midCone.onTrue(new EncoderandArm(bar, arm, 62464)); // 5
    // midCube.onTrue(new StandardEncoder(bar, arm, 9732)); // 6
 
 
    //  highCube.onTrue(new EncoderandArm(bar, arm, 30786)); // 2
    //  grabObject.onTrue(new StandardEncoder(bar, arm, 69977)); // 1
    //  resetBar.onTrue(new ResetVFbarEncoder(bar, arm, 0)); // 11
 
 
    //  clawIntake.whileTrue(new ClawIntake(claw, 1)); // 3
    //  clawOuttake.whileTrue(new ClawOuttake(claw, -1)); // 4
    //  clawToggle.whileTrue(new ClawToggle(claw)); // 8
 
    //  lockSolenoid.onTrue(new SolenoidCommand(arm)); // 7
 
 
    //  turretRight.whileTrue(new TurretTestCommand(turret, 0.3));
    //  turretLeft.whileTrue(new TurretTestCommand(turret, -0.3));
     /*Actual Command Mapping */
    //testing to see keshav can access git 
 
 
 
 
 
 
 
     /*    TestButton Mapping */
        JoystickButton armToggle = new JoystickButton(buttonPanel, 1);
        JoystickButton upVFBAR = new JoystickButton(buttonPanel, 2);
        JoystickButton reverseVFBAR = new JoystickButton(buttonPanel, 3);
        JoystickButton clawToggle = new JoystickButton(buttonPanel, 4);
        JoystickButton clawIntake = new JoystickButton(buttonPanel, 5);
        JoystickButton clawOutake = new JoystickButton(buttonPanel, 6);
        // JoystickButton lockSolenoid = new JoystickButton(buttonPanel, 7);
        // JoystickButton unlockSolenoid = new JoystickButton(buttonPanel, 8);
     /* TestButton Mapping */
        armToggle.onTrue(new ArmToggleCommand(arm));
        clawToggle.onTrue(new ClawToggle(claw));
        upVFBAR.whileTrue(new VirtualFourBarCommand(bar, arm, -0.4));
        reverseVFBAR.whileTrue(new VirtualFourBarCommand(bar, arm, 0.4));
        clawIntake.whileTrue(new ClawIntake(claw, -0.7));
        clawOutake.whileTrue(new ClawIntake(claw, 0.7));
     /* Test Mapping */

     /* Test Mapping */
 
     /* Command Mapping */
 
   }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return chooser.getSelected();
        //test to se if kunasl 
    }
}
