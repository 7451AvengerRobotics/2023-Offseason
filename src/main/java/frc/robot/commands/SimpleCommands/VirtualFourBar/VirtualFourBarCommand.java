package frc.robot.commands.SimpleCommands.VirtualFourBar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.other.Arm;
import frc.robot.subsystems.other.VirtualFourBar;

public class VirtualFourBarCommand extends CommandBase{
    private final VirtualFourBar bar;
    private final Arm arm;
    private final double power;
    public VirtualFourBarCommand(VirtualFourBar bar, Arm arm, double power){
        this.bar = bar;
        this.power = power;
        this.arm = arm;
        addRequirements(bar);
    }

    @Override
    public void initialize(){
        arm.unlockSolenoid();

    }

    @Override 
    public void execute(){
        bar.setPower(power);
    }
    
    @Override
    public void end(boolean interrupted){
        bar.setPower(0);
        arm.lockSolenoid();

    }

    @Override
    public boolean isFinished(){
        arm.lockSolenoid();
        return true;
    }
}

