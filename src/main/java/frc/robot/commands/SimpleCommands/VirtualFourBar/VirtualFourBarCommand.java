package frc.robot.commands.SimpleCommands.VirtualFourBar;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.other.Arm;
import frc.robot.subsystems.other.VirtualFourBar;

public class VirtualFourBarCommand extends CommandBase{
    private final VirtualFourBar bar;
    private final double power;
    private final Arm arm;
    public VirtualFourBarCommand(VirtualFourBar bar, Arm arm, double power){
        this.bar = bar;
        this.arm = arm;
        this.power = power;
    }

    @Override
    public void initialize(){

    }

    @Override 
    public void execute(){
        arm.unlockSolenoid();
        bar.setPower(power);
    }
    
    @Override
    public void end(boolean interrupted){
        arm.lockSolenoid();
        bar.setPower(0);


    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

