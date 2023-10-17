package frc.robot.commands.SimpleCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.other.Arm;

public class SolenoidUnlock extends CommandBase{
    private final Arm arm;
    public SolenoidUnlock(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        arm.unlockSolenoid();
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

