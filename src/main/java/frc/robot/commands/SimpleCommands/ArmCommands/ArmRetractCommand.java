package frc.robot.commands.SimpleCommands.ArmCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.other.Arm;

public class ArmRetractCommand extends CommandBase{
    private final Arm arm;

    public ArmRetractCommand(Arm arm){
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        arm.retract();
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

