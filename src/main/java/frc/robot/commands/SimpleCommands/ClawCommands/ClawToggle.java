package frc.robot.commands.SimpleCommands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.other.Claw;

public class ClawToggle extends CommandBase{
    private final Claw claw;

    public ClawToggle(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        claw.toggle();
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

