package frc.robot.commands.SimpleCommands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.other.Claw;

public class ClawRetract extends CommandBase{
    private final Claw claw;

    public ClawRetract(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        claw.retract();
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

