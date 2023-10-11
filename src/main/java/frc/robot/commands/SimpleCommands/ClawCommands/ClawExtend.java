package frc.robot.commands.SimpleCommands.ClawCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.other.Claw;

public class ClawExtend extends CommandBase{
    private final Claw claw;

    public ClawExtend(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize(){
    }

    @Override 
    public void execute(){
        claw.extend();
    }
    
    @Override
    public void end(boolean interrupted){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}

