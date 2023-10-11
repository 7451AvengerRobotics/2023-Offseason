package frc.robot.subsystems.other;



//import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.constants.PortConstants;

public class Claw extends SubsystemBase{
 

//Creating Claw and properties of it 
    private final DoubleSolenoid clawSolenoid;
    private final CANSparkMax clawMotorL;
    private final CANSparkMax clawMotorR;
    private static boolean isExtended;


    public Claw(){
        super();

 //Creating Claw and properties of it        
        clawSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, PortConstants.CLAW_PNEUMATIC[0], PortConstants.CLAW_PNEUMATIC[1]);
        clawMotorL = new CANSparkMax(PortConstants.Claw[0], MotorType.kBrushless);
        clawMotorR = new CANSparkMax(PortConstants.Claw[1], MotorType.kBrushless);
        
    }


   
//Extending the Claw
    public void extend(){
        clawSolenoid.set(Value.kForward);
        isExtended = true;
    }

//Retracting the Claw
    public void retract(){
        clawSolenoid.set(Value.kReverse);
        isExtended = false;
    }
//Toggling the Claw
    public void toggle(){
        if(isExtended){
            this.retract();
        }else{
            this.extend();
        }
    }
//Setting power to the claw
    public void setPower(double power){
        clawMotorL.set(power);
        clawMotorR.set(-power);
    }


}
