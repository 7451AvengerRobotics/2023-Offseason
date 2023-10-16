package frc.robot.subsystems.other;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;
import frc.robot.constants.TurretConstants;

public class VirtualFourBar extends SubsystemBase {

    private final CANSparkMax vFBAR;

    public VirtualFourBar(){
        
//Instantiating the Virtual Four Bar
        vFBAR = new CANSparkMax(22, MotorType.kBrushless);
        

//Configuring the PID Loop for Set to Position and for the soft limits so the arm cannot go past that encoder position
    

//Configuring the PID Values of the Virutal Four bar
   
    }


//Function allows user to set falcon power based on percentage
    public void setPower(double power){
        vFBAR.set(power);
    }



//Function allows user to reset the encoder position
  

}
