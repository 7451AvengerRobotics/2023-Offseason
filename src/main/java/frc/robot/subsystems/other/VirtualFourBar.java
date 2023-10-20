package frc.robot.subsystems.other;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;
import frc.robot.constants.TurretConstants;

public class VirtualFourBar extends SubsystemBase {

    private final CANSparkMax vFBAR;
    private final RelativeEncoder encoder;

    public VirtualFourBar(){
        
//Instantiating the Virtual Four Bar
        vFBAR = new CANSparkMax(22, MotorType.kBrushless);
        encoder = vFBAR.getEncoder();

    }


//Function allows user to set falcon power based on percentage




    public double getEncoderPosition(){
        return encoder.getPosition();
    }

    public void setPosition(double position){
        encoder.setPosition(position);
    }


    public void setPower(double power){
        vFBAR.set(power);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());
    }

//Function allows user to reset the encoder position
  

}
