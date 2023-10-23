package frc.robot.subsystems.other;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VirtualFourBar extends SubsystemBase {

    private final CANSparkMax vFBAR;
    private final RelativeEncoder encoder;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
    private SparkMaxPIDController m_pidController;

    public VirtualFourBar(){
        
//Instantiating the Virtual Four Bar
        vFBAR = new CANSparkMax(22, MotorType.kBrushless);
        encoder = vFBAR.getEncoder();

        m_pidController = vFBAR.getPIDController();

        kP = 0.1; 
        kI = 1e-4;
        kD = 1; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setIZone(kIz);
        m_pidController.setFF(kFF);
        m_pidController.setOutputRange(kMinOutput, kMaxOutput);


    }

    public double getEncoderPosition(){
        return encoder.getPosition();
    }

    public void setPosition(double position){
        m_pidController.setReference(position, CANSparkMax.ControlType.kPosition);
    }


    public void setPower(double power){
        vFBAR.set(power);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("EncoderPosition", encoder.getPosition());
    }
  

}
