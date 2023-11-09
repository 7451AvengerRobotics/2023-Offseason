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

        vFBAR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
        vFBAR.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    
        vFBAR.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) 42.54);
        vFBAR.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) -1.4);
        

        kP = 0.03; 
        kI = 0;
        kD = 0; 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 0.5; 
        kMinOutput = -0.5;

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
