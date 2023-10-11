package frc.robot.subsystems.other;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;
import frc.robot.constants.TurretConstants;

public class VirtualFourBar extends SubsystemBase {

    private final WPI_TalonFX vFBAR;

    public VirtualFourBar(){
        
//Instantiating the Virtual Four Bar
        vFBAR = new WPI_TalonFX(PortConstants.VirtualFourBar);
        vFBAR.setNeutralMode(NeutralMode.Brake);
        vFBAR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

//Configuring the PID Loop for Set to Position and for the soft limits so the arm cannot go past that encoder position
        vFBAR.setSensorPhase(true);
        vFBAR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, TurretConstants.kTimeoutMs);  
        vFBAR.selectProfileSlot(0, 0);
        vFBAR.setSelectedSensorPosition(0, 0,TurretConstants.kTimeoutMs);  
        vFBAR.configForwardSoftLimitThreshold(98590);
        vFBAR.configReverseSoftLimitThreshold(-300);
        vFBAR.configForwardSoftLimitEnable(true);
        vFBAR.configReverseSoftLimitEnable(true);

//Configuring the PID Values of the Virutal Four bar
        vFBAR.config_kP(0, 0.02);
        vFBAR.config_kI(0, 0);
        vFBAR.config_kD(0, 9);
    }


//Function allows user to set falcon power based on percentage
    public void setPower(double power){
        vFBAR.set(ControlMode.PercentOutput, power);
    }

//Function allows user to set falcon power based on encoder position
    public void setPosition(double encoderPosition){
        vFBAR.set(ControlMode.Position, encoderPosition);
    }

//Function allows user to retrieve the encoder 
    public double getencoderValues() {
        return vFBAR.getSelectedSensorPosition();
    }

//Function allows user to reset the encoder position
    public void zeroSensors() {
        vFBAR.getSensorCollection().setIntegratedSensorPosition(0, TurretConstants.kTimeoutMs);
    }

}
