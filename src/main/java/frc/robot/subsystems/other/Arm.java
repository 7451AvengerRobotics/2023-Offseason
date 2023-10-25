package frc.robot.subsystems.other;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
    // holds true if the big arm is extended
    private static boolean isExtended = false;
    // takes in air for pneumatics 
    private final Compressor compressor;
    // controls the solenoid for the big arm
    private final Solenoid armSolenoid;
    // locks solenoid for the mini arm
    private final Solenoid lockSolenoid;

    public Arm() {
        super();
        // initializing compressor and solenoid
        compressor = new Compressor(0, PneumaticsModuleType.CTREPCM); 
        armSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0); // need to change module id
        lockSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 6);
    }


    /*
    Stops the compressor in certain scenarios will most likely not be used
    */
    public void stop() {
        compressor.disable();
    }

     /*
    Starts the compressor. Has a digital pressure sensor so it will stop on its own
    */
    public void start() {
        compressor.enableDigital();
    }


    /*Extends the giant arm */
    public void extend() {
        // sets the solenoid output to on
        armSolenoid.set(true);
        isExtended = true;
    }

    /*Retracts the giant arm */
    public void retract() {
        // sets the solenoid output to off
        armSolenoid.set(false);
        isExtended = false;
    }


    /*Allows us to use one button to toggle the arm */
    public void toggle(){
        if(!isExtended){
            this.extend();
        }else{
            this.retract();
        }
    }

    /*Locking the solenoid for the VFBAR */
    public void unlockSolenoid() {
        lockSolenoid.set(true);
    }

    /*Unlocking the solenoid for the VFBAR */
    public void lockSolenoid() {
        lockSolenoid.set(false);
    }

    //Allows us to retrieve the arm state of the robot at the time of call
    public boolean getArmState() {
        return isExtended;
    }

}
