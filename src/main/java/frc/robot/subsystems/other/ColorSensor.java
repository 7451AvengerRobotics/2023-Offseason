package frc.robot.subsystems.other;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor extends SubsystemBase
{
    private final ColorSensorV3 sensor = new ColorSensorV3(null);
    private final LedHandler ledHandler = new LedHandler();
    private int prevDistance = 0;


    ColorSensor()
    {
        super();
    }

    public Color DetectColor()
    {
        return sensor.getColor();
    }

    @Override
    public void periodic() 
    {
        int distance = sensor.getProximity();
        if (distance!=prevDistance)
        {
            ledHandler.SetGreen();
        }
        else
        {
            ledHandler.SetBlue();
        }

        this.prevDistance = distance;
    }
}