package frc.robot.subsystems.other;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedHandler extends SubsystemBase
{
    private final CANdle candle = new CANdle(0, "rio");

    public LedHandler() {
        super();

        // configuration
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);
    }

    // changing colors
    public void SetColor(int r, int g, int b)
    {
        this.candle.setLEDs(r, g, b);
    }

    public void SetBlue()
    {
        SetColor(0, 0, 255);
    }
    
    public void SetGreen()
    {
        SetColor(0, 255, 0);
    }
}