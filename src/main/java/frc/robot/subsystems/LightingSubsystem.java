package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase {
    private CANdle lights;
    private CANdleConfiguration config;
    private Boolean wantCone = true;
    public LightingSubsystem() {
        lights = new CANdle(Constants.CANDLE_ID);
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        lights.configAllSettings(config);
    }

    public void lightYellow() {
        lights.setLEDs(Constants.yellow[0], Constants.yellow[1], Constants.yellow[2], 0, 0, 128);
        wantCone = false;
    }
    public void lightPurple() {
        lights.setLEDs(Constants.purple[0], Constants.purple[1], Constants.purple[2], 0, 0, 128);
        wantCone = true;
    }
    public void lightRed() {
        lights.setLEDs(Constants.red[0], Constants.red[1], Constants.red[2], 0, 0, 128);
    }
    public void lightBlue() {
        lights.setLEDs(Constants.blue[0], Constants.blue[1], Constants.blue[2], 0, 0, 128);
    }
    public BooleanSupplier getAliance() {
        return () -> SmartDashboard.getBoolean("IsRedAliance", false);
    }
    public BooleanSupplier wantCone() {
        return () -> wantCone;
    }
}