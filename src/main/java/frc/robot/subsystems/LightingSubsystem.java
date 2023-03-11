package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LightingSubsystem extends SubsystemBase{
    private CANdle lights;
    private CANdleConfiguration config;

    
    public enum lightsState {
        Yellow,
        Purple,
        Red,
        None
    }

    private lightsState lightsColor = lightsState.None;

    public LightingSubsystem () {
        lights = new CANdle(21);
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        lights.configAllSettings(config);
    }

    public void FlipLights(){
        
        SmartDashboard.putString("Light State", lightsColor.toString());
        
        switch (lightsColor) {
            case Yellow:
                lights.setLEDs(255, 255, 0, 0, 8, 128);
                lightsColor = lightsState.Purple;
                break;
            case Purple:
                lights.setLEDs(255, 0, 255, 0, 8, 128);
                lightsColor = lightsState.None;
                break;
            case None:
                lights.setLEDs(0, 0, 0, 0, 8, 128);
                lightsColor = lightsState.Yellow;
                break;
            case Red:
                break; 
        }
    }

    public void hasGamePieceLights() {
        lightsColor = lightsState.Red;
        lights.setLEDs(255, 0, 0, 0, 8, 128);
    }

    public void resetLights() {
        lightsColor = lightsState.None;
        lights.setLEDs(0, 0, 0, 0, 8, 128); 
    }
}
