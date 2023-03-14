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

    private int lightsFlip = 1;
    
    public LightingSubsystem () {
        lights = new CANdle(21);
        config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 1;
        lights.configAllSettings(config);
    }

    public void FlipLights(){
        if(lightsFlip == 1 ){
            lights.setLEDs(255, 255, 0, 0, 18, 256);
            lightsFlip = 2 ;
        } else if(lightsFlip == 2){
            lights.setLEDs(255, 0, 255, 0, 18, 256);
            lightsFlip = 3 ;
        }else {
            lights.setLEDs(0, 0, 0, 0, 0, 256);
            lightsFlip = 1;       
        }
    

        SmartDashboard.putNumber("is cone", lightsFlip);
    }

}
