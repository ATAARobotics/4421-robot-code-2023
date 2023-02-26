package frc.robot.subsystems;

import org.ejml.dense.row.SpecializedOps_DDRM;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopingArmSubsystem extends SubsystemBase {
    private CANSparkMax telescopingArmMotor = new CANSparkMax(Constants.TELESCOPING_ARM_MOTOR_ID, MotorType.kBrushless);
    private CANCoder telescopingArmEncoder = new CANCoder(Constants.TELESCOPING_ARM_ENCODER_ID);
    private DigitalInput topLimit;
    private DigitalInput bottomLimit;
    private boolean isTopLimit = false;
    private boolean isBottomLimit = false;
    private int movementState = 0;
    private double speed = 0.5;


    public TelescopingArmSubsystem() {
        telescopingArmMotor.setInverted(true);
        telescopingArmMotor.setIdleMode(IdleMode.kBrake);
        this.bottomLimit = new DigitalInput(3);
        this.topLimit = new DigitalInput(4);
    }

    @Override
    public void periodic() {
        CheckLimits();
        SmartDashboard.putNumber("Telescoping arm position", telescopingArmEncoder.getPosition());
        switch(movementState){
            case 1:
                if(!isBottomLimit){
                    telescopingArmMotor.set(-speed);
                }
                else{
                    movementState = 0;
                    telescopingArmMotor.set(0);

                }
                break;
            case 2:{
                if(!isTopLimit){
                    telescopingArmMotor.set(speed);
                }
                else{
                    movementState = 0;
                    telescopingArmMotor.set(0);

                }
                break;
            }
            default:
                telescopingArmMotor.set(0);
                break;
        }
    }


    public void in() {
        telescopingArmMotor.set(-speed);

    }

    public void out() {
        telescopingArmMotor.set(speed);

    }

    public void stop() {
        telescopingArmMotor.set(0.0);
    }

    public void RestEncoder(){
        telescopingArmEncoder.setPosition(0);
    }

    private void CheckLimits(){
        if (!bottomLimit.get()) {
            isBottomLimit = true;
          } else {
            isBottomLimit = false;
        }

        if (!topLimit.get()) {
            isTopLimit = true;
          } else {
            isTopLimit = false;
        }
    }

} 
