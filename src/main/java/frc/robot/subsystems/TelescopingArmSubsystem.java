package frc.robot.subsystems;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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
    private double speed = 1.0;


    public TelescopingArmSubsystem() {
        telescopingArmMotor.setInverted(true);
        telescopingArmMotor.setIdleMode(IdleMode.kBrake);
        this.bottomLimit = new DigitalInput(3);
    }

    @Override
    public void periodic() {
        CheckLimits();
        SmartDashboard.putNumber("Telescoping arm position", telescopingArmEncoder.getPosition());
        switch(movementState){
            case 1:
                if(!isBottomLimit){
                    if( telescopingArmEncoder.getPosition() > 450){
                        telescopingArmMotor.set(-speed);
                    }
                    else{
                        telescopingArmMotor.set(-speed/4);
                    }
                }else{
                    movementState = 0;
                    telescopingArmMotor.set(0);
                    telescopingArmEncoder.setPosition(0);

                }
                break;
            case 2:
                if(telescopingArmEncoder.getPosition() <= 1000){
                    telescopingArmMotor.set(speed);
                }
                else{
                    movementState = 0;
                    telescopingArmMotor.set(0);

                }
                break;
            case 3:
                if(telescopingArmEncoder.getPosition() < 3100){
                    telescopingArmMotor.set(speed);
                }
                else{
                    movementState = 0;
                    telescopingArmMotor.set(0);

                }
                break;
            case 4:
                if(telescopingArmEncoder.getPosition() < 2000){
                    telescopingArmMotor.set(speed);
                }
                else if (telescopingArmEncoder.getPosition() < 2100){
                    telescopingArmMotor.set(speed/4);
                }
                else{
                    movementState = 0;
                    telescopingArmMotor.set(0);

                }
                break;
            default:
                telescopingArmMotor.set(0);
                break;
        }
    }


    public void in() {
        movementState = 1;
    }

    public void out() {
        movementState = 3;

    }
    public void scoreCube(){
        movementState = 2;
    }
    public void scoreCone(double toggle){
        System.out.print(toggle);
        if(toggle >= 0.5){
            movementState = 4;
        }else{
            movementState = 0;
        }
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
    }

    public double getEncoder() {
        return telescopingArmEncoder.getPosition();
    }
    public void setBrakes(boolean toggle){
        if(toggle){
            telescopingArmMotor.setIdleMode(IdleMode.kBrake);
        }else{
            telescopingArmMotor.setIdleMode(IdleMode.kCoast);

        }
    }
    public int getMovementState(){
        return movementState;
    }
} 
