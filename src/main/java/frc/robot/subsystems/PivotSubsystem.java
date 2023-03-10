package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import org.ejml.dense.row.SpecializedOps_DDRM;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
public class PivotSubsystem extends SubsystemBase {
    private CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    private CANCoder pivotEncoder = new CANCoder(Constants.PIVOT_ENCODER_ID);
    private PIDController pivotPid = new PIDController(0, 0, 0);
    private DigitalInput proximitySwitchBottom;
    private DigitalInput proximitySwitchTop;
    private boolean sensedMetalBottom = false;
    private boolean sensedMetalTop = false;
    
    private double speed = 0.85;
    
    private int direction = 0;

    private double ropeLength = 1;
    private double winchCircumference = 0.0287 * Math.PI;
    private int movementState = 0;
    private boolean aboveTop = true;
    double setpoint;
    public PivotSubsystem() {
        pivotMotor.setIdleMode(IdleMode.kBrake);
        SmartDashboard.putNumber("Pivot P", 0);
        SmartDashboard.putNumber("Pivot I", 0);
        SmartDashboard.putNumber("Pivot D", 0);
        this.proximitySwitchBottom = new DigitalInput(1);
        this.proximitySwitchTop = new DigitalInput(2);
        pivotEncoder.setPosition(0);
    }

    public void periodic() {
        checkMetal();
        SmartDashboard.putNumber("Pivot Speed", pivotEncoder.getPosition());
        switch(movementState){
            //down switch always detect
            case 1:
                if(!sensedMetalBottom){
                    if(pivotEncoder.getPosition() >= -1700){
                        pivotMotor.set(-speed);
                    }else{
                        pivotMotor.set(-speed/3);
                    }
                }
                else{
                    aboveTop = false;
                    stop();

                }
                break;
            //top switch go up
            case 2:{
                if(!sensedMetalTop){
                    pivotMotor.set(speed);
                }
                else{
                    aboveTop = false;
                    stop();
                }
                break;
            }
            //top switch go down
            case 3:{
                if(!sensedMetalTop){
                    pivotMotor.set(-speed);
                }
                else{
                    movementState = 4;
                }
                break;
            }
            //double edge detection to align 
            case 4:{
                if(sensedMetalTop){
                    pivotMotor.set(-speed);
                }
                else{
                    stop();
                    aboveTop = false;

                }
                break;
            }
            //store the arm near the start position 
            case 5:{
                    if(sensedMetalTop){
                        aboveTop = true;
                    }
                    if (pivotEncoder.getPosition() <= 0){
                        pivotMotor.set(speed);
                    }
                    else{
                        stop();
                    }
                    break;
                }
            //top switch go down
            case 6:{
                if(!sensedMetalTop){
                    pivotMotor.set(-speed);
                }
                else{
                    movementState = 7;
                }
                break;
            }
            case 7:{
                if(sensedMetalTop){
                    pivotMotor.set(-speed);
                }
                else{
                    movementState = 2;

                }
                break;
            }
            // override for arm to go up 
            case 8:{ 
                pivotMotor.set(speed/2);
                break;
            }         
            default:
                stop();
        }

    
        
    }

    public void up() {
        movementState = 5;
    }

    public void storedPosition(){
            movementState = 3;
    }
    public void down() {
        System.out.println("movement state down");
        movementState = 1;

    }
    public void firstdown() {
        System.out.println("movement state down");
        movementState = 6;

    }
    public void stop() {
        pivotMotor.set(0);
        movementState = 0;
    }

    public void overridestop() {
        pivotEncoder.setPosition(0);
        pivotMotor.set(0);
        movementState = 0;   
    }
     public void forceup() {
        movementState = 8;   
     }

    public void checkMetal() {
        if (!proximitySwitchBottom.get()) {
            sensedMetalBottom = true;
          } else {
            sensedMetalBottom = false;
        }

        if (!proximitySwitchTop.get()) {
            sensedMetalTop = true;
          } else {
            sensedMetalTop = false;
        }
    }
    public int getMovementState(){
        return movementState;
    }
    public void RestEncoder(){
        pivotEncoder.setPosition(0);
    }
    public void setBrakes(boolean toggle){
        if(toggle){
            pivotMotor.setIdleMode(IdleMode.kBrake);
        }else{
            pivotMotor.setIdleMode(IdleMode.kCoast);

        }
    }
} 
