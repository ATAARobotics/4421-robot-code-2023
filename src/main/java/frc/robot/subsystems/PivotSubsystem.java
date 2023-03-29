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
    private CANSparkMax pivotMotor2 = new CANSparkMax(Constants.PIVOT_MOTOR2_ID, MotorType.kBrushless);
    private CANCoder pivotEncoder = new CANCoder(Constants.PIVOT_ENCODER_ID);
    // private PIDController pivotPid = new PIDController(0, 0, 0);
    private DigitalInput proximitySwitchBottom;
    private DigitalInput proximitySwitchTop;
    private boolean sensedMetalBottom = false;
    private boolean sensedMetalTop = false;
    
    private double speed = 0.9;
    
    private int direction = 0;

    private double ropeLength = 1;
    private double winchCircumference = 0.0287 * Math.PI;
    private int movementState = 0;
    private boolean aboveTop = true;
    double setpoint;
    public PivotSubsystem() {
        setBrakes(true);
        pivotMotor2.setInverted(true);
        // SmartDashboard.putNumber("Pivot P", 0);
        // SmartDashboard.putNumber("Pivot I", 0);
        // SmartDashboard.putNumber("Pivot D", 0);
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
                    if(pivotEncoder.getPosition() >= -2200){
                        setSpeed(-speed);
                    }else{
                        //setSpeed(-speed/3.5);
                        setSpeed(-0.2);
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
                    setSpeed(speed);
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
                    setSpeed(-speed);
                }
                else{
                    movementState = 4;
                }
                break;
            }
            //double edge detection to align 
            case 4:{
                if(sensedMetalTop){
                    setSpeed(-0.15);
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
                    if (pivotEncoder.getPosition() <= -250){
                        setSpeed(speed);
                    }
                    else{
                        if (pivotEncoder.getPosition() <= 0){
                            setSpeed(0.1);
                        }
                        else{
                            stop();
                        }
                    }
                    break;
                }
            //top switch go down
            case 6:{
                if(!sensedMetalTop){
                    setSpeed(-speed);
                }
                else{
                    movementState = 7;
                }
                break;
            }
            case 7:{
                if(sensedMetalTop){
                    setSpeed(-speed);
                }
                else{
                    movementState = 2;

                }
                break;
            }
            // override for arm to go up 
            case 8:{ 
                setSpeed(speed/2);
                break;
            }   
            case 9:{
                if(pivotEncoder.getPosition() >= -1200){
                    setSpeed(-speed);
                }
                else{
                    stop();
                }
                break;
            }     
            case 10:{
                if(!sensedMetalTop){
                    setSpeed(speed);
                }
                else{
                    movementState = 4;
                }
                break;
            }
            case 11:
                if(!sensedMetalBottom){
                    if(pivotEncoder.getPosition() >= -2200){
                        setSpeed(-speed*0.5);
                    }else{
                        //setSpeed(-speed/3.5);
                        setSpeed(-0.2);
                    }
                }
                else{
                    aboveTop = false;
                    stop();

                }
                break;
            case 12:
                if(!sensedMetalBottom){
                    if(pivotEncoder.getPosition() >= -1050){
                        setSpeed(-speed);
                    }else{
                        stop();
                    }
                }
                else{
                    aboveTop = false;
                    stop();

                }
                break;
            default:
                stop();
        }

    
        
    }

    public void up() {
        movementState = 5;
    }

    public void storedPosition(){
        if(pivotEncoder.getPosition() >= -800){
            movementState = 3;
        }else{
            movementState = 10;
        }
    }
    public void down() {
        movementState = 1;
    }
    public void downSlow(){
        movementState = 11;
    }
    public void firstdown() {
        movementState = 6;
    }

    public void highCube(){
        movementState = 12;
    }

    public void stop() {
        setSpeed(0);
        movementState = 0;
    }

    public void downPosition() {
        movementState = 9;
    }

    public void overridestop() {
        pivotEncoder.setPosition(0);
        setSpeed(0);
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
            pivotMotor2.setIdleMode(IdleMode.kBrake);
        }else{
            pivotMotor.setIdleMode(IdleMode.kCoast);
            pivotMotor2.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setSpeed(double speedy) {
        pivotMotor.set(speedy);
        pivotMotor2.set(speedy);
    }
} 
