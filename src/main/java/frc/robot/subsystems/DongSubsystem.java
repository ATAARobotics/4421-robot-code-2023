package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import java.nio.channels.NotYetBoundException;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

public class DongSubsystem extends SubsystemBase{

    TalonFX FrontLeft;
    TalonFX FrontRight;
    TalonFX RearRight;
    TalonFX RearLeft;

    CANcoder encooder;

    boolean LeftInveted;
    boolean RightInverted;

    double angle;
    double tolerance;

    double minAngle;
    double maxAngle;
    PIDController dongController;

    double pidOutput;

    double p;
    double i;
    double d;
    double ff;
    public double setPoint;
    
    public enum Dir{
        UP,
        DOWN
    }

    public DongSubsystem(){
        p = 0.5;
        i = 0.0;
        d = 0.015;
        dongController = new PIDController(p, i, d);
        
        pidOutput = dongController.calculate(angle);

        FrontLeft = new TalonFX((int)Constants.PivotConstants.FrontLeftPivotMotorID, "canivore");
        FrontRight = new TalonFX((int)Constants.PivotConstants.FrontRightPivotMotorID, "canivore");
        RearLeft = new TalonFX((int)Constants.PivotConstants.RearLeftPivotMotorID, "canivore");
        RearRight = new TalonFX((int)Constants.PivotConstants.RearRightPivotMotorID, "canivore");

        LeftInveted = Constants.PivotConstants.leftInverted;
        RightInverted = Constants.PivotConstants.rightInverted;

        encooder = new CANcoder((int)Constants.PivotConstants.EncooderID, "canivore");

        tolerance = Constants.PivotConstants.tolerance;

        minAngle = Constants.PivotConstants.minAngle;
        maxAngle = Constants.PivotConstants.maxAngle;

        angle = encooder.getAbsolutePosition().getValueAsDouble();
        setPoint = angle;

    }

    public void moveUp(){
        moveMotors(Dir.UP, Constants.PivotConstants.maxSpeed);
    }

    public void moveDown(){
        moveMotors(Dir.DOWN, Constants.PivotConstants.maxSpeed);
    }

    public void moveUpPID(){
        
    }

    public void moveDownPID(){

    }


    public void moveMotors(Dir dir, double speed){
        if (dir == Dir.UP){
            if (LeftInveted && !RightInverted){
                FrontLeft.set(-speed);
                RearLeft.set(-speed);

                FrontRight.set(speed);
                RearRight.set(speed);
            }
            else if (RightInverted && !LeftInveted){
                FrontLeft.set(speed);
                RearLeft.set(speed);

                FrontRight.set(-speed);
                RearRight.set(-speed);
            }
            else {
                throw new NotYetBoundException();
            }
        } else if (dir == Dir.DOWN) {
            if (LeftInveted && !RightInverted){
                FrontLeft.set(speed);
                RearLeft.set(speed);

                FrontRight.set(-speed);
                RearRight.set(-speed);
            }
            else if (RightInverted && !LeftInveted){
                FrontLeft.set(-speed);
                RearLeft.set(-speed);

                FrontRight.set(speed);
                RearRight.set(speed);
            }
            else {
                throw new NotYetBoundException();
            }
        } else {
            throw new NotYetBoundException();
        }
    }

    public void stopMotors(){
        FrontLeft.set(0);
        FrontRight.set(0);
        RearLeft.set(0);
        RearRight.set(0);
    }

    public void setSetpoint(double angle) {
        setPoint = angle;
    }

    public void upByTick() {
        setPoint += 0.01;
    }

    public void downByTick() {
        setPoint -= 0.01;
    }

    @Override
    public void periodic() {
        angle = encooder.getAbsolutePosition().getValueAsDouble();
    
        if (setPoint < minAngle) setPoint = minAngle;
        if (setPoint > maxAngle) setPoint = maxAngle;

        if (Math.abs(angle - setPoint) <= tolerance) {
            stopMotors();
        } else if (angle < minAngle) {
            moveUp();
        } else if (angle > maxAngle) {
            moveDown();
        } else if (angle > minAngle && angle > setPoint + tolerance) {
            moveDown();
        } else if (angle < maxAngle && angle < setPoint - tolerance) {
            moveUp();
        }
    
        System.out.println("=========== ENCODER VALUE: " + angle + " | SETPOINT: " + setPoint + " ===========");
        }   
}
    
