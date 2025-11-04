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

    CANcoder encoder;

    boolean LeftInverted;
    boolean RightInverted;

    double angle;
    double tolerance;

    double minAngle;
    double maxAngle;
    PIDController dongController;

    double pidOutput;

    double ff;
    final double ffValue = Constants.PivotConstants.ffValue;
    public double setPoint;
    public double speed;
    
    public enum Dir{
        UP,
        DOWN
    }

    public DongSubsystem(){

        dongController = new PIDController(Constants.PivotConstants.p, Constants.PivotConstants.i, Constants.PivotConstants.d);
        
        pidOutput = dongController.calculate(angle);

        FrontLeft = new TalonFX(Constants.PivotConstants.FrontLeftPivotMotorID, "canivore");
        FrontRight = new TalonFX(Constants.PivotConstants.FrontRightPivotMotorID, "canivore");
        RearLeft = new TalonFX(Constants.PivotConstants.RearLeftPivotMotorID, "canivore");
        RearRight = new TalonFX(Constants.PivotConstants.RearRightPivotMotorID, "canivore");

        LeftInverted = Constants.PivotConstants.leftInverted;
        RightInverted = Constants.PivotConstants.rightInverted;

        encoder = new CANcoder(Constants.PivotConstants.EncoderID, "canivore");

        tolerance = Constants.PivotConstants.tolerance;

        minAngle = Constants.PivotConstants.minAngle;
        maxAngle = Constants.PivotConstants.maxAngle;

        angle = encoder.getAbsolutePosition().getValueAsDouble();
        setPoint = angle;

    }

    public void moveUp(){
        moveMotors(Dir.UP, speed);
    }

    public void moveDown(){
        moveMotors(Dir.DOWN, speed);
    }


    public void moveMotors(Dir dir, double speed){
        if (dir == Dir.UP){
            if (LeftInverted && !RightInverted){
                FrontLeft.set(-speed);
                RearLeft.set(-speed);

                FrontRight.set(speed);
                RearRight.set(speed);
            }
            else if (RightInverted && !LeftInverted){
                FrontLeft.set(speed);
                RearLeft.set(speed);

                FrontRight.set(-speed);
                RearRight.set(-speed);
            }
            else {
                throw new NotYetBoundException();
            }
        } else if (dir == Dir.DOWN) {
            if (LeftInverted && !RightInverted){
                FrontLeft.set(speed);
                RearLeft.set(speed);

                FrontRight.set(-speed);
                RearRight.set(-speed);
            }
            else if (RightInverted && !LeftInverted){
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
        angle = encoder.getAbsolutePosition().getValueAsDouble();
        double ffangle;
        if (angle < Constants.PivotConstants.minAngle) {
            ffangle = -0.01;
        } else {
            ffangle = angle;
        }
        ff = ffValue * Math.cos(ffangle * (Math.PI/2) * (1/maxAngle));
        speed = dongController.calculate(angle, setPoint);
        if (speed >= 0) {
            speed += ff;
        }
        
        if (Math.abs(speed) > Constants.PivotConstants.maxSpeed) {
            if (speed > 0) {
                speed = Constants.PivotConstants.maxSpeed;
            } else {
                speed = -Constants.PivotConstants.maxSpeed;
            }
        }
        speed = Math.abs(speed);
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
    
        System.out.println("=========== ENCODER VALUE: " + angle + " | SETPOINT: " + setPoint + " ===========" + "\n\n\n Speed " + speed + "\n\n\n");
        }   
}
    
