package frc.robot.subsystems;

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
    private double climbMotorSpeed = 0.85;

    private double midElevatorEncoderTicks = 35;
    private double maxElevatorEncoderTicks = 150;

    private int direction = 0;
    private boolean autoClimbEnabled = true;

    private double ropeLength = 1;
    private double winchCircumference = 0.0287 * Math.PI;

    double setpoint;

    public PivotSubsystem() {
        pivotMotor.setIdleMode(IdleMode.kBrake);
        SmartDashboard.putNumber("Pivot P", 0);
        SmartDashboard.putNumber("Pivot I", 0);
        SmartDashboard.putNumber("Pivot D", 0);

    

    }

    public void periodic() {
        // SmartDashboard.putNumber("Pivot Speed", pivotPid.calculate(pivotEncoder.getPosition()));
        
        // pivotMotor.set(pivotPid.calculate(pivotEncoder.getPosition()));

        // SmartDashboard.putNumber("Pivot Encoder", pivotEncoder.getPosition());
        // SmartDashboard.putNumber("SetPoint", setpoint);
    
    }

    public void up() {
        //pivotMotor.set(-0.2);
        setpoint = 0;
        pivotPid.setSetpoint(setpoint);
    
    }


    public void down() {
        //pivotMotor.set(0.2);
        pivotPid = new PIDController(SmartDashboard.getNumber("Pivot P", 0), SmartDashboard.getNumber("Pivot I", 0), 
        SmartDashboard.getNumber("Pivot D", 0));
    }

    public void midway(){
        setpoint = 500;
        pivotPid.setSetpoint(setpoint);

    }

    public void negmidway(){
        setpoint = -500;
        pivotPid.setSetpoint(setpoint);

    }
    public void stop() {
        pivotMotor.set(0.0);
    }

 
    public void preventAutoClimb() {
        autoClimbEnabled = false;
    }
} 
