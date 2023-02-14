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

public class PivotSubsystem extends SubsystemBase {
    private CANSparkMax pivotMotor = new CANSparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    private CANCoder pivotEncoder = new CANCoder(Constants.PIVOT_ENCODER_ID);
    private double climbMotorSpeed = 0.85;

    private double midElevatorEncoderTicks = 35;
    private double maxElevatorEncoderTicks = 150;

    private int direction = 0;
    private boolean autoClimbEnabled = true;

    private double ropeLength = 1;
    private double winchCircumference = 0.0287 * Math.PI;

    public PivotSubsystem() {
        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);

    }



    public void up() {
        pivotMotor.set(-0.2);
    }


    public void down() {
        pivotMotor.set(0.2);
    }
    public void stop() {
        pivotMotor.set(0.0);
    }

 
    public void preventAutoClimb() {
        autoClimbEnabled = false;
    }
} 
