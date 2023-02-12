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

    @Override
    public void periodic() {
        switch (direction) {

            case 1:
                if (pivotEncoder.getPosition() < 500 ) {
                    setSpeed(0);
                }
            case 2:
                if (pivotEncoder.getPosition() > 0 ) {
                    setSpeed(1);
                }

            // case 3: 
            // if (pivotEncoder.getPosition() = 0 ) {
            //      setSpeed();
            // }

            default: 
            break;

        }

    }

    public void setUp() {
        setSpeed(1);

    }

    public void setNormalSpeed() {
        setSpeed(2);
    }

    public void setMaxSpeed() {
        setSpeed(3);
    }

    public void setSpeed(int speedLevel) {
        switch (speedLevel) {
            case 1:
                climbMotorSpeed = 0.3;
                break;

            case 2:
                climbMotorSpeed = 0.85;
                break;

            case 3:
                climbMotorSpeed = 1.0;
                break;

            default:
                DriverStation.reportError(speedLevel + " is not a valid speed level for the climber!", false);
                break;
        }
    }

    public void up() {
        pivotMotor.set(1);
        direction = 1;
    }

    public void autoUp() {
        if (autoClimbEnabled) {
            pivotMotor.set(climbMotorSpeed);
        }
    }

    public void stop() {
        pivotMotor.set(0.0);
    }

 
    public void preventAutoClimb() {
        autoClimbEnabled = false;
    }
} 
