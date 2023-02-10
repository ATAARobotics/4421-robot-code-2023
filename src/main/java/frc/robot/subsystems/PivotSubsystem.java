/*package frc.robot.subsystems;

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

    private boolean autoClimbEnabled = true;

    private double ropeLength = 1;
    private double winchCircumference = 0.0287 * Math.PI;

    public PivotSubsystem() {
        pivotMotor.setInverted(true);
        pivotMotor.setIdleMode(IdleMode.kBrake);

        resetElevatorEncoder();
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
        pivotMotor.set(climbMotorSpeed);
    }

    public void down() {
        if (!atMin()) {
            pivotMotor.set(-climbMotorSpeed);
        } else {
            pivotMotor.set(0.0);
        }
    }

    public void autoUp() {
        if (autoClimbEnabled) {
            pivotMotor.set(climbMotorSpeed);
        }
    }

    public void autoDown() {
        if (!atMin()) {
            if (autoClimbEnabled) {
                pivotMotor.set(-climbMotorSpeed);
            }
        } else {
            pivotMotor.set(0.0);
        }
    }

    public void stop() {
        pivotMotor.set(0.0);
    }

    public void resetElevatorEncoder() {
        m_elevatorEncoder.setPosition(0);
    }

    public boolean atMin() {
        return !elevatorDownDetector.get();
    }

    public boolean atMid() {
        return m_elevatorEncoder.getPosition() >= midElevatorEncoderTicks;
    }

    public boolean atMax() {
        return m_elevatorEncoder.getPosition() >= maxElevatorEncoderTicks;
    }

    public void slowRate() {
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 0); // Disable
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 0); // Disable
        climbMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 0); // Disable
    }
 
    public void preventAutoClimb() {
        autoClimbEnabled = false;
    }
} */
