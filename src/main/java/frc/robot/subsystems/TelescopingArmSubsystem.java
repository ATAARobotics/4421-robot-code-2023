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

    private int direction = 0;

    public TelescopingArmSubsystem() {
        telescopingArmMotor.setInverted(true);
        telescopingArmMotor.setIdleMode(IdleMode.kBrake);

    }

    @Override
    public void periodic() {
        switch (direction) {

            case 1:
                if (telescopingArmEncoder.getPosition() < 500 ) {
                    telescopingArmMotor.set(0);
                } else {
                    telescopingArmMotor.set(0.1);
                }
            case 2:
                if (telescopingArmEncoder.getPosition() > 0 ) {
                    telescopingArmMotor.set(0);
                } else {
                    telescopingArmMotor.set(-0.1);
                }

            default: 
                telescopingArmMotor.set(0);
                break;

        }

    }

    public void backward() {
        telescopingArmMotor.set(-0.1);
        direction = 2;
    }

    public void forward() {
        telescopingArmMotor.set(0.1);
        direction = 1;
    }

    public void stop() {
        telescopingArmMotor.set(0.0);
    }

} 
