package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    PWMSparkMax intake_motor;
    
    public ArmSubsystem () {
        this.intake_motor = new PWMSparkMax(Constants.INTAKE_MOTOR_PORT);
    }

    public CommandBase runIntake() {
        return this.runEnd(() -> intake_motor.set(0.1), () -> intake_motor.set(0.0));
    }
    public CommandBase runIntakeReversed() {
        return this.runEnd(() -> intake_motor.set(-0.1), () -> intake_motor.set(0.0));
    }
}
