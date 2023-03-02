package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    private PWMSparkMax intake_motor;
    private Counter intake_encoder;


    private double intake_speed = 0.55;
    private double outtake_speed = 0.25;
    private boolean hasGamePiece = false;
    private double intakeDelay = 0.35;
    private double rateCutoff = 0;
    private boolean runStarted = false;
    private Timer timer;
    private boolean sensedMetalBottom = false;
    private boolean sensedMetalTop = false;

    
    public IntakeSubsystem () {
        this.intake_motor = new PWMSparkMax(Constants.INTAKE_MOTOR_PORT);
        this.intake_encoder = new Counter(0);
        this.intake_encoder.setDistancePerPulse(1.0);
        this.timer = new Timer();
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putBoolean("runStarted", runStarted);
        SmartDashboard.putNumber("rate", intake_encoder.getRate());
        SmartDashboard.putBoolean("hasGamePiece", this.hasGamePiece);
    }

    public void timerReset(){
        runStarted = false;
        timer.stop();
        timer.reset();
    }
    public void runIntake() {
        intake_motor.set(intake_speed);
    }

    public void runIntakeReversed(double speedMultiplyer) {
        if(speedMultiplyer >= 0.5){
            intake_motor.set(-outtake_speed*0.85);
        }else{
            intake_motor.set(-outtake_speed);
        }
        hasGamePiece = false;
    }

    public void stopIntake(){
        intake_motor.set(0);

    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    public boolean isIntakeTimerDone() {
        return timer.hasElapsed(intakeDelay);
    }
}
