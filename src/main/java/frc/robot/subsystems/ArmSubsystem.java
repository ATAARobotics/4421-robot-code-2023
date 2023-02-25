package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    PWMSparkMax intake_motor;
    Counter intake_encoder;
    DigitalInput proximitySwitchBottom;
    DigitalInput proximitySwitchTop;


    private double intake_speed = 0.5;
    private double outtake_speed = 1;
    private boolean hasGamePiece = false;
    private double intakeDelay = 0.35;
    private double rateCutoff = 0;
    private boolean runStarted = false;
    private Timer timer;
    private boolean sensedMetalBottom = false;
    private boolean sensedMetalTop = false;

    
    public ArmSubsystem () {
        this.intake_motor = new PWMSparkMax(Constants.INTAKE_MOTOR_PORT);
        this.intake_encoder = new Counter(0);
        this.intake_encoder.setDistancePerPulse(1.0);
        this.timer = new Timer();
        this.proximitySwitchBottom = new DigitalInput(1);
        this.proximitySwitchTop = new DigitalInput(2);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
        SmartDashboard.putBoolean("runStarted", runStarted);
        SmartDashboard.putNumber("rate", intake_encoder.getRate());
        SmartDashboard.putBoolean("hasGamePiece", this.hasGamePiece);
        SmartDashboard.putBoolean("sensedMetalBottom", sensedMetalBottom);
        SmartDashboard.putBoolean("sensedMetalTop", sensedMetalTop);
        checkMetal();
    }

    public void timerReset(){
        runStarted = false;
        timer.stop();
        timer.reset();
    }
    public void runIntake() {
        
        if (intake_encoder.getRate() > rateCutoff) {
            runStarted = true;
        }

        if (!hasGamePiece) {
            // turns motor on
            intake_motor.set(intake_speed);
            // if has gamepiece, turns motor off
            if (runStarted && intake_encoder.getRate() < rateCutoff) {
                timer.start();
            }

            if (timer.hasElapsed(intakeDelay)) {
                intake_motor.set(0.0);
                this.hasGamePiece = true;
            }
        }else{
            intake_motor.set(0.0);
            runStarted = false;
            timer.stop();
            timer.reset();
        };
    }

    public void runIntakeReversed() {
        intake_motor.set(-outtake_speed);
        hasGamePiece = false;
    }

    public void stopIntake(){
        intake_motor.set(0);

    }

    public boolean hasGamePiece() {
        return hasGamePiece;
    }

    public void checkMetal() {
        // System.out.println("BOT: " + proximitySwitchBottom.get());
        // System.out.println("TOP: " + proximitySwitchTop.get());
        if (proximitySwitchBottom.get()) {
            // Proximity switch is triggered
            sensedMetalBottom = true;
            // System.out.println("YAY");
            SmartDashboard.putBoolean("sensedMetalBottom", sensedMetalBottom);
          } else {
            // Proximity switch is not triggered
            sensedMetalBottom = false;
            SmartDashboard.putBoolean("sensedMetalBottom", sensedMetalBottom);
        }

        if (proximitySwitchTop.get()) {
            // Proximity switch is triggered
            sensedMetalTop = true;
            // System.out.println("YAY2");
            SmartDashboard.putBoolean("sensedMetalTop", sensedMetalTop);
          } else {
            // Proximity switch is not triggered
            sensedMetalTop = false;
            SmartDashboard.putBoolean("sensedMetalTop", sensedMetalTop);
        }
    }
}
