package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends CommandBase{

    IntakeSubsystem m_intakeSubsystem;
    Timer timer;
    boolean timerStarted = false;

    public OuttakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        timer = new Timer();
    }

    @Override
    public void execute() {
        if (!timerStarted){
            timerStarted = true;
            timer.start();
        }
        m_intakeSubsystem.runIntakeReversed();
            
    }

    @Override
    public boolean isFinished() {
        if (timerStarted){
            return timer.get() > Constants.OUTTAKE_DELAY;
        }else{
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntake();
    }

    
}
