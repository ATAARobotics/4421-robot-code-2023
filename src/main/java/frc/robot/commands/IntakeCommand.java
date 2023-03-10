package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase{

    IntakeSubsystem m_intakeSubsystem;
    Timer timer;
    boolean timerStarted = false;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        timer = new Timer();
    }

    @Override
    public void execute() {
        if (!timerStarted){
            timerStarted = true;
            timer.start();
        }
        m_intakeSubsystem.runIntake(0);
    }

    @Override
    public boolean isFinished() {
        return m_intakeSubsystem.isIntakeTimerDone() || timer.get() > Constants.INTAKE_TIME_LIMIT;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntake();
    }
}
