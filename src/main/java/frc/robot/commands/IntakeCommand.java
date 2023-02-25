package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends CommandBase{

    ArmSubsystem m_armSubsystem;
    Timer timer;
    boolean timerStarted = false;

    public IntakeCommand(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        timer = new Timer();
    }

    @Override
    public void execute() {
        if (!timerStarted){
            timerStarted = true;
            timer.start();
        }
        m_armSubsystem.runIntake();
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.isIntakeTimerDone() || timer.get() > Constants.INTAKE_TIME_LIMIT;
    }
}
