package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutoConstants;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class IntakeCommand extends CommandBase{
    SwerveDriveSubsystem m_swerveDriveSubsystem;
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
            timer.start();
            timerStarted = true;
        }
        m_intakeSubsystem.runIntake(0);
    }

    @Override
    public boolean isFinished() {
        return (m_intakeSubsystem.isIntakeTimerDone() || timer.get() > Constants.INTAKE_TIME_LIMIT) && timerStarted == true;
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeSubsystem.stopIntake();
        // m_swerveDriveSubsystem.setSwerveDrive(0, 0, 0, false);
    }
}
