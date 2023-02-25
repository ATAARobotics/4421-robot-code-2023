package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class OuttakeCommand extends CommandBase{

    ArmSubsystem m_armSubsystem;
    Timer timer;
    boolean timerStarted = false;

    public OuttakeCommand(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        timer = new Timer();
    }

    @Override
    public void execute() {
        if (!timerStarted){
            timerStarted = true;
            timer.start();
        }
        m_armSubsystem.runIntakeReversed();
            
    }

    @Override
    public boolean isFinished() {
        return timer.get() > Constants.OUTTAKE_DELAY;
    }
}
