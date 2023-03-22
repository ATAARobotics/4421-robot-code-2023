package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ScoreCone extends CommandBase {

    PivotSubsystem m_pivotSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    TelescopingArmSubsystem m_telescopingArmSubsystem;
    String state;
    boolean firstrun = true;
    public ScoreCone(PivotSubsystem pivotSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;
    }

    @Override
    public void initialize() {
        new SequentialCommandGroup(
            new InstantCommand(m_pivotSubsystem::downPosition, m_pivotSubsystem),
            new WaitUntilCommand(() -> m_pivotSubsystem.getMovementState() == 0),
            new InstantCommand(() -> m_intakeSubsystem.runIntakeReversed(0), m_intakeSubsystem),
            new InstantCommand(m_telescopingArmSubsystem::in, m_telescopingArmSubsystem)
           );
    }

    @Override
    public boolean isFinished() {
        return m_pivotSubsystem.getMovementState() == 0 && m_telescopingArmSubsystem.getMovementState() == 0;
    }
}
