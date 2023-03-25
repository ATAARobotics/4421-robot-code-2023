package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class ScoreCone extends SequentialCommandGroup {

    PivotSubsystem m_pivotSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    TelescopingArmSubsystem m_telescopingArmSubsystem;
    String state;
    boolean firstrun = true;
    public ScoreCone(PivotSubsystem pivotSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, IntakeSubsystem intakeSubsystem) {
        addRequirements(pivotSubsystem, telescopingArmSubsystem, intakeSubsystem);
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;
        System.out.println("hello");
        addCommands(
            new InstantCommand(() -> System.out.println("score cone")),
            new InstantCommand(m_pivotSubsystem::downPosition, m_pivotSubsystem),
            new WaitUntilCommand(() -> m_pivotSubsystem.getMovementState() == 0),
            new InstantCommand(() -> m_intakeSubsystem.runIntakeReversed(0), m_intakeSubsystem),
            new InstantCommand(m_telescopingArmSubsystem::in, m_telescopingArmSubsystem),
            new WaitCommand(0.75),
            new InstantCommand(m_intakeSubsystem::stopIntake, m_intakeSubsystem)

        );
        
    }
}
