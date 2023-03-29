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

public class ScoreCube extends SequentialCommandGroup {

    PivotSubsystem m_pivotSubsystem;
    IntakeSubsystem m_intakeSubsystem;
    TelescopingArmSubsystem m_telescopingArmSubsystem;
    String state;
    boolean firstrun = true;
    public ScoreCube(PivotSubsystem pivotSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, IntakeSubsystem intakeSubsystem) {
        addRequirements(pivotSubsystem, telescopingArmSubsystem, intakeSubsystem);
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;
        System.out.println("hello");
        addCommands(
            new InstantCommand(m_pivotSubsystem::highCube, m_pivotSubsystem),
            new InstantCommand(m_telescopingArmSubsystem::scoreCube, m_telescopingArmSubsystem),
            new WaitUntilCommand(() -> m_telescopingArmSubsystem.getMovementState() == 0)

        );
        
    }
}
