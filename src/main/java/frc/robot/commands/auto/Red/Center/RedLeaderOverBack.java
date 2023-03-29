package frc.robot.commands.auto.Red.Center;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.DeadReckoning;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.TelescopingArmCommand;
import frc.robot.subsystems.*;
import frc.robot.AutoConstants;

/*  Description of this auto
*   1. Score high cube
*   2. Drives over charging station
*   3. Park on driving station
*/

// TODO: Test (based on PathPlanner coordinates)

public class RedLeaderOverBack extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final TelescopingArmSubsystem m_telescopingArmSubsystem;
    private final PivotSubsystem m_pivotSubsystem;

    

    public RedLeaderOverBack(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;

        addRequirements(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingArmSubsystem, m_pivotSubsystem);

        addCommands(                    
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),

                //get into high cube scoring position
                new InstantCommand(m_pivotSubsystem::highCube, m_pivotSubsystem),
                new InstantCommand(m_telescopingArmSubsystem::scoreCube, m_telescopingArmSubsystem),
                new WaitUntilCommand(() -> m_telescopingArmSubsystem.getMovementState() == 0),
                // score
                new InstantCommand(() -> m_intakeSubsystem.runIntakeReversed(1), m_intakeSubsystem),
                new WaitCommand(0.4),
                new InstantCommand(m_intakeSubsystem::stopIntake, m_intakeSubsystem),
                new InstantCommand(m_pivotSubsystem::up, m_pivotSubsystem),
                new InstantCommand(m_telescopingArmSubsystem::in, m_telescopingArmSubsystem),

                // Drive over charging station with dead-reckoning
                
                //new DeadReckoning(m_swerveDriveSubsystem, -1.0, 0.0, 1.5),
                new DeadReckoning(m_swerveDriveSubsystem, -1.0, 0.0, 5),

                // Auto-Balance on charging station
                new WaitCommand(0.25),
                new DeadReckoning(m_swerveDriveSubsystem, 3.0, 0.0, 1.5),
                new AutoBalance(m_swerveDriveSubsystem, true)

        );

    }

}