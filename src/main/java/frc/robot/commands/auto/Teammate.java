package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.DeadReckoning;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.RotateCommand;
import frc.robot.commands.TelescopingArmCommand;
import frc.robot.subsystems.*;
import frc.robot.AutoConstants;

/*  Description of this auto
*   1. Score high cube
*   2. Drives over charging station
*   3. Park on driving station
*/

// TODO: Test (based on PathPlanner coordinates)

public class Teammate extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final TelescopingArmSubsystem m_telescopingArmSubsystem;
    private final PivotSubsystem m_pivotSubsystem;


    public Teammate(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;

        addRequirements(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingArmSubsystem, m_pivotSubsystem);

        addCommands(                    
            new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
            new PivotCommand(m_pivotSubsystem, "firstdown"),
            new TelescopingArmCommand(m_telescopingArmSubsystem, "cube"),
            // extend arm to shooting pos + moves down pivot
            new WaitCommand(0.25),
            // score
            new OuttakeCommand(m_intakeSubsystem),

            // Drive over charging station with dead-reckoning
            new ParallelCommandGroup(
                new DeadReckoning(m_swerveDriveSubsystem, -1.0, 0.0, 4.0),
                new TelescopingArmCommand(m_telescopingArmSubsystem, "in")
            ),
            new PivotCommand(m_pivotSubsystem, "down")

        );

    }

}