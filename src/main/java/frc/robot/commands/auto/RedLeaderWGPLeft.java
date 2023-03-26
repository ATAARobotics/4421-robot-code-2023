package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.DeadReckoning;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.TelescopingArmCommand;
import frc.robot.subsystems.*;
import frc.robot.AutoConstants;

/*  Description of this auto (WGP = With Game Piece)
*   1. Score high cube
*   2. Drives over charging station
*   3. Drive and pick up game piece
*   4. Drive near charging station
*   5. Park on driving station
*/

// TODO: Test (based on PathPlanner coordinates)

public class RedLeaderWGPLeft extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final TelescopingArmSubsystem m_telescopingArmSubsystem;
    private final PivotSubsystem m_pivotSubsystem;

    

    public RedLeaderWGPLeft(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;
        addRequirements(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingArmSubsystem, m_pivotSubsystem);

        double startingX = m_swerveDriveSubsystem.getOdometry().startingX;
        double startingY = m_swerveDriveSubsystem.getOdometry().startingY;
        double startingRot = m_swerveDriveSubsystem.getOdometry().startingRot;

        addCommands(                    
            new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
            new InstantCommand(() -> m_swerveDriveSubsystem.setInitialPose()),

                new PivotCommand(m_pivotSubsystem, "firstdown"),
                new WaitCommand(0.25),
                // score
                new OuttakeCommand(m_intakeSubsystem),

                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(-0.4, -0.7, new Rotation2d(0)), false),
                

                // Drive over charging station with dead-reckoning
                new DeadReckoning(m_swerveDriveSubsystem, -2.0, 0.0, 2.8),

                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(m_swerveDriveSubsystem.getOdometry().getPoseMeters().getX(), m_swerveDriveSubsystem.getOdometry().getPoseMeters().getY(), new Rotation2d(Math.PI)), true),
                

                new PivotCommand(m_pivotSubsystem, "down"),

                new ParallelRaceGroup(
                    new IntakeCommand(m_intakeSubsystem),
                    new DeadReckoning(m_swerveDriveSubsystem, -1, 0, 1.25)
                ),

                new ParallelCommandGroup(
                    new DeadReckoning(m_swerveDriveSubsystem, 1, 1, 0.5),
                    new PivotCommand(m_pivotSubsystem, "up")
                ),

                // Auto-Balance on charging station
                new WaitCommand(0.25),
                new DeadReckoning(m_swerveDriveSubsystem, 1.0, 0.0, 2),
                new AutoBalance(m_swerveDriveSubsystem, true)

        );

    }

}