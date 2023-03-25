package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.TelescopingArmCommand;
import frc.robot.subsystems.*;

public class RedLeftGamePiece extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final TelescopingArmSubsystem m_telescopingArmSubsystem;
    private final PivotSubsystem m_pivotSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    public RedLeftGamePiece(SwerveDriveSubsystem swerveDriveSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem, IntakeSubsystem intakeSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_pivotSubsystem = pivotSubsystem;

        addRequirements(m_swerveDriveSubsystem, m_telescopingArmSubsystem, m_pivotSubsystem, m_intakeSubsystem);

        addCommands(
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                new TelescopingArmCommand(m_telescopingArmSubsystem, "cube"),
                new PivotCommand(m_pivotSubsystem, "firstdown"),
                // score
                new OuttakeCommand(m_intakeSubsystem),
                
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(2, 0, new Rotation2d(0)), false),
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(2, 2, new Rotation2d(0)), false),
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(0, 2, new Rotation2d(0)), false),
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(0, 0, new Rotation2d(0)), true)
        );

    }

}