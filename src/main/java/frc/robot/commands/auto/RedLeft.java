package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.subsystems.*;

public class RedLeft extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;

    public RedLeft(SwerveDriveSubsystem swerveDriveSubsystem) {
        addRequirements(swerveDriveSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;

        addCommands(
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                // score
                
                // drive to cone + rotate parallel with (lower arm, run intake(run until finished))
                new ParallelCommandGroup(new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(14.6, 6.7, new Rotation2d(0)))) 

                // drive back + rotate parallel with raising arm to scoring pos
                // new ParallelCommandGroup(new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(14.6, 6.7, new Rotation2d(0))))

                // place cone
        );

    }

}