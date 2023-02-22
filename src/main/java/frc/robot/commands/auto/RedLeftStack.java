package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.subsystems.*;

public class RedLeftStack extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;

    public RedLeftStack(SwerveDriveSubsystem swerveDriveSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(m_swerveDriveSubsystem);

        addCommands(
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                // score
                
                // drive to midpoint + rotate parallel with (lower arm, run intake(run until finished))
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(12.84, 4.46, new Rotation2d(Math.PI))),
                
                // drive to cone
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(10.92, 4.46, new Rotation2d(0.0))),

                // drive back + rotate parallel with raising arm to scoring pos
                
                // mid point + rotate
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(12.84, 4.46, new Rotation2d(Math.PI))),
                
                // scoring position
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(15.17, 4.80, new Rotation2d(Math.PI)))
                
                // place cone


        );

    }

}