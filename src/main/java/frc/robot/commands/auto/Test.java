package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.subsystems.*;

public class Test extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;

    public Test(SwerveDriveSubsystem swerveDriveSubsystem) {
        addRequirements(swerveDriveSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;

        addCommands(
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 180)),
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0], AutoConstants.RED_RIGHT_MID_POINT[1], new Rotation2d(0)))
        );

    }

}