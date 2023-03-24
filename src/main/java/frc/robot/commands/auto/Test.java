package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.*;

public class Test extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    public Test(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem) {
        addRequirements(swerveDriveSubsystem, intakeSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addCommands(
                new IntakeCommand(m_intakeSubsystem, m_swerveDriveSubsystem)
        );

    }

}