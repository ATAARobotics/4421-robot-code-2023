package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.Constants;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.subsystems.*;

/*  Description of this auto
*   1. Score high cube
*   2. Drives to pick up cone and score
*/

// default angle when beginning = Math.PI

public class RedRightStack extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final ArmSubsystem m_armSubsystem;

    // for tweaking with tolerances and speed for auto
    double DTOLERANCE = Constants.DTOLERANCE;
    double RTOLERANCE = Constants.RTOLERANCE;
    double SPEEDLIMIT = Constants.SPEEDLIMIT;
    double ROTLIMIT = Constants.ROTLIMIT;

    public RedRightStack(SwerveDriveSubsystem swerveDriveSubsystem, ArmSubsystem armSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_armSubsystem = armSubsystem;

        addRequirements(m_swerveDriveSubsystem, m_armSubsystem);

        addCommands(
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                // score
                new OuttakeCommand(m_armSubsystem),
                // drive to midpoint + rotate parallel with (lower arm, run intake(run until finished))
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0], AutoConstants.RED_RIGHT_MID_POINT[1], new Rotation2d(Math.PI))),
                
                // drive to cone
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_GAME_PIECE[0], AutoConstants.RED_RIGHT_GAME_PIECE[1], new Rotation2d(0.0))),

                // drive back + rotate parallel with raising arm to scoring pos
                
                // mid point + rotate
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0], AutoConstants.RED_RIGHT_MID_POINT[1], new Rotation2d(Math.PI))),
                
                // scoring position
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_RIGHT_SCORING[0], AutoConstants.RED_RIGHT_RIGHT_SCORING[1], new Rotation2d(Math.PI))),
                new IntakeCommand(m_armSubsystem)
                // place cone


        );

    }

}