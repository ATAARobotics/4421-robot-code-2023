package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.subsystems.*;

/*  Description of this auto
*   1. Score high cube
*   2. Drives to pick up cone and score
*/

// default angle when beginning = Math.PI

public class RedLeftStack extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;

    // for tweaking with tolerances and speed for auto
    double DTOLERANCE = Constants.DTOLERANCE;
    double RTOLERANCE = Constants.RTOLERANCE;
    double SPEEDLIMIT = Constants.SPEEDLIMIT;
    double ROTLIMIT = Constants.ROTLIMIT;

    double leftMidPoint[] = {12.38, 0.86};
    double leftGamePiece[] = {10.92, 0.86};
    double leftLeftScoring[] = {15.17, 0.16};

    public RedLeftStack(SwerveDriveSubsystem swerveDriveSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(m_swerveDriveSubsystem);

        addCommands(
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                // score
                
                // drive to midpoint + rotate parallel with (lower arm, run intake(run until finished))
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(leftMidPoint[0], leftMidPoint[1], new Rotation2d(Math.PI))),
                
                // drive to cone
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(leftGamePiece[0], leftGamePiece[1], new Rotation2d(0.0))),

                // drive back + rotate parallel with raising arm to scoring pos
                
                // mid point + rotate
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(leftMidPoint[0], leftMidPoint[1], new Rotation2d(Math.PI))),
                
                // scoring position
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(leftLeftScoring[0], leftLeftScoring[1], new Rotation2d(Math.PI)))
                
                // place cone


        );

    }

}