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
*   2. Drives over charging station
*   3. Park on driving station
*/

// TODO: Test (based on PathPlanner coordinates)

public class RedLeader extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;

    // for tweaking with tolerances and speed for auto
    double DTOLERANCE = Constants.DTOLERANCE;
    double RTOLERANCE = Constants.RTOLERANCE;
    double SPEEDLIMIT = Constants.SPEEDLIMIT;
    double ROTLIMIT = Constants.ROTLIMIT;

    double outCommunityZone[] = {11.35, 2.36};

    public RedLeader(SwerveDriveSubsystem swerveDriveSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;

        addRequirements(m_swerveDriveSubsystem);

        addCommands(
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                // score
                
                // Drive over charging station
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(outCommunityZone[0], outCommunityZone[1], new Rotation2d(Math.PI)))

                // Auto-Balance on charging station


        );

    }

}