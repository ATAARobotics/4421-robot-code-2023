package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
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

public class RedLeaderWGP extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;

    // for tweaking with tolerances and speed for auto
    double DTOLERANCE = Constants.DTOLERANCE;
    double RTOLERANCE = Constants.RTOLERANCE;
    double SPEEDLIMIT = Constants.SPEEDLIMIT;
    double ROTLIMIT = Constants.ROTLIMIT;

    

    public RedLeaderWGP(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_swerveDriveSubsystem, m_intakeSubsystem);

        addCommands(                    
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                // score
                new OuttakeCommand(m_intakeSubsystem),
                
                // TODO: Drive over charging station with dead-reckoning (meteo)

                // drive to pick up game piece
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_MID_RIGHT_GAME_PIECE[0], AutoConstants.RED_MID_RIGHT_GAME_PIECE[1], new Rotation2d(0)), true),
                    new IntakeCommand(m_intakeSubsystem)
                ),
                

                // drive close to the charging station
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_MID_RIGHT_CHARGING_STATION[0], AutoConstants.RED_MID_RIGHT_CHARGING_STATION[1], new Rotation2d(Math.PI)), false)

                // TODO: Auto-Balance on charging station


        );

    }

}