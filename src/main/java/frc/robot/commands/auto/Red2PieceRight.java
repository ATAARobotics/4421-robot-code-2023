package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

/*  Description of this auto
*   1. Score high cube
*   2. Drives over charging station
*   3. Park on driving station
*/

// TODO: Test (based on PathPlanner coordinates)

public class Red2PieceRight extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final TelescopingArmSubsystem m_telescopingArmSubsystem;
    private final PivotSubsystem m_pivotSubsystem;

    // for tweaking with tolerances and speed for auto
    double DTOLERANCE = Constants.DTOLERANCE;
    double RTOLERANCE = Constants.RTOLERANCE;
    double SPEEDLIMIT = Constants.SPEEDLIMIT;
    double ROTLIMIT = Constants.ROTLIMIT;

    

    public Red2PieceRight(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;

        addRequirements(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingArmSubsystem, m_pivotSubsystem);

        addCommands(                    
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                new InstantCommand(() -> m_swerveDriveSubsystem.setInitialPose(new Pose2d(AutoConstants.RED_RIGHT_MID_SCORING[0], AutoConstants.RED_RIGHT_MID_SCORING[1], new Rotation2d(0)))),
                
                new PivotCommand(m_pivotSubsystem, "firstdown"),
                new WaitCommand(0.25),
                // score
                new OuttakeCommand(m_intakeSubsystem),

                // drive to midpoint + rotate + lower arm
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0], AutoConstants.RED_RIGHT_MID_POINT[1], new Rotation2d(0)), false),
                
                // drive to cone + parallel with intake
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_GAME_PIECE[0], AutoConstants.RED_RIGHT_GAME_PIECE[1], new Rotation2d(Math.PI+0.01)), false),
                    new PivotCommand(m_pivotSubsystem, "down")
                ),

                new ParallelCommandGroup(
                    new IntakeCommand(m_intakeSubsystem, m_swerveDriveSubsystem),
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_GAME_PIECE[0]-0.60, AutoConstants.RED_RIGHT_GAME_PIECE[1], new Rotation2d(Math.PI+0.01)), false)
                ),

                // mid point + rotate + raising arm to scoring pos + extending
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0], AutoConstants.RED_RIGHT_MID_POINT[1], new Rotation2d(0)), false),
                    new PivotCommand(m_pivotSubsystem, "up")
                ),
                
                // scoring position
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_RIGHT_SCORING[0], AutoConstants.RED_RIGHT_RIGHT_SCORING[1]-0.15, new Rotation2d(0)), false)


        );

    }

}