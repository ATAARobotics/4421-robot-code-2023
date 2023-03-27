package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoBalance;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.DeadReckoning;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.ScoreCone;
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

    

    public Red2PieceRight(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;

        double startingX = m_swerveDriveSubsystem.getOdometry().startingX;
        double startingY = m_swerveDriveSubsystem.getOdometry().startingY;
        double startingRot = m_swerveDriveSubsystem.getOdometry().startingRot;

        addRequirements(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingArmSubsystem, m_pivotSubsystem);

        addCommands(                    
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                new InstantCommand(() -> m_swerveDriveSubsystem.setInitialPose()),
                
                new InstantCommand(m_pivotSubsystem::downPosition, m_pivotSubsystem),
                new WaitUntilCommand(() -> (m_pivotSubsystem.getMovementState() == 0)),
                // score
                new InstantCommand(() -> m_intakeSubsystem.runIntakeReversed(1), m_intakeSubsystem),
                new WaitCommand(0.2),
                new InstantCommand(m_intakeSubsystem::stopIntake, m_intakeSubsystem),
                // drive to midpoint + rotate + lower arm
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0]+startingX, AutoConstants.RED_RIGHT_MID_POINT[1]+startingY, new Rotation2d(Math.PI/2+startingRot)), false, true, false),
                
                // drive to cone + parallel with intake
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_GAME_PIECE[0]+startingX, AutoConstants.RED_RIGHT_GAME_PIECE[1]+startingY, new Rotation2d(Math.PI+startingRot)), true),
                    new PivotCommand(m_pivotSubsystem, "down")
                ),

                new ParallelRaceGroup(
                    new IntakeCommand(m_intakeSubsystem),
                    new DeadReckoning(swerveDriveSubsystem, -1, 0, 1.25)
                    // new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_GAME_PIECE[0]-1.00, AutoConstants.RED_RIGHT_GAME_PIECE[1], new Rotation2d(Math.PI)), false)
                ),

                // mid point + rotate + raising arm to scoring pos + extending
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0]+startingX, AutoConstants.RED_RIGHT_MID_POINT[1]+startingY, new Rotation2d(0+startingRot)), false, false, false),
                    new InstantCommand(m_pivotSubsystem::up)
                ),
                
                // after coming back
                new InstantCommand(m_telescopingArmSubsystem::in),
                // drive into scoring in 2 commands
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_COMMUNITY_RIGHT_SCORING[0]+startingX, AutoConstants.RED_COMMUNITY_RIGHT_SCORING[1]+startingY, new Rotation2d(0+startingRot)), false, true, false),
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_LEFT_SCORING[0]+startingX, AutoConstants.RED_RIGHT_LEFT_SCORING[1]+startingY, new Rotation2d(0+startingRot)), true),
                // scoring commands
                new InstantCommand(m_pivotSubsystem::storedPosition),
                new InstantCommand(() -> m_telescopingArmSubsystem.scoreCone(1)),
                new DeadReckoning(m_swerveDriveSubsystem, 1.5, 0, 0.5),
                new WaitUntilCommand(() -> m_telescopingArmSubsystem.getMovementState() == 0),
                new ScoreCone(m_pivotSubsystem, m_telescopingArmSubsystem, m_intakeSubsystem),
                new InstantCommand(m_pivotSubsystem::up),
                new InstantCommand(m_telescopingArmSubsystem::in),

                // drive to gamepieces
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_COMMUNITY_RIGHT_SCORING[0]+startingX, AutoConstants.RED_COMMUNITY_RIGHT_SCORING[1]+startingY, new Rotation2d(0+startingRot)), false, true, false),
                
                // midpoint
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0]+startingX, AutoConstants.RED_RIGHT_MID_POINT[1]+startingY, new Rotation2d(0+startingRot)), false, false, false),

                // drive to new gamepiece
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_GAME_PIECE[0]+startingX, AutoConstants.RED_RIGHT_GAME_PIECE[1]+startingY, new Rotation2d(Math.PI+startingRot)), true),
                    new PivotCommand(m_pivotSubsystem, "down")
                )

        );

    }

}