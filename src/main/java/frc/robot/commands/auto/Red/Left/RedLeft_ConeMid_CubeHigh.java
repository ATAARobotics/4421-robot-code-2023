package frc.robot.commands.auto.Red.Left;

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

public class RedLeft_ConeMid_CubeHigh extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final TelescopingArmSubsystem m_telescopingArmSubsystem;
    private final PivotSubsystem m_pivotSubsystem;

    

    public RedLeft_ConeMid_CubeHigh(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem) {
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
                
                new InstantCommand(m_pivotSubsystem::storedPosition),
                new InstantCommand(() -> m_telescopingArmSubsystem.scoreCone(1)),
                new WaitUntilCommand(() -> m_telescopingArmSubsystem.getMovementState() == 0),
                new ScoreCone(m_pivotSubsystem, m_telescopingArmSubsystem, m_intakeSubsystem),
                new InstantCommand(m_pivotSubsystem::up),
                new InstantCommand(m_telescopingArmSubsystem::in),

                // drive to midpoint + rotate + lower arm
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.A2_RED_LEFT_BUMPCHECK_IN[0]+startingX, AutoConstants.A2_RED_LEFT_BUMPCHECK_IN[1]+startingY, new Rotation2d(Math.PI+startingRot)), false, true, true, 1.5),

                // drive to cone + parallel with intake
                new InstantCommand(m_pivotSubsystem::down),
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.A2_RED_LEFT_BUMPCHECK_OUT[0]+startingX, AutoConstants.A2_RED_LEFT_BUMPCHECK_OUT[1]+startingY, new Rotation2d(Math.PI+startingRot)), false, true, true, 0.75),

                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.A2_RED_LEFT_GAME_PIECE[0]+startingX, AutoConstants.A2_RED_LEFT_GAME_PIECE[1]+startingY, new Rotation2d(Math.PI+startingRot)), false, true, true),
                new ParallelCommandGroup(
                    new DeadReckoning(swerveDriveSubsystem, -1, 0, 0.5),
                    new InstantCommand(() -> m_intakeSubsystem.runIntake(0), m_intakeSubsystem)
                ),
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.A2_RED_LEFT_BUMPCHECK_OUT[0]+startingX, AutoConstants.A2_RED_LEFT_BUMPCHECK_OUT[1]+startingY, new Rotation2d(0+startingRot)), false, true, false),
                    new InstantCommand(m_intakeSubsystem::stopIntake, m_intakeSubsystem),
                    new InstantCommand(m_pivotSubsystem::up)
                ),
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.A2_RED_LEFT_BUMPCHECK_IN[0]+startingX, AutoConstants.A2_RED_LEFT_BUMPCHECK_IN[1]+startingY, new Rotation2d(0+startingRot)), false, true, false, 0.75),
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.A2_RED_LEFT_MID_SCORING[0]+startingX, AutoConstants.A2_RED_LEFT_MID_SCORING[1]+startingY, new Rotation2d(0+startingRot)), false, true, false),
                    new InstantCommand(m_pivotSubsystem::highCube, m_pivotSubsystem)
                ),
                //get into high cube scoring position
                new InstantCommand(m_telescopingArmSubsystem::scoreCube, m_telescopingArmSubsystem),
                new WaitUntilCommand(() -> m_telescopingArmSubsystem.getMovementState() == 0),
                // score
                new InstantCommand(() -> m_intakeSubsystem.runIntakeReversed(1), m_intakeSubsystem)
        );

    }

}