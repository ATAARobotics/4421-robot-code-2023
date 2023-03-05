package frc.robot.commands.auto;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.AutoConstants;
import frc.robot.Constants;
import frc.robot.commands.AutoDriveToWayPoint;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OuttakeCommand;
import frc.robot.commands.PivotCommand;
import frc.robot.commands.TelescopingArmCommand;
import frc.robot.subsystems.*;

/*  Description of this auto
*   1. Score high cube
*   2. Drives to pick up cone and score
*/

// default angle when beginning = Math.PI

public class RedRightStack extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final TelescopingArmSubsystem m_telescopingArmSubsystem;
    private final PivotSubsystem m_pivotSubsystem;

    // for tweaking with tolerances and speed for auto
    double DTOLERANCE = Constants.DTOLERANCE;
    double RTOLERANCE = Constants.RTOLERANCE;
    double SPEEDLIMIT = Constants.SPEEDLIMIT;
    double ROTLIMIT = Constants.ROTLIMIT;

    public RedRightStack(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;

        addRequirements(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingArmSubsystem, m_pivotSubsystem);

        addCommands(
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                
                // move down to scoring pos
                new ParallelCommandGroup(
                   //new TelescopingArmCommand(m_telescopingArmSubsystem, Constants.TELESCOPING_SCORING_POINT_CUBE),
                    new PivotCommand(m_pivotSubsystem, "firstdown")
                ),
                // score
                new OuttakeCommand(m_intakeSubsystem),

                // drive to midpoint + rotate TODO: parallel with lower arm
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0], AutoConstants.RED_RIGHT_MID_POINT[1], new Rotation2d(Math.PI)), false),
                    new PivotCommand(m_pivotSubsystem, "down")
                ),
                
                // drive to cone + parallel with intake
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_GAME_PIECE[0], AutoConstants.RED_RIGHT_GAME_PIECE[1], new Rotation2d(0.0)), true),
                    new IntakeCommand(intakeSubsystem)
                ),

                // mid point + rotate + TODO: parallel with raising arm to scoring pos + extending
                new ParallelCommandGroup(
                    new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_MID_POINT[0], AutoConstants.RED_RIGHT_MID_POINT[1], new Rotation2d(Math.PI)), false),
                    new PivotCommand(m_pivotSubsystem, "up")
                ),
                
                // scoring position
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_RIGHT_RIGHT_SCORING[0], AutoConstants.RED_RIGHT_RIGHT_SCORING[1], new Rotation2d(Math.PI)), true)
                
                // place cone
                // new StartEndCommand(m_intakeSubsystem.runIntakeReversed(), m_intakeSubsystem.stopIntake(), m_intakeSubsystem)
                


        );

    }

}