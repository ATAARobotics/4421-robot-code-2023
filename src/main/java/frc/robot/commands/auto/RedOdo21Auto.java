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

public class RedOdo21Auto extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final TelescopingArmSubsystem m_telescopingArmSubsystem;
    private final PivotSubsystem m_pivotSubsystem;

    

    public RedOdo21Auto(SwerveDriveSubsystem swerveDriveSubsystem, IntakeSubsystem intakeSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, PivotSubsystem pivotSubsystem) {
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_intakeSubsystem = intakeSubsystem;
        m_telescopingArmSubsystem = telescopingArmSubsystem;
        m_pivotSubsystem = pivotSubsystem;

        addRequirements(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingArmSubsystem, m_pivotSubsystem);

        addCommands(                    
                new InstantCommand(() -> m_swerveDriveSubsystem.setFieldOriented(true, 0)),
                new InstantCommand(() -> m_swerveDriveSubsystem.setInitialPose(new Pose2d(AutoConstants.Red_MID_MID_SCORING[0], AutoConstants.Red_MID_MID_SCORING[1], new Rotation2d(0)))),
                
                new PivotCommand(m_pivotSubsystem, "firstdown"),
                new TelescopingArmCommand(m_telescopingArmSubsystem, "cube"),
                new WaitCommand(0.25),
                // score
                new OuttakeCommand(m_intakeSubsystem),

                // Drive over charging station with odometry
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_MID_OVER_CHARGING_STATION[0], AutoConstants.RED_MID_OVER_CHARGING_STATION[1], new Rotation2d(0)), false),
                new AutoDriveToWayPoint(m_swerveDriveSubsystem, new Pose2d(AutoConstants.RED_MID_OVER_CHARGING_STATION[0], AutoConstants.RED_MID_OVER_CHARGING_STATION[1], new Rotation2d(0)), false),

                // Auto-Balance on charging station
                new WaitCommand(0.25),
                new DeadReckoning(m_swerveDriveSubsystem, 1.0, 0.0, 2),
                new AutoBalance(m_swerveDriveSubsystem, true)

        );

    }

}