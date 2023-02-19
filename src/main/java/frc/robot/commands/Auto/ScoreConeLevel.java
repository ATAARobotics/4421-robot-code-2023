package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.AutoCommand;
import frc.robot.AutoPaths;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.subsystems.*;

public class ScoreConeLevel extends SequentialCommandGroup {
    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final PivotSubsystem m_PivotSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final TelescopingArmSubsystem m_TelescopingArmSubsystem;


    public ScoreConeLevel(SwerveDriveSubsystem swerveDriveSubsystem, PivotSubsystem pivotSubsystem, ArmSubsystem armSubsystem, TelescopingArmSubsystem telescopingArmSubsystem, AutoPaths autoPaths) {
        addRequirements(swerveDriveSubsystem, pivotSubsystem, armSubsystem, telescopingArmSubsystem);
        m_swerveDriveSubsystem = swerveDriveSubsystem;
        m_PivotSubsystem = pivotSubsystem;
        m_ArmSubsystem = armSubsystem;
        m_TelescopingArmSubsystem = telescopingArmSubsystem;
        addCommands(
                new AutoDriveCommand(swerveDriveSubsystem, autoPaths.getStraight(), true )
        );

    }

}