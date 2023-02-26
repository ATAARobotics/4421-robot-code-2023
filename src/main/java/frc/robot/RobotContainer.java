package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.*;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class RobotContainer {

    // The initial position of the robot relative to the field. This is measured
    // from the left-hand corner of the field closest to the driver, from the
    // driver's perspective

    public Translation2d initialPosition = new Translation2d(0, 0);

    // Create hardware objects
    private Pigeon pigeon;
    private final OI joysticks = new OI();

    private final SwerveDriveSubsystem m_swerveDriveSubsystem;
    private final PivotSubsystem m_pivotSubsystem;
    private final IntakeSubsystem m_intakeSubsystem;
    private final TelescopingArmSubsystem m_telescopingSubsystem;
    // Auto Stuff
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();



    public RobotContainer() {
        // Hardware-based objects
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        pigeon = new Pigeon();

        m_swerveDriveSubsystem = new SwerveDriveSubsystem(pigeon, initialPosition, "canivore");
        new AprilTagLimelight(m_swerveDriveSubsystem.getOdometry());
       
        m_intakeSubsystem = new IntakeSubsystem();
        m_pivotSubsystem = new PivotSubsystem();
        m_telescopingSubsystem = new TelescopingArmSubsystem();

        m_swerveDriveSubsystem.setBrakes(true);

        m_swerveDriveSubsystem.setDefaultCommand(
                new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                        joysticks::getYVelocity,
                        joysticks::getRotationVelocity, () -> 1,
                        () -> 1));
        // autoChooser
        autoChooser.setDefaultOption("RedRightStack", new RedRightStack(m_swerveDriveSubsystem));
        autoChooser.addOption("Red Leader", new RedLeader(m_swerveDriveSubsystem));
        autoChooser.addOption("RedLeftStack", new RedLeftStack(m_swerveDriveSubsystem));
        autoChooser.addOption("Square", new Square(m_swerveDriveSubsystem));
        autoChooser.addOption("Test", new Test(m_swerveDriveSubsystem));
        autoChooser.addOption("Do Nothing", null);

        SmartDashboard.putData("Auto Chooser", autoChooser);

        LiveWindow.disableAllTelemetry();

        configureBindings();
    }


    private void configureBindings() {
        joysticks.IntakeIn.onTrue(new InstantCommand(m_intakeSubsystem::timerReset))
        .whileTrue(new RunCommand(m_intakeSubsystem::runIntake))
        .onFalse(new InstantCommand(m_intakeSubsystem::stopIntake));

        joysticks.IntakeOut.whileTrue(new RunCommand(m_intakeSubsystem::runIntakeReversed))
        .onFalse(new InstantCommand(m_intakeSubsystem::stopIntake));
       
        joysticks.PivotUp.onTrue(new InstantCommand(m_pivotSubsystem::up, m_pivotSubsystem));
        joysticks.OverridePivotUp.onTrue(new RunCommand(m_pivotSubsystem::overrideUp, m_pivotSubsystem));

        joysticks.PivotDown.onTrue(new InstantCommand(m_pivotSubsystem::down, m_pivotSubsystem));


        joysticks.TelescopingOut.whileTrue(new RunCommand(m_telescopingSubsystem::out, m_telescopingSubsystem))
        .onFalse(new RunCommand(m_telescopingSubsystem::stop, m_telescopingSubsystem));
        joysticks.TelescopingIn.whileTrue(new RunCommand(m_telescopingSubsystem::in, m_telescopingSubsystem))
        .onFalse(new RunCommand(m_telescopingSubsystem::stop, m_telescopingSubsystem));


        // joysticks.TelescopingIn.whileTrue(new RunCommand(m_telescopingSubsystem::in, m_telescopingSubsystem))
        // .onFalse(new InstantCommand(m_telescopingSubsystem::stop));

        joysticks.SlideLeft.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0.1,
                        () -> 0,
                        () -> 0, () -> 1,
                        () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                        joysticks::getYVelocity,
                                        joysticks::getRotationVelocity, () -> 1,
                                        () -> 1));
        joysticks.SlideRight.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> -0.1,
                        () -> 0,
                        () -> 0, () -> 1,
                        () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                        joysticks::getYVelocity,
                                        joysticks::getRotationVelocity, () -> 1,
                                        () -> 1));
        joysticks.RotateLeft.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0,
                        () -> 0,
                        () -> 0.1, () -> 1,
                        () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                        joysticks::getYVelocity,
                                        joysticks::getRotationVelocity, () -> 1,
                                        () -> 1));
        joysticks.RotateRight.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0,
                        () -> 0,
                        joysticks::getRotationVelocity, () -> 1,
                        () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                        joysticks::getYVelocity,
                                        joysticks::getRotationVelocity, () -> 1,
                                        () -> 1));   
        joysticks.AutoBalance.whileTrue(
                new AutoBalance(m_swerveDriveSubsystem, true)
        );
    }

    public OI getOI() {
        return joysticks;
    }

    public SwerveDriveSubsystem getSwerveDriveSubsystem() {
        return m_swerveDriveSubsystem;
    }

   

    public SendableChooser<Command> getAutonomousChooser() {
        return autoChooser;
    }
}
