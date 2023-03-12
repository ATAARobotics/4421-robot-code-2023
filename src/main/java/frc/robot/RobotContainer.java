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
    private final LightingSubsystem m_lightingSubsystem;
    // Auto Stuff
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private double swerveSpeed = 0.4421;

    public RobotContainer() {
        // Hardware-based objects
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        pigeon = new Pigeon();

        m_swerveDriveSubsystem = new SwerveDriveSubsystem(pigeon, initialPosition, "canivore");
        //new AprilTagLimelight(m_swerveDriveSubsystem.getOdometry());
       
        
        m_pivotSubsystem = new PivotSubsystem();
        m_telescopingSubsystem = new TelescopingArmSubsystem();
        m_lightingSubsystem = new LightingSubsystem();
        m_intakeSubsystem = new IntakeSubsystem(m_lightingSubsystem);

        m_swerveDriveSubsystem.setBrakes(true);

        m_swerveDriveSubsystem.setDefaultCommand(
                new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                        joysticks::getYVelocity,
                        joysticks::getRotationVelocity, this::getSwerveSpeed,
                        () -> 1));
        // autoChooser
        
        // Red Autos
        autoChooser.setDefaultOption("RedLeaderOverBack", new RedLeaderOverBack(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        autoChooser.addOption("RedLeaderBalance", new RedLeaderBalance(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        // autoChooser.addOption("RedLeftDeadReckoning", new RedLeftDeadReckoning(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        // autoChooser.addOption("RedRightReckoning", new RedRightDeadReckoning(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        
        // Red + Odometry Autos
        // autoChooser.addOption("RedLeftStack", new RedLeftStack(m_swerveDriveSubsystem, m_intakeSubsystem));
        // autoChooser.addOption("RedRightStack", new RedRightStack(m_swerveDriveSubsystem, m_intakeSubsystem));
        // autoChooser.addOption("RedLeaderWGP", new RedLeaderWGP(m_swerveDriveSubsystem, m_intakeSubsystem));
        
        // Blue Autos
        autoChooser.addOption("Teammate", new Teammate(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        // autoChooser.addOption("BlueLeftDeadReckoning", new BlueLeftDeadReckoning(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        // autoChooser.addOption("BlueRightDeadReckoning", new BlueRightDeadReckoning(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));

        // Testing Autos
        // autoChooser.addOption("Square", new Square(m_swerveDriveSubsystem));
        // autoChooser.addOption("Test", new Test(m_swerveDriveSubsystem));

        // Do Nothing Auto
        autoChooser.addOption("Do Nothing", null);

        SmartDashboard.putData("Auto Chooser", autoChooser);

        LiveWindow.disableAllTelemetry();

        configureBindings();
    }


    private void configureBindings() {
        joysticks.IntakeIn.or(new Trigger(() -> joysticks.RotIntake.getAsBoolean())).whileTrue(new RunCommand(() -> m_intakeSubsystem.runIntake(joysticks.getOuttake() - joysticks.getOuttakeInversed())))
        .onFalse(new InstantCommand(() -> {
            m_intakeSubsystem.stopIntake();
            m_intakeSubsystem.resetRunStarted();
        }));

        joysticks.IntakeOut.whileTrue(new RunCommand(() -> m_intakeSubsystem.runIntakeReversed(joysticks.getOuttake() - joysticks.getOuttakeInversed())))
        .onFalse(new InstantCommand(m_intakeSubsystem::stopIntake));
       
        joysticks.PivotUp.whileTrue(new StartEndCommand(m_pivotSubsystem::up, m_pivotSubsystem::stop, m_pivotSubsystem));
        joysticks.DownToStop.whileTrue(new StartEndCommand(m_pivotSubsystem::storedPosition, m_pivotSubsystem::stop, m_pivotSubsystem));
        joysticks.PivotDown.whileTrue(new StartEndCommand(m_pivotSubsystem::down, m_pivotSubsystem::stop, m_pivotSubsystem));
        joysticks.OverridePivotUp.whileTrue(new RunCommand(m_pivotSubsystem::forceup, m_pivotSubsystem))
        .onFalse(new InstantCommand(m_pivotSubsystem::overridestop, m_pivotSubsystem));

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
                                        joysticks::getRotationVelocity, this::getSwerveSpeed,
                                        () -> 1));
        joysticks.SlideRight.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> -0.1,
                        () -> 0,
                        () -> 0, () -> 1,
                        () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                        joysticks::getYVelocity,
                                        joysticks::getRotationVelocity, this::getSwerveSpeed,
                                        () -> 1));
        joysticks.RotateLeft.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0,
                        () -> 0.1,
                        () -> 0, () -> 1,
                        () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                        joysticks::getYVelocity,
                                        joysticks::getRotationVelocity, this::getSwerveSpeed,
                                        () -> 1));
        joysticks.RotateRight.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0,
                        () -> -0.1,
                        () -> 0, () -> 1,
                        () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                        joysticks::getYVelocity,
                                        joysticks::getRotationVelocity, this::getSwerveSpeed,
                                        () -> 1));   
        joysticks.AutoBalance.whileTrue(
                new AutoBalance(m_swerveDriveSubsystem, true)
        );
        joysticks.Forward.onTrue(new InstantCommand(() -> swerveSpeed=1))
        .onFalse(new InstantCommand(() -> swerveSpeed=0.4421));

        joysticks.LightSwitch.onTrue(new InstantCommand(m_lightingSubsystem::FlipLights));

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

    public PivotSubsystem getPivotSubsystem(){
        return m_pivotSubsystem;
    }

    public TelescopingArmSubsystem getTelescopingArmSubsystem(){
        return m_telescopingSubsystem;
    }

    public double getSwerveSpeed(){
        return swerveSpeed;
    }
}
