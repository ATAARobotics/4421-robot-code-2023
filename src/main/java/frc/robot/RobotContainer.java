package frc.robot;

import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.LightingCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.TelescopingArmSubsystem;
import frc.robot.subsystems.AprilTagLimelight;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LightingSubsystem;
import frc.robot.subsystems.MusicSubsystem;
import frc.robot.subsystems.PivotSubsystem;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;

/*
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
*/

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
        private final ArmSubsystem m_armSubsystem;
        private final TelescopingArmSubsystem m_telescopingSubsystem;
        private final LightingSubsystem m_lightingSubsystem;
        private final MusicSubsystem m_musicSubsystem;
        // private final LimelightSubsystem m_limelightSubsystem;
        private final AutoPaths m_autoPaths;

        private boolean visionEnabled = true;
        private boolean visionTargeting = false;

        private double aimRotationSpeed = 0.25 * 0.7;

        

        // Auto Stuff
        private final SendableChooser<Command> autoChooser = new SendableChooser<>();
        public static ProfiledPIDController rotationController = new ProfiledPIDController(0.9, 0, 0.001,
                        new TrapezoidProfile.Constraints(Constants.MAXIMUM_ROTATIONAL_SPEED_AUTO,
                                        Constants.MAXIMUM_ROTATIONAL_ACCELERATION));

        public RobotContainer() {
                // Hardware-based objects
                // NetworkTableInstance inst = NetworkTableInstance.getDefault();
                pigeon = new Pigeon();

                m_swerveDriveSubsystem = new SwerveDriveSubsystem(pigeon, initialPosition, "canivore");
                m_lightingSubsystem = new LightingSubsystem();
                m_musicSubsystem = new MusicSubsystem();
                m_armSubsystem = new ArmSubsystem();
                m_pivotSubsystem = new PivotSubsystem();
                m_telescopingSubsystem = new TelescopingArmSubsystem();
                m_autoPaths = new AutoPaths();
                LightingCommand lightingCommand = new LightingCommand(m_lightingSubsystem, m_armSubsystem, OI.wantACone());
                // m_limelightSubsystem = new LimelightSubsystem();

                // path planner loader // TODO: array list?

                HashMap<String, Command> eventMap = new HashMap<>();

                // Set the magazine to index
                m_swerveDriveSubsystem.setBrakes(false);
                m_musicSubsystem.setDefaultCommand(new RunCommand(m_musicSubsystem::musicPlay));
                m_swerveDriveSubsystem.setDefaultCommand(
                                new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                                joysticks::getYVelocity,
                                                joysticks::getRotationVelocity, () -> 1,
                                                () -> 1));
                // m_shooterSubsystem.setDefaultCommand(new
                // RunCommand(m_shooterSubsystem::shooterHighFar, m_shooterSubsystem));
                /*
                 * autoChooser.setDefaultOption("Straight",
                 * new Straight(m_swerveDriveSubsystem, m_intakeSubsystem,
                 * m_hoodSubsystem, m_magazineSubsystem, m_shooterSubsystem));
                 */
                // autoChooser.addOption("Test Path", testPath);
                SmartDashboard.putData("Auto Chooser", autoChooser);
                LiveWindow.disableAllTelemetry();
                // visionAlignCommand = new VisionAlignCommand(m_limelightSubsystem,
                // m_swerveDriveSubsystem);
                /*
                 * autoClimbCommand = new AutoClimbCommand(m_climbArmSubsystem,
                 * m_climbMotorSubsystem, joysticks.autoClimb,
                 * joysticks.abortAutoClimb);
                 */
                configureBindings();
        }

        public void AutoInit(double rotation) {
                rotationController.enableContinuousInput(-Math.PI, Math.PI);

                rotationController.reset(new TrapezoidProfile.State(rotation, 0.0));

        }

        private void configureBindings() {
                joysticks.PlayMusic.onTrue(new ConditionalCommand(
                        new RunCommand(m_musicSubsystem::musicPause), 
                        new RunCommand(m_musicSubsystem::musicPlay), 
                        m_musicSubsystem.isPlaying()));
                joysticks.IntakeIn.onTrue(new InstantCommand(m_armSubsystem::timerReset))
                .whileTrue(new RunCommand(m_armSubsystem::runIntake))
                .onFalse(new InstantCommand(m_armSubsystem::stopIntake));

                joysticks.IntakeOut.whileTrue(new RunCommand(m_armSubsystem::runIntakeReversed))
                .onFalse(new InstantCommand(m_armSubsystem::stopIntake));
               
                joysticks.PivotUp.whileTrue(new RunCommand(m_pivotSubsystem::up, m_pivotSubsystem))
                .onFalse(new InstantCommand(m_pivotSubsystem::stop));
                joysticks.PivotDown.whileTrue(new RunCommand(m_pivotSubsystem::down, m_pivotSubsystem))
                .onFalse(new InstantCommand(m_pivotSubsystem::stop));

                joysticks.TelescopingOut.whileTrue(new RunCommand(m_telescopingSubsystem::out, m_telescopingSubsystem))
                .onFalse(new RunCommand(m_telescopingSubsystem::stop, m_telescopingSubsystem));
                joysticks.TelescopingIn.whileTrue(new RunCommand(m_telescopingSubsystem::in, m_telescopingSubsystem))
                .onFalse(new RunCommand(m_telescopingSubsystem::stop, m_telescopingSubsystem));

                // joysticks.TelescopingIn.whileTrue(new RunCommand(m_telescopingSubsystem::in, m_telescopingSubsystem))
                // .onFalse(new InstantCommand(m_telescopingSubsystem::stop));

                joysticks.Forward.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0,
                                () -> 1,
                                joysticks::getRotationVelocity, () -> 1,
                                () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                                                joysticks::getYVelocity,
                                                joysticks::getRotationVelocity, () -> 1,
                                                () -> 1));

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

        public ProfiledPIDController getRotationController() {
                return rotationController;
        }
}
