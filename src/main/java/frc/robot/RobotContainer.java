package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.auto.Red.Right.*;
import frc.robot.commands.auto.Red.Left.*;
import frc.robot.commands.auto.Red.Center.*;
import frc.robot.commands.auto.Blue.Right.*;
import frc.robot.commands.auto.Blue.Left.*;
import frc.robot.commands.auto.Blue.Center.*;
import frc.robot.commands.auto.Irrelevant.*;
import frc.robot.commands.auto.testers.*;
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

    private double swerveSpeed = Constants.SLOW_MAXIMUM_SPEED;
    private double swerveSpeedRot = Constants.SLOW_MAXIMUM_ROTATIONAL_SPEED;


    public RobotContainer(Alliance alliance) {
        // Hardware-based objects
        // NetworkTableInstance inst = NetworkTableInstance.getDefault();
        pigeon = new Pigeon();
        
        m_swerveDriveSubsystem = new SwerveDriveSubsystem(pigeon, initialPosition, "canivore", alliance);
        // new AprilTagLimelight(m_swerveDriveSubsystem.getOdometry(), m_swerveDriveSubsystem);
       
        m_pivotSubsystem = new PivotSubsystem();
        m_telescopingSubsystem = new TelescopingArmSubsystem();
        m_lightingSubsystem = new LightingSubsystem();
        m_intakeSubsystem = new IntakeSubsystem(m_lightingSubsystem);

        m_swerveDriveSubsystem.setBrakes(true);

        m_swerveDriveSubsystem.setDefaultCommand(
                new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
                        joysticks::getYVelocity,
                        joysticks::getHeading, joysticks::getRotationVelocity, this::getSwerveSpeed,
                        this::getSwerveSpeedRot));
        // autoChooser
        
        // Red Autos
        // Do Nothing Auto
        autoChooser.setDefaultOption("Do Nothing", new DoNothing(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        
        if(alliance == Alliance.Red){
            //Red
            autoChooser.addOption("RedRight_ConeMid_CubeHigh_Charge", new RedRight_ConeMid_CubeHigh_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedRight_ConeMid_CubeMid_Charge", new RedRight_ConeMid_CubeMid_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedRight_ConeMid_CubeHigh_ConeGrab", new RedRight_ConeMid_CubeHigh_ConeGrab(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedRight_ConeMid_CubeMid_ConeGrab", new RedRight_ConeMid_CubeMid_ConeGrab(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedRight_ConeMid_CubeHigh", new RedRight_ConeMid_CubeHigh(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedRight_ConeMid_CubeMid", new RedRight_ConeMid_CubeMid(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));

            autoChooser.addOption("RedLeft_ConeMid_CubeHigh", new RedLeft_ConeMid_CubeHigh(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedLeft_ConeMid_CubeMid", new RedLeft_ConeMid_CubeMid(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));

            autoChooser.addOption("RedCenter_CubeHigh_Over_Charge", new RedCenter_CubeHigh_Over_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedCenter_CubeMid_Over_Charge", new RedCenter_CubeMid_Over_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedCenter_CubeHigh_Charge", new RedCenter_CubeHigh_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("RedCenter_CubeMid_Charge", new RedCenter_CubeMid_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        }
        if(alliance == Alliance.Blue){
            //Blue
            autoChooser.addOption("BlueLeft_ConeMid_CubeHigh_Charge", new BlueLeft_ConeMid_CubeHigh_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueLeft_ConeMid_CubeMid_Charge", new BlueLeft_ConeMid_CubeMid_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueLeft_ConeMid_CubeHigh_ConeGrab", new BlueLeft_ConeMid_CubeHigh_ConeGrab(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueLeft_ConeMid_CubeMid_ConeGrab", new BlueLeft_ConeMid_CubeMid_ConeGrab(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueLeft_ConeMid_CubeHigh", new BlueLeft_ConeMid_CubeHigh(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueLeft_ConeMid_CubeMid", new BlueLeft_ConeMid_CubeMid(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            
            autoChooser.addOption("BlueRight_ConeMid_CubeHigh", new BlueRight_ConeMid_CubeHigh(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueRight_ConeMid_CubeMid", new BlueRight_ConeMid_CubeMid(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));

            autoChooser.addOption("BlueCenter_CubeHigh_Over_Charge", new BlueCenter_CubeHigh_Over_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueCenter_CubeMid_Over_Charge", new BlueCenter_CubeMid_Over_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueCenter_CubeHigh_Charge", new BlueCenter_CubeHigh_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
            autoChooser.addOption("BlueCenter_CubeMid_Charge", new BlueCenter_CubeMid_Charge(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));   
        }

        //Irrelevant
        autoChooser.addOption("Irrelevant_CubeHigh", new Irrelevant_CubeHigh(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        autoChooser.addOption("Irrelevant_CubeMid", new Irrelevant_CubeMid(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        autoChooser.addOption("Irrelevant_ConeMid", new Irrelevant_ConeMid(m_swerveDriveSubsystem, m_intakeSubsystem, m_telescopingSubsystem, m_pivotSubsystem));
        

        // Testing Autos
        //autoChooser.addOption("Square", new Square(m_swerveDriveSubsystem));
        //autoChooser.addOption("Test", new Test(m_swerveDriveSubsystem, m_intakeSubsystem));

        // autoChooser.addOption("SquareWithRot", new SquareWithRot(m_swerveDriveSubsystem));
        // autoChooser.addOption("SquareWithOtherRot", new SquareWithOtherRot(m_swerveDriveSubsystem));


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
        
        joysticks.DownToStop.and(() -> joysticks.getOuttakeInversed() <= 0.5).and(() -> joysticks.getOuttake() <= 0.5).whileTrue(new StartEndCommand(m_pivotSubsystem::downPosition, m_pivotSubsystem::stop, m_pivotSubsystem))
        .whileTrue(new StartEndCommand(m_telescopingSubsystem::in, m_telescopingSubsystem::stop, m_telescopingSubsystem));
        

        joysticks.DownToStop.and(() -> joysticks.getOuttakeInversed() <= 0.5).and(() -> joysticks.getOuttake() >= 0.5).whileTrue(new StartEndCommand(m_pivotSubsystem::storedPosition, m_pivotSubsystem::stop, m_pivotSubsystem))
        .whileTrue(new StartEndCommand(() -> m_telescopingSubsystem.scoreCone(joysticks.getOuttake()), m_telescopingSubsystem::stop, m_telescopingSubsystem));
        
        joysticks.DownToStop.and(() -> joysticks.getOuttakeInversed() >= 0.5).and(() -> joysticks.getOuttake() <= 0.5).onTrue(new ScoreCone(m_pivotSubsystem, m_telescopingSubsystem, m_intakeSubsystem))
        .onFalse(new InstantCommand(m_pivotSubsystem::stop, m_pivotSubsystem)).onFalse(new InstantCommand(m_intakeSubsystem::stopIntake, m_intakeSubsystem)).onFalse(new InstantCommand(m_telescopingSubsystem::stop, m_telescopingSubsystem));
        
        joysticks.DownToStop.and(() -> joysticks.getOuttakeInversed() >= 0.5).and(() -> joysticks.getOuttake() >= 0.5).onTrue(new ScoreCube(m_pivotSubsystem, m_telescopingSubsystem, m_intakeSubsystem))
        .onFalse(new InstantCommand(m_pivotSubsystem::stop, m_pivotSubsystem)).onFalse(new InstantCommand(m_intakeSubsystem::stopIntake, m_intakeSubsystem)).onFalse(new InstantCommand(m_telescopingSubsystem::stop, m_telescopingSubsystem));
        
        joysticks.PivotDown.whileTrue(new StartEndCommand(m_pivotSubsystem::down, m_pivotSubsystem::stop, m_pivotSubsystem));
        joysticks.OverridePivotUp.whileTrue(new RunCommand(m_pivotSubsystem::forceup, m_pivotSubsystem))
        .onFalse(new InstantCommand(m_pivotSubsystem::overridestop, m_pivotSubsystem));

        joysticks.TelescopingOut.whileTrue(new RunCommand(m_telescopingSubsystem::out, m_telescopingSubsystem))
        .onFalse(new RunCommand(m_telescopingSubsystem::stop, m_telescopingSubsystem));
        joysticks.TelescopingIn.whileTrue(new RunCommand(m_telescopingSubsystem::in, m_telescopingSubsystem))
        .onFalse(new RunCommand(m_telescopingSubsystem::stop, m_telescopingSubsystem));

        joysticks.ResetOdo.onTrue(new InstantCommand(m_swerveDriveSubsystem::resetPosition));
        // joysticks.TelescopingIn.whileTrue(new RunCommand(m_telescopingSubsystem::in, m_telescopingSubsystem))
        // .onFalse(new InstantCommand(m_telescopingSubsystem::stop));

        // joysticks.SlideLeft.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0.1,
        //                 () -> 0,
        //                 () -> 0, () -> 1,
        //                 () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                                 joysticks::getYVelocity,
        //                                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                                 () -> 1));
        // joysticks.SlideRight.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> -0.1,
        //                 () -> 0,
        //                 () -> 0, () -> 1,
        //                 () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                                 joysticks::getYVelocity,
        //                                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                                 () -> 1));
        // joysticks.RotateLeft.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 1,
        //                 () -> 0,
        //                 () -> 0, () -> 1,
        //                 () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                 joysticks::getYVelocity,
        //                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                 this::getSwerveSpeedRot));

        // joysticks.RotateRight.onTrue(new DriveCommand(m_swerveDriveSubsystem, () -> 0,
        //                 () -> -0.1,
        //                 () -> 0, () -> 1,
        //                 () -> 1)).onFalse(new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                                 joysticks::getYVelocity,
        //                                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                                 () -> 1));   
        // joysticks.AutoBalance.onTrue(
        //         new AutoBalance(m_swerveDriveSubsystem, true)
        // ).onFalse(
        //     new DriveCommand(m_swerveDriveSubsystem, joysticks::getXVelocity,
        //                 joysticks::getYVelocity,
        //                 joysticks::getRotationVelocity, this::getSwerveSpeed,
        //                 () -> 1)
        // );
        joysticks.Forward.onTrue(new InstantCommand(() -> {swerveSpeed=Constants.MAXIMUM_SPEED; swerveSpeedRot=Constants.MAXIMUM_ROTATIONAL_SPEED;}))
        .onFalse(new InstantCommand(() -> {swerveSpeed=Constants.SLOW_MAXIMUM_SPEED; swerveSpeedRot=Constants.SLOW_MAXIMUM_ROTATIONAL_SPEED;}));

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
    public double getSwerveSpeedRot(){
        return swerveSpeedRot;
    }
}
