package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveOdometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoDriveToWayPoint extends CommandBase {

    private SwerveDriveSubsystem m_swerveDriveSubsystem;
    private SwerveOdometry odometry;

    // Poses
    private Pose2d targetPose;
    private Pose2d robotPose;
    private Pose2d goalPose;

    // speed variables
    private double xSpeed;
    private double ySpeed;
    private double rotTemp;
    private double rotSpeed;
    private double speedLimit;
    private double rotLimit;

    private boolean isEndPoint;

    // PID
    private final PIDController xController = new PIDController(3.0, 0, 0);
    private final PIDController yController = new PIDController(3.0, 0, 0);
    private final PIDController rotController = new PIDController(3.0, 0.1, 0);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV);

    private boolean Y_ACH = false;
    private boolean X_ACH = false;
    private boolean ROT_ACH = false;

    public AutoDriveToWayPoint(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose, double driveTolerance, double rotTolerance, double speedLimit, double rotLimit, boolean isEndPoint) {
        this.m_swerveDriveSubsystem = swerveDriveSubsystem;
        this.targetPose = targetPose;
        this.odometry = swerveDriveSubsystem.getOdometry();
        rotController.enableContinuousInput(-Math.PI, Math.PI);
        this.speedLimit = speedLimit;
        this.rotLimit = rotLimit;
        this.isEndPoint = isEndPoint;
        addRequirements(this.m_swerveDriveSubsystem);
    }

    public AutoDriveToWayPoint(SwerveDriveSubsystem swerveDriveSubsystem, Pose2d targetPose, boolean isEndPoint) {
      this(swerveDriveSubsystem, targetPose, Constants.DTOLERANCE, Constants.RTOLERANCE, Constants.SPEEDLIMIT, Constants.ROTLIMIT, isEndPoint);
    }

    @Override
    public void initialize() {
        m_swerveDriveSubsystem.setBrakes(true);
        goalPose = targetPose;

        if (isEndPoint) {
          xController.setTolerance(Constants.E_DTOLERANCE);
          yController.setTolerance(Constants.E_DTOLERANCE);
          rotController.setTolerance(Units.degreesToRadians(Constants.E_RTOLERANCE));
        } else {
          xController.setTolerance(Constants.DTOLERANCE);
          yController.setTolerance(Constants.DTOLERANCE);
          rotController.setTolerance(Units.degreesToRadians(Constants.RTOLERANCE));
        }

        xController.setSetpoint(goalPose.getX());
        yController.setSetpoint(goalPose.getY());
        rotController.setSetpoint(goalPose.getRotation().getRadians());
    }

    @Override
    public void execute() {

      robotPose = odometry.getPoseMeters();

      SmartDashboard.putNumber("X-Goal", goalPose.getX());
      SmartDashboard.putNumber("Y-Goal", goalPose.getY());
      SmartDashboard.putNumber("Rot-Goal", goalPose.getRotation().getRadians());
      
      SmartDashboard.putNumber("robotPoseX", robotPose.getX());
      SmartDashboard.putNumber("robotPoseY", robotPose.getY());
      SmartDashboard.putNumber("robotPoseR", robotPose.getRotation().getRadians());


      SmartDashboard.putBoolean("X-ACH", false);
      xSpeed = -MathUtil.clamp(xController.calculate(robotPose.getX()), -speedLimit, speedLimit);
      if (xController.atSetpoint()) {
        SmartDashboard.putBoolean("X-ACH", true);
        xSpeed = 0;
      }

      SmartDashboard.putBoolean("Y-ACH", false);
      ySpeed = -MathUtil.clamp(yController.calculate(robotPose.getY()), -speedLimit, speedLimit);
      if (yController.atSetpoint()) {
        SmartDashboard.putBoolean("Y-ACH", true);
        ySpeed = 0;
      }

      // rotTemp = m_swerveDriveSubsystem.getHeading();
      // if(robotPose.getRotation().getRadians() > 0) {
      //   rotTemp = rotTemp - Math.PI;
      // }
      // else {
      //   rotTemp = rotTemp + Math.PI;
      // }

      SmartDashboard.putBoolean("ROT-ACH", false);
      rotSpeed = MathUtil.clamp(rotController.calculate(robotPose.getRotation().getRadians()), -rotLimit, rotLimit);
      if (rotController.atSetpoint()) {
        SmartDashboard.putBoolean("ROT-ACH", true);
        rotSpeed = 0;
      }

      SmartDashboard.putNumber("xSpeed", xSpeed);
      SmartDashboard.putNumber("ySpeed", ySpeed);
      SmartDashboard.putNumber("rotSpeed", rotSpeed);

      // Drive // x and y is flipped
      m_swerveDriveSubsystem.setSwerveDrive(xSpeed + feedforward.calculate(odometry.getXVel()), ySpeed + feedforward.calculate(odometry.getYVel()), rotSpeed, true);
    }

    @Override
    public boolean isFinished() {
        return (xController.atSetpoint() && yController.atSetpoint() && rotController.atSetpoint());
    }
}
