package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveOdometry;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class RotateCommand extends CommandBase {

  // speed variables
  private double rotTemp;
  private double rotSpeed;
  private double rotLimit;
  private double angle;
  private Pose2d robotPose;
  private final PIDController rotController = new PIDController(3.0, 0.1, 0);

  private final SwerveDriveSubsystem m_swerveDriveSubsystem;

  // takes in targetPose and the tolerance it is allowed. rotTolerance(degrees)
  public RotateCommand(SwerveDriveSubsystem swerveDrive, double angle, double tolerance) {
    this.m_swerveDriveSubsystem = swerveDrive;
    this.angle = angle;
    // stop when values are small

    rotController.setTolerance(Units.degreesToRadians(tolerance));
    rotController.enableContinuousInput(-Math.PI, Math.PI);
    rotController.setSetpoint(angle);
    addRequirements(swerveDrive);
  }

  @Override
  public void execute() {

      // Transform the tag's pose to set our goal
      
      robotPose = m_swerveDriveSubsystem.getOdometry().getPoseMeters();

      rotSpeed = MathUtil.clamp(rotController.calculate(robotPose.getRotation().getRadians()), -rotLimit, rotLimit);
      if (rotController.atSetpoint()) {
        rotSpeed = 0;
      }
      // SmartDashboard.putNumber("rotSpeed", rotSpeed);

      // Drive // yspeed = xspeed  // x speed = y speed. setSwerveDrive is wrong
      m_swerveDriveSubsystem.setSwerveDrive(0, 0, rotSpeed, false);

  }

  @Override
    public boolean isFinished() {
        return (rotController.atSetpoint());
    }
    @Override
    public void end(boolean interrupted) {
        m_swerveDriveSubsystem.setSwerveDrive(0, 0, 0, false);
    }
}
