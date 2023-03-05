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

  // constants
  private static final double RP = 2.0;

  // speed variables
  private double rotTemp;
  private double rotSpeed;
  private double rotLimit;
  private double angle;
  private final PIDController rotController = new PIDController(RP, 0, 0);

  private final SwerveDriveSubsystem swerveDrive;

  // takes in targetPose and the tolerance it is allowed. rotTolerance(degrees)
  public RotateCommand(SwerveDriveSubsystem swerveDrive, double angle, double tolerance) {
    this.swerveDrive = swerveDrive;
    this.angle = angle;
    // stop when values are small

    rotController.setTolerance(Units.degreesToRadians(tolerance));
    rotController.enableContinuousInput(-Math.PI, Math.PI);

    // // PID VALUES
    SmartDashboard.putNumber("R-P", RP);
    SmartDashboard.putNumber("R-I", 0.0);
    SmartDashboard.putNumber("R-D", 0.0);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {
    // double newrp = SmartDashboard.getNumber("R-P", 2.0);
    // double newri = SmartDashboard.getNumber("R-I", 0.0);
    // double newrd = SmartDashboard.getNumber("R-D", 0.0);
    rotController.reset();
  }

  @Override
  public void execute() {

      // Transform the tag's pose to set our goal
      
      rotController.setSetpoint(angle);

      rotTemp = swerveDrive.getHeading();

      rotSpeed = MathUtil.clamp(-rotController.calculate(rotTemp), -rotLimit, rotLimit);
      if (rotController.atSetpoint()) {
        rotSpeed = 0;
      }
      SmartDashboard.putNumber("rotSpeed", rotSpeed);

      // Drive // yspeed = xspeed  // x speed = y speed. setSwerveDrive is wrong
      swerveDrive.setSwerveDrive(0, 0, rotSpeed, false);

  }

  @Override
    public boolean isFinished() {
        return (rotController.atSetpoint());
    }
    @Override
    public void end(boolean interrupted) {
        swerveDrive.setSwerveDrive(0, 0, 0, false);
    }
}
