package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoBalance extends CommandBase {
    private double error;
    private double currentAngle;
    private double drivePower;
    private SwerveDriveSubsystem swerveSubsystem;

    public AutoBalance(SwerveDriveSubsystem swerve) {

        this.swerveSubsystem = swerve;
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment the line below this to simulate the gyroscope axis with a
        // controller joystick
        // Double currentAngle = -1 *
        // Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
        this.currentAngle = swerveSubsystem.getPigeon().getPitch();

        error = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAngle;
        drivePower = -Math.min(Constants.BEAM_BALANACED_DRIVE_KP * error, 1);

        // Our robot needed an extra push to drive up in reverse, probably due to weight
        // imbalances

        // Limit the max power
        if (Math.abs(drivePower) > 0.5) {
            drivePower = Math.copySign(0.5, drivePower);
        }

        swerveSubsystem.setSwerveDrive(0, drivePower, 0, false);

        // Debugging Print Statments
        SmartDashboard.putNumber("Current Angle", currentAngle);
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("Drive Power", drivePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.setSwerveDrive(0, 0, 0, false);
    }

    // Returns true when the command should end.
    //@Override
    //public boolean isFinished() {
        //return Math.abs(error) < Constants.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES;
    //}

}