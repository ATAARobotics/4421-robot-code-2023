package frc.robot.commands;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DriveCommand extends CommandBase {
    private SwerveDriveSubsystem swerveSubsystem;
    private DoubleSupplier xSupplier, ySupplier, rotationSupplier, speedSupplier, rotationSpeedSupplier;


    public DriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier speedSupplier, DoubleSupplier rotationSpeedSupplier) {
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.speedSupplier = speedSupplier;
        this.rotationSupplier = rotationSupplier;
        this.rotationSpeedSupplier = rotationSpeedSupplier;
        swerveSubsystem = swerve;

        addRequirements(swerve);
    }

    public DriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier speedSupplier) {
        this(swerve, xSupplier, ySupplier, rotationSupplier, speedSupplier, () -> 0.25);
    }

    public DriveCommand(SwerveDriveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier) {
        this(swerve, xSupplier, ySupplier, rotationSupplier, () -> Constants.SLOW_MAXIMUM_SPEED, () -> 0.25);
    }

    @Override
    public void execute() {
        double xSpeed = MathUtil.clamp(xSupplier.getAsDouble() * speedSupplier.getAsDouble(), -speedSupplier.getAsDouble(), speedSupplier.getAsDouble());
        double ySpeed = MathUtil.clamp(ySupplier.getAsDouble() * speedSupplier.getAsDouble(), -speedSupplier.getAsDouble(), speedSupplier.getAsDouble());
        double rotationSpeed = MathUtil.clamp(rotationSupplier.getAsDouble() * rotationSpeedSupplier.getAsDouble() * Constants.MAXIMUM_ROTATIONAL_SPEED, -Constants.MAXIMUM_ROTATIONAL_SPEED, Constants.MAXIMUM_ROTATIONAL_SPEED);

        swerveSubsystem.setSwerveDrive(xSpeed, ySpeed, rotationSpeed, true);
    }

}