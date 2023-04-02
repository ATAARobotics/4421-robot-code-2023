package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class DeadReckoning extends CommandBase{

    SwerveDriveSubsystem m_SwerveDriveSubsystem;
    Timer timer;
    double X_Vel;
    double Y_Vel;
    double R_Vel;
    double timeRunning;

    public DeadReckoning(SwerveDriveSubsystem swerveDriveSubsystem, double X_Vel, double Y_Vel, double timeRunning) {
        this.m_SwerveDriveSubsystem = swerveDriveSubsystem;
        this.X_Vel = X_Vel;
        this.Y_Vel = Y_Vel;
        this.timer = new Timer();
        this.timeRunning = timeRunning;
    }

    @Override
    public void execute() {
        timer.start();
        m_SwerveDriveSubsystem.setSwerveDrive(-X_Vel, -Y_Vel, 0, true);
    }

    @Override
    public boolean isFinished() {
        return timer.get() > timeRunning;
    }
    @Override
    public void end(boolean interrupted) {
        m_SwerveDriveSubsystem.setSwerveDrive(0, 0, 0, true);
    }
}
