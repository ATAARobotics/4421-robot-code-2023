package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingArmSubsystem;

public class TelescopingArmCommand extends CommandBase{
    
    TelescopingArmSubsystem m_telescopingArmSubsystem;
    double setPoint;
    double TelescopingTolerance = 5.0;

    public TelescopingArmCommand(TelescopingArmSubsystem telescopingArmSubsystem, double setPoint) {
        this.m_telescopingArmSubsystem = telescopingArmSubsystem;
        this.setPoint = setPoint;
    }

    @Override
    public void execute() {
        m_telescopingArmSubsystem.out();
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(m_telescopingArmSubsystem.getEncoder() - setPoint) < TelescopingTolerance);
    }

    @Override
    public void end(boolean interrupted) {
        m_telescopingArmSubsystem.stop();
    }
}