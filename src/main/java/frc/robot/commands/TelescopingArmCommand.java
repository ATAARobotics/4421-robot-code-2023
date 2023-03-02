package frc.robot.commands;

import javax.print.DocFlavor.STRING;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopingArmSubsystem;

public class TelescopingArmCommand extends CommandBase{
    
    TelescopingArmSubsystem m_telescopingArmSubsystem;
    double setPoint;
    double TelescopingTolerance = 5.0;
    String state;
    boolean firstrun = true;
    public TelescopingArmCommand(TelescopingArmSubsystem telescopingArmSubsystem, String state) {
        this.m_telescopingArmSubsystem = telescopingArmSubsystem;
        this.state = state;
    }

    @Override
    public void initialize() {
        switch(state){
            case "cube":
                firstrun = false;
                m_telescopingArmSubsystem.scoreCube();
                break;
        }
      
    }

    @Override
    public boolean isFinished() {
        return m_telescopingArmSubsystem.getMovementState() == 0 && firstrun == false;
    }
}