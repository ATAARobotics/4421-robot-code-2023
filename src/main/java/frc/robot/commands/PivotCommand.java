package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.*;

public class PivotCommand extends CommandBase{

    PivotSubsystem m_pivotSubsystem;
    String state;

    public PivotCommand(PivotSubsystem intakeSubsystem, String state) {
        this.m_pivotSubsystem = intakeSubsystem;
        this.state = state;
    }
    @Override
    public void initialize() {
        switch(state){
            case "up":
                m_pivotSubsystem.up();
                break;
            case "down":
                m_pivotSubsystem.down();
                break;
            case "firstdown":
                m_pivotSubsystem.firstdown();
                break;
        }
      m_pivotSubsystem.up();
      
    }

    @Override
    public boolean isFinished() {
        return m_pivotSubsystem.getMovementState() == 0;
    }
}
