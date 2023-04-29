package frc.robot.commands;

import javax.swing.text.StyledEditorKit.BoldAction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class AutoBalance extends CommandBase {
    private double error;
    private double currentAngle;
    private double drivePower;
    private SwerveDriveSubsystem swerveSubsystem;
    private Boolean isForward;
    private Boolean firstisOn = false;
    private Boolean isOn = false;
    private Boolean flipedState = false;
    private int flipcount = 0;
    private boolean notSlow = true;
    public AutoBalance(SwerveDriveSubsystem swerve, Boolean isForward) {
        this.isForward = isForward;
        this.swerveSubsystem = swerve;
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        if(swerveSubsystem.getPigeon().getPitch() > 0){
            flipedState = true;
        }else{
            flipedState = false;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Uncomment the line below this to simulate the gyroscope axis with a
        // controller joystick
        // Double currentAngle = -1 *
        // Robot.controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS) * 45;
        // System.out.println(flipcount);
        this.currentAngle = swerveSubsystem.getPigeon().getPitch();
        if(flipedState && currentAngle <= 10){
            notSlow = false;
        }else{
            if(flipedState ==  false && currentAngle >= -10 ){
                notSlow = false;
            }
        }
        error = Constants.BEAM_BALANCED_GOAL_DEGREES - currentAngle;
        if(notSlow){
            drivePower = -MathUtil.clamp(error * Constants.BEAM_BALANACED_DRIVE_KP, -3, 3);
        }else{
            drivePower = -MathUtil.clamp(error * Constants.BEAM_BALANACED_DRIVE_KP * 0.2, -0.7, 0.7);
        }
        System.out.println(drivePower);
        // Our robot needed an extra push to drive up in reverse, probably due to weight
        // imbalances

        // Limit the max power
        if (Math.abs(drivePower) > 1) {
            drivePower = Math.copySign(1, drivePower);
        }

        swerveSubsystem.setSwerveDrive(drivePower, 0, 0, 0, false);
        // Debugging Print Statments
        SmartDashboard.putNumber("Current Angle", currentAngle);
        SmartDashboard.putNumber("Error", error);
        SmartDashboard.putNumber("Drive Power", drivePower);
    
    }


    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
       return false;
    }

}