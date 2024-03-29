package frc.robot;

import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    // Timer for keeping track of when to disable brakes after being disabled so
    // that the robot stops safely - DO NOT USE COMMANDS-DOES NOT WORK WHEN DISABLED
    private Timer brakesOffTimer = new Timer();

    private RobotContainer robotContainer = null;
    private Command m_autonomousCommand = null;
    //Auto selector on SmartDashboard

    public Robot() {
        robotContainer = new RobotContainer(DriverStation.getAlliance());
        if (!Constants.COMP_MODE) {
            DriverStation.silenceJoystickConnectionWarning(true);
        } else {
            DriverStation.silenceJoystickConnectionWarning(false);
        }

        
    }
    // add path group
    
    @Override
    public void robotInit() {
        //Create the auto programs in robotInit because it uses a ton of trigonometry, which is computationally expensive
        //auto.createPrograms();
        // PathPlannerServer.startServer(5811);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (Constants.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putNumber("Battery Voltage", RobotController.getBatteryVoltage());
            SmartDashboard.putNumber("Drive Controller Temp",
                    robotContainer.getSwerveDriveSubsystem().getDriveTemperature());
            SmartDashboard.putNumber("Rotation Controller Temp",
                    robotContainer.getSwerveDriveSubsystem().getRotationTemperature());
            SmartDashboard.putNumber("Robot Heading", robotContainer.getSwerveDriveSubsystem().getHeading());
        }
    }

    @Override
    public void disabledInit() {
        // Cancel all commands
        CommandScheduler.getInstance().cancelAll();
        robotContainer.getPivotSubsystem().stop();
        robotContainer.getTelescopingArmSubsystem().stop();
        // Write remaining blackbox data to file
        // Start brake timer
        brakesOffTimer.reset();
        brakesOffTimer.start();
    }

    @Override
    public void disabledPeriodic() {
        if (brakesOffTimer.get() > 2.5) {
            brakesOffTimer.stop();
            brakesOffTimer.reset();
            robotContainer.getSwerveDriveSubsystem().setBrakes(false);
        }
    }

    @Override
    public void autonomousInit() {
        robotContainer.getPivotSubsystem().stop();
        robotContainer.getTelescopingArmSubsystem().stop();
        robotContainer.getPivotSubsystem().RestEncoder();
        m_autonomousCommand = robotContainer.getAutonomousChooser().getSelected();

        m_autonomousCommand.schedule();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if(Constants.FIELD_ORIENTED){
            robotContainer.getSwerveDriveSubsystem().setFieldOriented(true, 0);
        }else{
            robotContainer.getSwerveDriveSubsystem().setFieldOriented(false, 0);
        }
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
            m_autonomousCommand = null;
        }
        robotContainer.getPivotSubsystem().stop();
        robotContainer.getTelescopingArmSubsystem().stop();
        robotContainer.getSwerveDriveSubsystem().setBrakes(true);
        robotContainer.getPivotSubsystem().setBrakes(true);
        robotContainer.getTelescopingArmSubsystem().setBrakes(true);

    }

    @Override
    public void teleopPeriodic() {
        robotContainer.getOI().checkInputs();
        if(robotContainer.getOI().getToggleFieldOriented()){
            robotContainer.getSwerveDriveSubsystem().setFieldOriented(!robotContainer.getSwerveDriveSubsystem().getFieldOriented(), 0);
        }
        
        if (Constants.REPORTING_DIAGNOSTICS) {

            SmartDashboard.putNumber("Joy X", robotContainer.getOI().getXVelocity());
            SmartDashboard.putNumber("Joy Y", robotContainer.getOI().getYVelocity());
            SmartDashboard.putNumber("Rotation", robotContainer.getOI().getRotationVelocity());
            SmartDashboard.putNumber("Slider", robotContainer.getOI().getSpeed());
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
