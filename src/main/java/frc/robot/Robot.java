package frc.robot;

<<<<<<< HEAD
=======
import java.util.List;


>>>>>>> b6e67774626d2e33a32a35b2129f984621e419ff
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    // Timer for keeping track of when to disable brakes after being disabled so
    // that the robot stops safely - DO NOT USE COMMANDS-DOES NOT WORK WHEN DISABLED
    private Timer brakesOffTimer = new Timer();

    private RobotContainer robotContainer = null;

<<<<<<< HEAD
    private Command m_autonomousCommand = null;;
=======
    private Command m_autonomousCommand = null;
>>>>>>> b6e67774626d2e33a32a35b2129f984621e419ff
    //Auto selector on SmartDashboard

    public Robot() {
        robotContainer = new RobotContainer();
        if (!Constants.COMP_MODE) {
            DriverStation.silenceJoystickConnectionWarning(true);
        } else {
            DriverStation.silenceJoystickConnectionWarning(false);
        }

        
    }
<<<<<<< HEAD

=======
    // add path group
>>>>>>> b6e67774626d2e33a32a35b2129f984621e419ff
    
    @Override
    public void robotInit() {
        //Create the auto programs in robotInit because it uses a ton of trigonometry, which is computationally expensive
        //auto.createPrograms();
<<<<<<< HEAD

=======
>>>>>>> b6e67774626d2e33a32a35b2129f984621e419ff
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
<<<<<<< HEAD
        m_autonomousCommand = robotContainer.getAutonomousChooser().getSelected();
        robotContainer.AutoInit(0);
        m_autonomousCommand.schedule();
    }
=======
        
      }
>>>>>>> b6e67774626d2e33a32a35b2129f984621e419ff

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if(!Constants.FIELD_ORIENTED){
            robotContainer.getSwerveDriveSubsystem().setFieldOriented(false, 0);
        }
<<<<<<< HEAD
        SmartDashboard.putString("Limelight State", "Messuring Not Started");
=======

>>>>>>> b6e67774626d2e33a32a35b2129f984621e419ff

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
            m_autonomousCommand = null;
        }
        robotContainer.getSwerveDriveSubsystem().setBrakes(true);
<<<<<<< HEAD
=======

>>>>>>> b6e67774626d2e33a32a35b2129f984621e419ff
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
