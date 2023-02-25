package frc.robot;

import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.util.Properties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

class OI {

    private BetterJoystick driveStick = new BetterJoystick(0, 1);
    private BetterJoystick rotationStick = new BetterJoystick(1, 1);
    private BetterJoystick gunnerStick = new BetterJoystick(2, 0);

    //Driver Values
    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;
    private boolean toggleFieldOriented;
    private double speed;
    public JoystickButton Forward;
    public JoystickButton AutoBalance;

    //Gunner Values
    public JoystickButton PivotUp;
    public JoystickButton PivotDown;
    public JoystickButton IntakeIn;
    public JoystickButton IntakeOut;
    public JoystickButton TelescopingOut;
    public JoystickButton TelescopingIn;

    public OI() {
        // Configure the button bindings
        try (InputStream input = new FileInputStream("/home/lvuser/deploy/bindings.properties")) {
            Properties bindings = new Properties();

            bindings.load(input);

            driveStick.configureBindings(bindings);
            rotationStick.configureBindings(bindings);
            gunnerStick.configureBindings(bindings);

            input.close();
        } catch (FileNotFoundException e) {
            DriverStation.reportError("Button bindings file not found!", false);
        } catch (IOException e) {
            DriverStation.reportError("IOException on button binding file", false);
        }

        // Set up command-based stuff
        Forward = driveStick.getWPIJoystickButton("Forward");
        AutoBalance = driveStick.getWPIJoystickButton("AutoBalance");
        
        PivotUp = gunnerStick.getWPIJoystickButton("PivotUp");
        PivotDown = gunnerStick.getWPIJoystickButton("PivotDown");
        IntakeIn = gunnerStick.getWPIJoystickButton("IntakeIn");
        IntakeOut = gunnerStick.getWPIJoystickButton("IntakeOut");
        TelescopingOut = gunnerStick.getWPIJoystickButton("TelecopingOut");
        TelescopingIn = gunnerStick.getWPIJoystickButton("TelecopingIn");
    }

    public void rumbleGunnerOn() {
        gunnerStick.setRumble(1);
    }

    public void rumbleGunnerOff() {
        gunnerStick.setRumble(0);
    }

    public void checkInputs() {
        speed = (-driveStick.getAnalog("Speed") + 1) / 4 + 0.5;
        xVelocity = driveStick.getAnalog("XVelocity")*speed;
        yVelocity = driveStick.getAnalog("YVelocity")*speed;
        //rotationVelocity = rotationStick.getAnalog("XVelocity");
        rotationVelocity = driveStick.getAnalog("RotationVelocity");


        // Dead zones
        if (Math.sqrt(Math.pow(xVelocity, 2) + Math.pow(yVelocity, 2)) < Constants.JOY_DEAD_ZONE) {
            xVelocity = 0;
            yVelocity = 0;
        }
        if (Math.abs(rotationVelocity) < Constants.JOY_DEAD_ZONE) {
            rotationVelocity = 0;
        }
        //kinda crimnal 
        xVelocity = Math.signum(xVelocity) * Math.abs(Math.pow(xVelocity, Constants.JOYSTICK_SENSITIVITY));
        yVelocity = Math.signum(yVelocity) * Math.abs(Math.pow(yVelocity, Constants.JOYSTICK_SENSITIVITY));
        rotationVelocity = Math.signum(rotationVelocity)
                * Math.abs(Math.pow(rotationVelocity, Constants.TURNING_SENSITIVITY));
        
        toggleFieldOriented = driveStick.getButton("ToggleFieldOriented");
    }

    // Getter functions for controls
    public double getXVelocity() {
        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public double getSpeed() {
        return speed;
    }

    public double getRotationVelocity() {
        return rotationVelocity;
    }

    public boolean getToggleFieldOriented() {
        return toggleFieldOriented;
    }
}