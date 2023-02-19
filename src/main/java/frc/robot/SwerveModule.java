package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

public class SwerveModule {

    // Restrictions on the minimum and maximum speed of the rotation motors (0 to 1)
    private double maxRotationSpeed = 1.0;
    private double minRotationSpeed = 0.0;
    private double epsilon = 0.000000000000001;
    private TalonFX driveMotor;
    private TalonFX rotationMotor;
    private CANCoder rotationEncoder;

    private double ticksPerMeter;

    // The rotation encoders all have their zero position in a different place, so
    // keep track of how far off zero is from straight ahead
    private double rotationOffset;

    // The right-hand modules have their wheels facing the other way, so we need to
    // invert their direction
    private double inversionConstant = 1.0;

    // The ID number of the module
    private int id;

    // The name of the module - not used for much other than debugging
    private String name;

    // The velocity (-1 to 1) to run the motor
    private double driveVelocity = 0.0;
    private double reverseMultiplier = 1.0;

    // Create a PID for controlling the angle of the module
    private PIDController angleController = new PIDController(0.4, 0.0, 0.001);

    // Create a PID for controlling the velocity of the module
    private PIDController velocityController = new PIDController(0.2, 1.2, 0.005);

    // Create a feedforward for Velocity
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);
    double newP = 0.65;
    double newI = 0.65;
    double newD = 0.005;
    private double curP = 0;
    private double curI = 0;
    private double curD = 0;

    private double feedforwardValue;

    //The last time the Swerve Module was updated
    private double lastUpdate = 0.0;
    private double lastVelocity = 0.0;

    // percent Output
    private double voltage;

    // Safety override
    private boolean cancelAllMotion = false;

    /**
     * Creates a swerve module with the given hardware
     * 
     * @param driveMotor         The Talon SRX running the wheel
     * @param rotationMotor      The Victor SPX that rotates the wheel
     * @param rotationEncoder    The input from the encoder
     * @param rotationOffset     The distance from zero that forward is on the
     *                           encoder
     * @param invertDrive        Whether to invert the direction of the wheel
     * @param driveTicksPerMeter The number of encoder ticks per meter on the drive
     *                           motor
     * @param id                 The ID of the module
     * @param name               The name of the module
     */
    public SwerveModule(TalonFX driveMotor, TalonFX rotationMotor, CANCoder rotationEncoder, double rotationOffset,
            boolean invertDrive, double driveTicksPerMeter, int id, String name) {
        this.driveMotor = driveMotor;
        this.rotationMotor = rotationMotor;
        this.rotationEncoder = rotationEncoder;
        this.rotationOffset = rotationOffset;

        // Current limit the motors to avoid brownouts
        this.driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 26, 26, 1));
        this.rotationMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 26, 26, 1));

        this.ticksPerMeter = driveTicksPerMeter;

        this.id = id;
        this.name = name;

        if (invertDrive) {
            this.inversionConstant = -1.0;
        }

        // Set up the encoder from the drive motor
        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        this.driveMotor.setSelectedSensorPosition(0);

        // Set up the encoder from the rotation motor
        this.rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        // Because the rotation is on a circle, not a line, we want to take the shortest
        // route to the setpoint - this function tells the PID it is on a circle from 0
        // to 2*Pi
        angleController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * This function should run every teleopPeriodic
     */
    public boolean periodic(double timestamp) {
        newP = SmartDashboard.getNumber("Drive-P", newP);
        newI = SmartDashboard.getNumber("Drive-I", newI);
        newD = SmartDashboard.getNumber("Drive-D", newD);
        if (Math.abs(curP - newP) > epsilon) {
            curP = newP;
            velocityController.setP(curP);
            velocityController.reset();
        }
        if (Math.abs(curI - newI) > epsilon) {
            curI = newI;
            velocityController.setI(curI);
            velocityController.reset();
        }
        if (Math.abs(curD - newD) > epsilon) {
            curD = newD;
            velocityController.setD(curD);
            velocityController.reset();
        }

        //Get the amount of time since the last update
        double period = timestamp - lastUpdate;
        double velocityChange = getVelocity() - lastVelocity;

        //Stores the current timestamp as the most recent update
        lastUpdate = timestamp;

        // Set the drive velocity
        double calculated = 0.0;
        double velocity = 0.0;
        double rotationVelocity = 0.0;
        if (driveVelocity != 0.0 && !cancelAllMotion) {
            // Get the rotation velocity
            rotationVelocity = angleController.calculate(getAngle());
            // Clamp the value (not scale because faster is okay, it's on a PID)
            rotationVelocity = MathUtil.clamp(rotationVelocity, -maxRotationSpeed, maxRotationSpeed);
            if (rotationVelocity > -minRotationSpeed && rotationVelocity < minRotationSpeed) {
                rotationVelocity = 0.0;
            }
            // Set the rotation motor velocity based on the next value from the angle PID,
            // clamped to not exceed the maximum speed
            rotationMotor.set(ControlMode.PercentOutput, rotationVelocity);

            calculated = velocityController.calculate(getVelocity());
            // divide feed forward by battery voltage (about 12.5?) to convert to percent output
            // maybe get the actual battery voltage instead?
            voltage = (calculated * inversionConstant) + ((feedforwardValue * inversionConstant) / 12.5);
            double percentVoltage = voltage;
            driveMotor.set(ControlMode.PercentOutput, percentVoltage);
            SmartDashboard.putNumber(name + " Percent Output", percentVoltage);
        } else {
            driveMotor.set(ControlMode.PercentOutput, 0.0);
            velocityController.reset();
            rotationMotor.set(ControlMode.PercentOutput, 0.0);
            angleController.reset();
        }

        // change feedforward
        feedforwardValue = feedforward.calculate(getVelocity(), velocityChange / period);

        if (Constants.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putNumber(name + " Speed Setpoint", driveVelocity);
            SmartDashboard.putNumber(name + " PID Output", rotationVelocity);
            SmartDashboard.putNumber(name + " PID Error", angleController.getPositionError());
            SmartDashboard.putNumber(name + " Raw Speed", velocity);
            SmartDashboard.putNumber(name + " Speed (m per s)", getVelocity());
            SmartDashboard.putNumber(name + " Angle", getAngle());
            SmartDashboard.putNumber(name + " Angle Target", getTargetAngle());
            SmartDashboard.putNumber(name + " Distance", getDistance(false));
            SmartDashboard.putNumber(name + " Raw Encoder Ticks", driveMotor.getSelectedSensorPosition());
            SmartDashboard.putNumber(name + " Raw Rotation", rotationEncoder.getAbsolutePosition());
            SmartDashboard.putNumber(name + " Calculated", calculated);
            SmartDashboard.putNumber(name + " FeedForward", feedforwardValue);
        }

        return false;
    }

    /**
     * Sets the velocity to drive the module in meters/second.
     * This can exceed the maximum velocity specified in RobotMap.
     * 
     * @param velocity The velocity to drive the module.
     */
    public void setDriveVelocity(double velocity) {
        driveVelocity = velocity * reverseMultiplier;
        velocityController.setSetpoint(driveVelocity);
    }

    /**
     * Sets the target in radians for the angle PID
     * 
     * @param angle The angle to try to reach. This value should be between -Pi and
     *              Pi
     */
    public void setTargetAngle(double angle) {
        double currentAngle = getAngle();

        // If the smallest angle between the current angle and the target is greater
        // than Pi/2, invert the velocity and turn the wheel to a closer angle
        // Math.atan2(y, x) computes the angle to a given point from the x-axis
        if (Math.abs(Math.atan2(Math.sin(angle - currentAngle), Math.cos(angle - currentAngle))) > Math.PI / 2.0) {
            angle += Math.PI;
            angle %= 2.0 * Math.PI;
            // Ensure the value is not negative
            if (angle < 0) {
                angle += 2.0 * Math.PI;
            }
            reverseMultiplier = -1.0;
        } else {
            reverseMultiplier = 1.0;
        }

        angleController.setSetpoint(angle);
    }

    /**
     * Get the distance that the drive wheel has turned
     * 
     * @param rawTicks Whether the output should be in raw encoder ticks instead of
     *                 meters
     */
    public double getDistance(boolean rawTicks) {
        // Raw encoder ticks
        double distance = driveMotor.getSelectedSensorPosition();

        if (!rawTicks) {
            // Meters
            distance /= ticksPerMeter;
        }

        return distance;
    }

    /**
     * Gets the current velocity in meters/second that the drive wheel is moving
     */
    public double getVelocity() {
        // Raw encoder ticks per 100 ms???? maybe 1s?
        double velocity = driveMotor.getSelectedSensorVelocity();

        // Raw encoder ticks per 1 s
        velocity *= 10;

        // Meters per second
        velocity /= ticksPerMeter;

        velocity = velocity * inversionConstant;
        return velocity;
    }

    /**
     * Gets the angle in radians of the module from -Pi to Pi
     */
    public double getAngle() {
        double angle = (rotationEncoder.getAbsolutePosition() / 360 * 2.0 * Math.PI) + rotationOffset;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        angle -= Math.PI;

        return angle;
    }

    /**
     * Gets the target angle in radians from the angle PID
     */
    public double getTargetAngle() {
        return angleController.getSetpoint();
    }

    public double getxvelocity() {
        double x = 0.0;
        x = getVelocity() * Math.cos(getAngle());
        return x;
    }

    public double getyvelocity() {
        double y = 0.0;
        y = getVelocity() * Math.sin(getAngle());
        return y;
    }
                

    /**
     * Stops all motion on this module - safety override
     */
    public void stop() {
        cancelAllMotion = true;
    }

    public void setBrakes(boolean brakesOn) {
        if (brakesOn) {
            driveMotor.setNeutralMode(NeutralMode.Brake);
            rotationMotor.setNeutralMode(NeutralMode.Brake);
        } else {
            driveMotor.setNeutralMode(NeutralMode.Coast);
            rotationMotor.setNeutralMode(NeutralMode.Coast);
        }
    }

    /**
     * Get the id of the module
     */
    public int getId() {
        return this.id;
    }

    /**
     * Get the name of the module
     */
    public String getName() {
        return this.name;
    }

    public double getDriveTemperature() {
        return driveMotor.getTemperature();
    }

    public double getRotationTemperature() {
        return rotationMotor.getTemperature();
    }
}
