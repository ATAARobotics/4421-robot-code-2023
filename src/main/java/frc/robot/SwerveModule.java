package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private PIDController angleController = new PIDController(0.8, 0.0, 0.001);

    // Create a PID for controlling the velocity of the module
    private PIDController velocityController = new PIDController(0.2, 1.2, 0.005);

    // Create a feedforward for Velocity
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.driveKS, Constants.driveKV, Constants.driveKA);
    double newP = 0.65;
    double newI = 0.65;
    double newD = 0.005;
    private Rotation2d lastAngle = new Rotation2d();
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

    public void setState(SwerveModuleState state){
        double speed = state.speedMetersPerSecond;
        double angle = state.angle.getRadians();
        SwerveModuleState finalSwerveState = new SwerveModuleState(speed,Rotation2d.fromRadians(angle));

        setTargetAngle(finalSwerveState.angle.getRadians());
        setSpeed(finalSwerveState);
        setAngle();
    }

    private void setAngle(){
        SmartDashboard.putNumber(name + " angle", rotationEncoder.getAbsolutePosition()+rotationOffset);
        double rotationVelocity = -angleController.calculate(getAngle());
        // Clamp the value (not scale because faster is okay, it's on a PID)
        rotationVelocity = MathUtil.clamp(rotationVelocity, -maxRotationSpeed, maxRotationSpeed);
        if (rotationVelocity > -minRotationSpeed && rotationVelocity < minRotationSpeed) {
            rotationVelocity = 0.0;
        }
        // Set the rotation motor velocity based on the next value from the angle PID,
        // clamped to not exceed the maximum speed
        rotationMotor.set(ControlMode.PercentOutput, rotationVelocity);
    }

    private void setSpeed(SwerveModuleState state){
        double percentOutput = inversionConstant*reverseMultiplier*state.speedMetersPerSecond / 4.5;
        driveMotor.set(ControlMode.PercentOutput, percentOutput);
    }
    /**
     * Sets the velocity to drive the module in meters/second.
     * This can exceed the maximum velocity specified in RobotMap.
     * 
     * @param velocity The velocity to drive the module.
     */


    /**
     * Get the distance that the drive wheel has turned
     * Get the distance that the drive wheel has turned
     * 
     * @param rawTicks Whether the output should be in raw encoder ticks instead of
     *                 meters
     */


    /**
     * Gets the current velocity in meters/second that the drive wheel is moving
     */

    /**
     * Gets the angle in radians of the module from -Pi to Pi
     * @param rawTicks Whether the output should be in raw encoder ticks instead of
     *                 meters
     */


    /**
     * Gets the current velocity in meters/second that the drive wheel is moving
     */

    /**
     * Gets the angle in radians of the module from -Pi to Pi
     */
    public double getAngle() {
        double angle = ((rotationEncoder.getAbsolutePosition()+rotationOffset) / 360 * 2.0 * Math.PI) ;
        angle %= 2.0 * Math.PI;
        if (angle < 0.0) {
            angle += 2.0 * Math.PI;
        }
        angle -= Math.PI;

        return angle;
    }

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
     * Gets the target angle in radians from the angle PID
     */
    public double getTargetAngle() {
        return angleController.getSetpoint();
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

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(inversionConstant*driveMotor.getSelectedSensorPosition() * (Units.inchesToMeters(4.0)*Math.PI) / (6.75 * 2048.0), Rotation2d.fromDegrees(rotationEncoder.getAbsolutePosition()+rotationOffset));
    }
}
