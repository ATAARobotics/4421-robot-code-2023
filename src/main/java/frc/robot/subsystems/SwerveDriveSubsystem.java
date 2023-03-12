package frc.robot.subsystems;

import java.util.function.Consumer;
import javax.swing.plaf.synth.SynthDesktopIconUI;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Pigeon;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.SwerveOdometry;

public class SwerveDriveSubsystem extends SubsystemBase {
    private Pigeon pigeon;

    // Whether the swerve should be field-oriented
    boolean fieldOriented = true;

    // An array of all the modules on the swerve drive
    private SwerveModule[] swerveModules;

    // The odometry for the swerve drive
    private SwerveOdometry odometry;
    private boolean useOdometry = false;

    // The current pose of the robot
    private Pose2d pose;
    private double autoOffset = 0;

    // The position that the robot started at
    private Pose2d initialPose;

    private double driveMotorHighestTemp = 0;
    private double rotationMotorHighestTemp = 0;

    private double initialPoseX;
    private double initialPoseY;

    // Safety speed override, this *shouldn't* ever be true
    private boolean safetyDisable = false;

    private double xVelocity;
    private double yVelocity;
    private double rotationVelocity;

    private double[] velocities;
    private double[] angles;
    private ChassisSpeeds moduleSpeeds =new ChassisSpeeds(0, 0, 0);
    
    public final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.WHEELBASE / 2.0, -Constants.TRACK_WIDTH / 2.0),
        new Translation2d(Constants.WHEELBASE / 2.0, Constants.TRACK_WIDTH / 2.0),
        new Translation2d(-Constants.WHEELBASE / 2.0, -Constants.TRACK_WIDTH / 2.0),
        new Translation2d(-Constants.WHEELBASE / 2.0, Constants.TRACK_WIDTH / 2.0));
            /**
     * Set up the swerve drive
     * 
     * @param gyro        The gyro object running on the robot
     * @param initialPose The initial pose that the robot is in
     */
    public SwerveDriveSubsystem(Pigeon pigeon, Translation2d initialPosition, String bus) {
        this.pigeon = pigeon;
        this.initialPose = new Pose2d(initialPosition, new Rotation2d(0.0));
        TalonFX[] driveMotors = { new TalonFX(Constants.DRIVE_MOTORS_ID[0], bus),
                new TalonFX(Constants.DRIVE_MOTORS_ID[1], bus), new TalonFX(Constants.DRIVE_MOTORS_ID[2], bus),
                new TalonFX(Constants.DRIVE_MOTORS_ID[3], bus) };
        TalonFX[] rotationMotors = { new TalonFX(Constants.ROTATION_MOTORS_ID[0], bus),
                new TalonFX(Constants.ROTATION_MOTORS_ID[1], bus), new TalonFX(Constants.ROTATION_MOTORS_ID[2], bus),
                new TalonFX(Constants.ROTATION_MOTORS_ID[3], bus) };

        // Initialize four swerve modules using the SwerveModule class
        SwerveModule frontLeftModule = new SwerveModule(driveMotors[0], rotationMotors[0],
                new CANCoder(Constants.ROTATION_ENCODERS_ID[0], bus), Constants.ANGLE_OFFSET[0], true,
                Constants.TICKS_PER_METER[0], 0, "Front Left");
        SwerveModule frontRightModule = new SwerveModule(driveMotors[1], rotationMotors[1],
                new CANCoder(Constants.ROTATION_ENCODERS_ID[1], bus), Constants.ANGLE_OFFSET[1], true,
                Constants.TICKS_PER_METER[1], 1, "Front Right");
        SwerveModule rearLeftModule = new SwerveModule(driveMotors[2], rotationMotors[2],
                new CANCoder(Constants.ROTATION_ENCODERS_ID[2], bus), Constants.ANGLE_OFFSET[2], true,
                Constants.TICKS_PER_METER[2], 2, "Rear Left");
        SwerveModule rearRightModule = new SwerveModule(driveMotors[3], rotationMotors[3],
                new CANCoder(Constants.ROTATION_ENCODERS_ID[3], bus), Constants.ANGLE_OFFSET[3], true,
                Constants.TICKS_PER_METER[3], 3, "Rear Right");

        // Put the swerve modules in an array so we can process them easier
        swerveModules = new SwerveModule[] {
                frontLeftModule,
                frontRightModule,
                rearLeftModule,
                rearRightModule
        };



        // Set up odometry
        odometry = new SwerveOdometry(initialPose, pigeon);

        // Initialize the pose
        pose = initialPose;
    }

    /**
     * This function should be run during every teleop and auto periodic
     */
    public void setSwerveDrive(double xVelocity, double yVelocity, double rotationVelocity, boolean useOdometry) {
        if (fieldOriented) {
            this.moduleSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, yVelocity, rotationVelocity, Rotation2d.fromDegrees(-pigeon.getYaw()));
        }else{
            this.moduleSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
        }
        this.useOdometry = useOdometry;
    }

    public SwerveOdometry getOdometry() {
        return this.odometry;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Field Oriented", fieldOriented);
        double gyroAngle = getHeading();
        SmartDashboard.putNumber("Gyro Value", pigeon.getYaw());
        SmartDashboard.putNumber("Gyro Value RAW", pigeon.getYawRaw());

        SmartDashboard.putBoolean("IS FIELD ORIENTED", this.fieldOriented);

        // check if robot is tipped
        // checkTipped();
        SmartDashboard.putBoolean("Odometry Initialized", odometry.getIsInitialized());

        // Smart Dashboard PID
        SmartDashboard.setDefaultNumber("Drive-P", 0.2);
        SmartDashboard.setDefaultNumber("Drive-I", 1.2);
        SmartDashboard.setDefaultNumber("Drive-D", 0.005);

        

        if (Constants.REPORTING_DIAGNOSTICS) {
            SmartDashboard.putNumber("X Velocity", xVelocity);
            SmartDashboard.putNumber("Y Velocity", yVelocity);
            SmartDashboard.putNumber("Rotation Velocity", rotationVelocity);
        }

        if (!safetyDisable) {
            // if (Constants.REPORTING_DIAGNOSTICS) {
            // SmartDashboard.putNumber("Gyro Value", pigeon.getYaw());
            // }
            SwerveModuleState[] moduleStates = swerveKinematics.toSwerveModuleStates(moduleSpeeds);
            // Execute functions on each swerve module
            for (SwerveModule module : swerveModules) {
                module.setState(moduleStates[module.getId()]);
                // Run periodic tasks on the module (running motors)
            }
        } else {
            for (SwerveModule module : swerveModules) {
                module.stop();
            }
            DriverStation.reportError("DANGER: MODULES DISABLED FOR SAFETY", false);
        }

        // Get motor temperatures
        if (Constants.REPORTING_DIAGNOSTICS) {
            double driveTemp = Double.NEGATIVE_INFINITY;
            double rotTemp = Double.NEGATIVE_INFINITY;
            for (SwerveModule module : swerveModules) {
                driveTemp = Math.max(driveTemp, module.getDriveTemperature());
                rotTemp = Math.max(rotTemp, module.getRotationTemperature());
            }
            driveMotorHighestTemp = driveTemp;
            rotationMotorHighestTemp = rotTemp;
        }
    }

    /**
     * Sets whether the robot should be field-oriented
     */
    public void setFieldOriented(boolean fieldOriented, double currentYaw) {
        this.fieldOriented = fieldOriented;
        pigeon.setYaw(currentYaw);
        pigeon.resetPitch();
    }

    /**
     * Gets whether the robot is field-oriented
     */
    public boolean getFieldOriented() {
        return fieldOriented;
    }

    /**
     * Gets the robot heading
     */
    public double getHeading() {
        return pigeon.getYaw();
    }

    /**
     * Resets the robot heading
     */

    /**
     * Gets the current pose of the robot
     */
    public Pose2d getPose() {
        System.out.println("Diff X: " + Math.abs(pose.getX() - initialPoseX));
        System.out.println("Diff Y: " + Math.abs(pose.getY() - initialPoseY));
        return pose;
    }

    public void SetAutoOffset(double autoOffset) {
        this.autoOffset = autoOffset;
    }

    /**
     * Resets the pose to the initial pose
     */
    public void resetPosition() {
        pose = initialPose;
        odometry.setPose(pose);
    }

    public void resetHeading() {
        pigeon.setYaw(0);
    }

    /**
     * Sets the initial pose of the swerve (useful for auto)
     */
    public void setInitialPose(Pose2d pose) {
        initialPose = pose;
        odometry.setPose(pose);
        initialPoseX = pose.getX();
        initialPoseY = pose.getY();
    }

    /**
     * Sets the brakes on each module
     */
    public void setBrakes(boolean brakesOn) {
        for (SwerveModule module : swerveModules) {
            module.setBrakes(brakesOn);
        }
    }

    /**
     * Get the temperature of the hottest drive motor in degrees Celsius
     */
    public double getDriveTemperature() {
        return driveMotorHighestTemp;
    }

    /**
     * Get the temperature of the hottest rotation motor in degrees Celsius
     */
    public double getRotationTemperature() {
        return rotationMotorHighestTemp;
    }

    // turning left is positive, thus the negative
    public Consumer<ChassisSpeeds> setChassisSpeed = chassisSpeed -> {
        // System.out.println(chassisSpeed);
        this.setSwerveDrive(chassisSpeed.vxMetersPerSecond, chassisSpeed.vyMetersPerSecond,
                -chassisSpeed.omegaRadiansPerSecond, true);
    };

    /**
     * Gets the velocity of a specific module in meters/second
     * 
     * @param moduleId The ID of the module to get
     */
    public double getModuleVelocity(int moduleId) {
        return velocities[moduleId];
    }

    /**
     * Gets the angle that the module should be set to
     * 
     * @param moduleId The ID of the module to get
     */
    public double getModuleAngle(int moduleId) {
        return angles[moduleId];
    }

    public double getXVelocity() {

        return xVelocity;
    }

    public double getYVelocity() {
        return yVelocity;
    }

    public double getRotationVelocity() {
        return rotationVelocity;
    }

    public void checkTipped() {

        // Tipping Checker
        if (Math.abs(pigeon.getPitch()) > Constants.tipThreshold) {
            odometry.reinitialize();
        }

        if (Math.abs(pigeon.getRoll()) > Constants.tipThreshold) {
            odometry.reinitialize();
        }
    }

    public Pigeon getPigeon(){
        return pigeon;
    }
}
