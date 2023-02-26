package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveOdometry {

    //Stores the current position of the robot
    private Pose2d pose;
    private Pigeon pigeon;
    private SwerveModulePosition[] modulePostions;
    //The last time the odometry was updated
    private double lastUpdate = 0.0;

    private boolean isInitialized = false;


    public SwerveDriveOdometry swerveOdometry;

    public SwerveOdometry(Pose2d initialPose, Pigeon pigeon, SwerveModulePosition[] modulePostions) {
        this.pose = initialPose;
        this.pigeon = pigeon;
        this.modulePostions = modulePostions;
        swerveOdometry = new SwerveDriveOdometry(Constants.swerveKinematics, Rotation2d.fromRadians(pigeon.getYaw()), modulePostions);
    }

    /**
     * Updates the current location of the robot
     * @param currentAngle The current angle given by the gyro from -Pi to Pi
     * @param timestamp The current timestamp
     */
    public Pose2d update(SwerveModulePosition[] modulePostions) {
        this.modulePostions = modulePostions;
        pose = swerveOdometry.update(Rotation2d.fromRadians(pigeon.getYaw()), modulePostions);

        //Updates the position of the robot based on the distance traveled
        SmartDashboard.putNumber("Robot Pose X", pose.getX());
        SmartDashboard.putNumber("Robot Pose Y", pose.getY());
        SmartDashboard.putNumber("Robot Pose Angle", pose.getRotation().getDegrees());

        return pose;
        
    }

    /**
     * Gets the current pose of the robot as a Pose2d object
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Sets a new pose manually
     */
    public void setPose(Pose2d pose, SwerveModulePosition[] modulePostions) {
        this.swerveOdometry.resetPosition(Rotation2d.fromRadians(pigeon.getYaw()), modulePostions, pose);

    }

    // take the average of the 2 poses
    public void addAprilTag(Pose2d pose) {

        if (!isInitialized) {
            setPose(pose, modulePostions);
            pigeon.setYaw(-Math.PI);
            isInitialized = true;
            return;
        }
        if (AprilTagError(this.pose, pose)){
            double x, y;
        
            x = (this.pose.getX() + pose.getX()) / 2.0;
            y = (this.pose.getY() + pose.getY()) / 2.0;
            this.pose = new Pose2d(x, y, this.pose.getRotation());
        }
    }
    public boolean AprilTagError(Pose2d currentPose, Pose2d newPose){
        double error = Math.sqrt(Math.pow(currentPose.getX() - newPose.getX(), 2) + Math.pow(currentPose.getY() - newPose.getY(), 2));
        if (error < 0.1){
            return true;
        }
        else{
            return false;
        }
    }

    public void reinitialize() {
        isInitialized = false;
    }

    public boolean getIsInitialized() {
        return isInitialized;
    }
}
