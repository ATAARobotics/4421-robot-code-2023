// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveOdometry;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;


public class AprilTagLimelight extends SubsystemBase {  
  
  private static final double CAMERA_HEIGHT_METERS = 0.7;
  private static final double CAMERA_PITCH_RADIANS = 0;
  private static final double SAFETY_OFFSET = 0.6;
  private static final double AMBIGUITY_CUTOFF = 0.5;
  
  Gyro gyro;
  double range;
  Pose2d robotPose;
  PhotonCamera camera = new PhotonCamera("Limelight");
  PhotonCamera photonCamera;
  frc.robot.AprilTag aprilTagPos;
  SwerveOdometry odometry;
  
  public AprilTagLimelight(SwerveOdometry odometry) {
    super();
    this.range = 0.0d;
    this.odometry = odometry;
  }

  @Override
  public void periodic() {
    // Query the latest result from PhotonVision
    var result = camera.getLatestResult();
    // Check if the latest result has any targets.
    boolean hasTargets = result.hasTargets();
    // Get the current best target.
    
    int targetID = -1;

    if (hasTargets == true) {
      // Get information from target.
      PhotonTrackedTarget target = result.getBestTarget();
      double yaw = target.getYaw();
      double pitch = target.getPitch();
      double area = target.getArea();
      double skew = target.getSkew();
      //  Transform2d pose = target.getCameraToTarget();
      List<TargetCorner> corners = target.getDetectedCorners();
      targetID = target.getFiducialId();
      double poseAmbiguity = target.getPoseAmbiguity();
      if (poseAmbiguity >= AMBIGUITY_CUTOFF) {
        return;
      }
      //  Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      //  Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();
    
    // Vision-alignment mode
    // Query the latest result from PhotonVision
    result = camera.getLatestResult();

      robotPose = odometry.getPose();
        // First calculate range
        // double range = PhotonUtils.calculateDistanceToTargetMeters(
        //                   CAMERA_HEIGHT_METERS,
        //                   TARGET_HEIGHT_METERS,
        //                   CAMERA_PITCH_RADIANS,
        //                   Units.degreesToRadians(result.getBestTarget().getPitch()));
        //                   double GOAL_RANGE_METERS = range-SAFETY_OFFSET;
        //                   // Use this range as the measurement we give to the PID controller.
            
        // this.range = range; 



        // Calculate a translation from the camera to the target.
        // Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation( //what we want
        //   range, Rotation2d.fromDegrees(target.getYaw()));

        double kTargetPitch = target.getPitch();
        edu.wpi.first.math.geometry.Transform3d cameraToRobot = new edu.wpi.first.math.geometry.Transform3d();
        Pose3d aprilTagFieldLayout = new Pose3d();

        // red or blue side calculation
        boolean redSide = false;
      
        
        // Calculate robot's field relative pose
        Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout, cameraToRobot);
        

        // adds the position of robot to april tag to find the actual position
        if (targetID >= 1 && targetID <= 8) {
          aprilTagPos = Constants.VisionConstants.AprilTagPos[targetID-1];
          redSide = (aprilTagPos.aprilTagPose.getRotation().getQuaternion().getZ() > 0.5);
          Pose2d tempPose = getActualPose(robotPose.toPose2d(), aprilTagPos.aprilTagPose.toPose2d(), redSide);
          odometry.addAprilTag(tempPose);
        }
    }
    }

  public Pose2d getActualPose(Pose2d robot, Pose2d april, boolean redSide) {
    double x, y, rot, inverseMultipler;
    inverseMultipler = 1.0;

    if (redSide) {
      inverseMultipler = -1.0;
    }

    x = robot.getX() * inverseMultipler + april.getX();
    y = robot.getY() * inverseMultipler + april.getY();
    rot = (robot.getRotation().getRadians() + april.getRotation().getRadians()) % (2*Math.PI) - Math.PI;
    Pose2d newPose = new Pose2d(x, y, new Rotation2d(rot));
    return newPose;
  }


  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}
