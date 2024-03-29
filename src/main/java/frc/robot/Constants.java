package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.Preferences;

/**
 * A centralized file that keeps track of constants of the robot, such as device
 * IDs, device ports and robot dimensions
 * 
 * This is not the same as the RobotMaps from previous years, the only thing in
 * this class is constants, each hardware class defines its own motors and
 * whatnot
 */
public class Constants {
    // Disables some safeties and enables logging of warnings we expect and know
    // about during development
    public static final boolean COMP_MODE = false;

    /**
     * Identify which robot this is using Preferences on the rio. This is used to
     * get things like different ticks per meter, offsets, and dimensions.
     * If, for some reason you need to set this, you can put the following commented
     * line into the Robot class' constructor:
     * Preferences.setBoolean("compBot", *VALUE HERE*);
     */
    public static final boolean COMP_BOT = Preferences.getBoolean("compBot", true);

    // Enforces a maximum safe speed of the motors. This may cause steering issues.
    public static final double MAX_SAFE_SPEED_OVERRIDE = COMP_MODE ? 1.0 : 0.8;

// Measurements are in meters
    public static final double WHEELBASE = COMP_BOT ? 0.476: 0.476; 
    public static final double TRACK_WIDTH = COMP_BOT ? 0.584: 0.584;

    // Maximum linear speed is in meters/second
    public static final double MAXIMUM_SPEED = 3.75;
    public static final double SLOW_MAXIMUM_SPEED = 1.5;
    // USED ONLY IN AUTO - Maximum acceleration is in meters/second/second
    public static final double MAXIMUM_ACCELERATION = 2.0;

    public static final double MAXIMUM_ROTATIONAL_SPEED = Math.PI;
    // Maximum rotational speed is in radians/second Auto
    public static final double MAXIMUM_ROTATIONAL_SPEED_AUTO = Math.PI;
    // USED ONLY IN AUTO - Maximum rotational acceleration is in
    // radians/second/second
    public static final double MAXIMUM_ROTATIONAL_ACCELERATION = Math.PI;

//     // Swerve offset
//     public static final double[] ANGLE_OFFSET = COMP_BOT ? new double[] {
//         -24.0820313-45+5.2, 147.919922+135+15-0.6, 82.96875+104-5.2, 139.6582-180-10.65
//     }
//             : new double[] {
//                     0, 0, 0, 0
//             };
    public static final double[] ANGLE_OFFSET = COMP_BOT ? new double[] {
        24.0820313-90, -147.919922+90, -82.96875-90, -139.6582+90
    }
            : new double[] {
                    0, 0, 0, 0
            };
    /*
     * CAN ID and CAN Bus
     * CAN Bus options supported: "rio", "canivore"
     * ***IF CANIVORE FAILS CHANGE SWERVE_BUS_ACTIVE TO false***
     */

    public static final double driveKS = (0.32 / 12);
    public static final double driveKV = (1.51 / 12);
    public static final double driveKA = (0.27 / 12);

    // CAN FD Device IDs
    public static final int[] DRIVE_MOTORS_ID = { 1, 2, 3, 4 };
    public static final int[] ROTATION_MOTORS_ID = { 5, 6, 7, 8 };
    public static final int[] ROTATION_ENCODERS_ID = { 9, 10, 11, 12 };
    public static final int PIGEON_ID = 20;

    // CAN Legacy Device IDs
    public static final int PIVOT_MOTOR_ID = 13;
    public static final int PIVOT_MOTOR2_ID = 22;
    public static final int TELESCOPING_ARM_MOTOR_ID = 14;
    public static final int INTAKE_MOTOR_ID = 15;
    public static final int PIVOT_ENCODER_ID = 16;
    public static final int TELESCOPING_ARM_ENCODER_ID = 17;
    
    // CANdle
    public static final int CANDLE_ID = 21;
    public static final int[] yellow = {255, 225, 0};
    public static final int[] red = {255, 0, 0};
    public static final int[] blue = {100, 100, 255};
    public static final int[] purple = {100, 0, 100};

    /*
     * CAN Bus (Legacy) NOT CURRENTLY SUPPORTED
     * public static final String SPARK_MOTOR_BUS = "rio";
     */

    // PWM Ports
    public static final int INTAKE_MOTOR_PORT = 0;


    // Drive encoder ticks per meter
    public static final double[] TICKS_PER_METER = COMP_BOT ? new double[] {
        43310.1955374, 43310.1955374, 43310.1955374, 43310.1955374
    }
            : new double[] {
                    0, 0, 0, 0
            };

    // DRIVER CONFIG
    // Dead zones of each joystick - Measured from 0 to 1. This should always be at
    // least 0.1.
    public static final double JOY_DEAD_ZONE = 0.3;
    // Whether teleop should start in field oriented mode
    public static final boolean FIELD_ORIENTED = true;
    // The sensitivity value for the joysticks - the values are exponentiated to
    // this value, so higher numbers result in a lower sensitivity, 1 results in
    // normal sensitivity, and decimals increase sensitivity
    public static final double JOYSTICK_SENSITIVITY = 1;
    public static final double TURNING_SENSITIVITY = 3;

    // LOGGING
    // Set this to true if you want to log diagnostics to SmartDashboard
    public static final boolean REPORTING_DIAGNOSTICS = true;


    //Swerve Odometry Constants
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(WHEELBASE / 2.0, -TRACK_WIDTH / 2.0),
            new Translation2d(-WHEELBASE / 2.0, TRACK_WIDTH / 2.0),
            new Translation2d(-WHEELBASE / 2.0, -TRACK_WIDTH / 2.0));
    //Auto Balence
    public static final double BEAM_BALANACED_DRIVE_KP = 0.02;
    public static final double BEAM_BALANCED_GOAL_DEGREES = 0;
    public static final double BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES = 1;

    //telescoping arm setpoints
    public static final double TELESCOPING_STOWAGE_POINT = 452.0;
    public static final double TELESCOPING_INTAKE_POINT = 47.724609375;
    public static final double TELESCOPING_SCORING_POINT_CONE = -4809.0;
    public static final double TELESCOPING_SCORING_POINT_CUBE = 0.0;

    public static class VisionConstants {
        /**
         * Physical location of the camera on the robot, relative to the center of the robot.
         */
        public static final Transform3d CAMERA_TO_ROBOT =
            new Transform3d(new Translation3d(-0.3425, 0.0, -0.233), new Rotation3d());
        public static final Transform3d ROBOT_TO_CAMERA = CAMERA_TO_ROBOT.inverse();

        // AprilTag Positions for the 2023 competition
        public static final AprilTag[] AprilTagPos  = {
                new AprilTag(1, 15.513558, 1.071626, 0.462788, 0.0, 0.0, 0.0, 1.0), 
                new AprilTag(2, 15.513558, 2.748026, 0.462788, 0.0, 0.0, 0.0, 1.0),
                new AprilTag(3, 15.513558, 4.424426, 0.462788, 0.0, 0.0, 0.0, 1.0),
                
                new AprilTag(4, 16.178784, 6.749796, 0.695452, 0.0, 0.0, 0.0, 1.0),
                new AprilTag(5, 0.36195, 6.749796, 0.695452, 1.0, 0.0, 0.0, 0.0),
                
                new AprilTag(6, 1.02743, 4.424426, 0.462788, 1.0, 0.0, 0.0, 0.0),
                new AprilTag(7, 1.02743, 2.748026, 0.462788, 1.0, 0.0, 0.0, 0.0),
                new AprilTag(8, 1.02743, 1.071626, 0.462788, 1.0, 0.0, 0.0, 0.0)
        };
      }


      public static class placementConstants {

                public static enum placements {
                        leftLeftBlue(0),
                        leftMidBlue(1),
                        leftRightBlue(2),

                        midLeftBlue(3),
                        midMidBlue(4),
                        midRightBlue(5),

                        rightLeftBlue(6),
                        rightMidBlue(7),
                        rightRightBlue(8),

                        leftLeftRed(9),
                        leftMidRed(10),
                        leftRightRed(11),

                        midLeftRed(12),
                        midMidRed(13),
                        midRightRed(14),

                        rightLeftRed(15),
                        rightMidRed(16),
                        rightRightRed(17),

                        feederBlue(18),
                        feederRed(19);

                        private final int value;
                        placements(final int newvalue) {
                                value = newvalue;
                        }
                        public int getValue() {
                                return value;
                        }
                };
        
        };

        public static final double E_DTOLERANCE = 0.03; // meters
        public static final double E_RTOLERANCE = 3.0; // degrees

        // not endpoint tolerances
        public static final double DTOLERANCE = 0.09; // meters
        public static final double RTOLERANCE = 5.0; // degrees
        
        public static final double SPEEDLIMIT = 1.5; // meters per second
        public static final double ROTLIMIT = 2*Math.PI;


        // pigeon tip threshold to reset odometry in degrees
        public static final double tipThreshold = 5.0;

        // Intake time limit
        public static final double INTAKE_TIME_LIMIT = 0.25;

        // Outake Delay
        public static final double OUTTAKE_DELAY = 0.35;
}
