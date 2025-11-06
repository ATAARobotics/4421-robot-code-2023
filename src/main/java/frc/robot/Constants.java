package frc.robot;

public class Constants {
    public static class PivotConstants {
        public static final int FrontLeftPivotMotorID = 31;
        public static final int RearLeftPivotMotorID = 33;
        public static final int FrontRightPivotMotorID = 32;
        public static final int RearRightPivotMotorID = 34;

        public static final int EncoderID = 35;

        public static final double maxSpeed = 0.60;
        public static final double tolerance = 0.0;

        public static final double maxAngle = 0.241455;
        public static final double minAngle = -0.01;

        public static final boolean leftInverted = true;
        public static final boolean rightInverted = false;

        public static final double ffValue = 0.015;
        
        public static final double p = 2.5;
        public static final double i = 0.00025;
        public static final double d = 0;
    }    
}