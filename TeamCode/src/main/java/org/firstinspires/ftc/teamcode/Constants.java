package org.firstinspires.ftc.teamcode;

public class Constants {
    enum Commands {
        NONE,
        ROBOT_RESET,
        GYRO_RESET
    }
    public static class Global {
    }
    public static class Auton {
        public static double autonDriveSpeed = 0.3;
    }
    public static class Drivetrain {
        public static double ticksPerRev = 28;
        public static double gearReduction = 20;
        public static double wheelDiamInMM = 75;
        public static double wheelCircumferenceInMM = wheelDiamInMM * Math.PI;
        public static double ticksPerMM = ticksPerRev * gearReduction / wheelCircumferenceInMM;
        public static double ticksPerInch = ticksPerMM * 25.4;
        public static class turnController {
            public static double targetThreshold = 1.0; //how many degrees is close enough
            public static double kP = 0.01;
            public static double kI = 0.0;
            public static double kD = 0.003;
        }
        public static class driveController {
            public static double targetThreshold = 0.5; //how many inches is close enough
            public static double targetThresholdTicks = targetThreshold * ticksPerInch;
            public static double kP = 0.0005;
            public static double kI = 0.0;
            public static double kD = 0.003;
        }
    }
}
