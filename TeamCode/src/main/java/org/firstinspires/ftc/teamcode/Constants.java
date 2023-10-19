package org.firstinspires.ftc.teamcode;

public class Constants {
    public enum Commands { NONE, ROBOT_RESET, GYRO_RESET, DETERMINE_TEAM, PID_TURN_0, PID_TURN_90, PID_TURN_N90, PID_TURN_180 }
    public enum Alliance { RED, BLUE, NONE }
    public enum TSELocation { LEFT, MIDDLE, RIGHT, NONE }
    public static class Global {
    }
    public static class Auton {
        public static double autonDriveSpeed = 0.3;
    }
    public static class Drivetrain {
        public static class turnController {
            public static double targetThreshold = 1.0; //how many degrees is close enough
            public static double kP = 0.01;
            public static double kI = 0.0;
            public static double kD = 0.003;
        }
        public static class driveController {
            public static double ticksPerRev = 28;
            public static double gearReduction = 20;
            public static double wheelDiamMM = 75;
            public static double wheelCircumferenceMM = wheelDiamMM * Math.PI;
            public static double ticksPerMM = ticksPerRev * gearReduction / wheelCircumferenceMM;
            public static double ticksPerInch = ticksPerMM * 25.4;
            public static double targetThreshold = 0.5; //how many inches is close enough
            public static double targetThresholdTicks = targetThreshold * ticksPerInch;
            public static double kP = 0.0005;
            public static double kI = 0.0;
            public static double kD = 0.003;
        }
    }

    public static class PixelDropper {
        public static enum Positions {
            DOWN(0.0),
            UP(20.0);
            private final double angle;
            private Positions(double angle) { this.angle = angle; }
            public double getAngle() { return this.angle; }
        }
    }

    /**
     * The Manipulator constants relate to the top-end system above the drivetrain.
     * The manipulator contains 2 major parts, the elevator and the extend.
     * tilt - This controls the angle of the delivery "bucket" and ranges from ~-19 degrees (floor pickup) to ~+58 degrees (hanger) relative to the floor plane
     * elevator - This controls the length of the extended arm with the delivery bucket and ranges from 0 inches to 12.3 inches
     */
    public static class Manipulator {
        public static enum Positions {
            //NAME(angle,length,distance)
            //angle = (double) angle of the elevator, from the limit switch reference plane
            //length = (double) length of the extend
            //distance = (double) robot distance from backstage (-1 if not used)
            START(0.0,1.0,-1.0),
            TRANSPORT(18.0,0.0,-1.0),
            FLOOR_CLOSE(0.0,1.0,-1.0),
            FLOOR_FAR(3.0,2.0,-1.0),
            FLOOR_DESTACK(3.2,2.0,-1.0),
            SCORE_ROW1(49.6,1.9,0.0),
            //SCORE_ROWX(78.0,12.3,0.0),
            CLIMB_READY(78.0,3.0,-1.0),
            CLIMB_UP(78.0,12.3,-1.0),
            CLIMB_LIFT(78.0,11.0,-1.0);
            private final double angle, length, distance;
            private Positions(double angle, double length, double distance) {
                this.angle = angle;
                this.length = length;
                this.distance = distance;
            }
            public double getAngle() { return this.angle; }
            public double getLength() { return this.length; }
            public double getDistance() { return this.distance; }
        }
        public static class tiltController {
            public static double targetThreshold = 0.5; //how many degrees is close enough
            public static double kP = 0.01;
            public static double kI = 0.0;
            public static double kD = 0.003;
            public static class limits {
                public static double minAngle = 0.0; //Minimum angle of target (at limit sw) //this is -19 from horizontal floor plane
                public static double maxAngle = 78.0; //Maximum angle of target (at limit sw)
                public static double maxFloorPickupAngle = 10.0; //Maximum angle of floor pickup (extended position)
                public static double transportAngle = 19.0; //This should be part of an enum for different positions;
                public static double minScoringAngle = 50.6; //Lowest scoring position, just above bottom row of pixels;
                public static double maxScoringAngle = 65.0; //Highest scoring position, just above row of pixels that cross top scoring line;
            }
        }
        public static class elevatorController {
            public static double ticksPerRev = 28;
            public static double gearReduction = 20;
            public static double drumDiamInches = 1.3;
            public static double drumCircumferenceInches = drumDiamInches * Math.PI;
            public static double ticksPerInch = (ticksPerRev * gearReduction / drumCircumferenceInches) / 2; //2 stage elevator makes 2:1 reduction
            public static double targetThreshold = 0.5; //how many inches is close enough
            public static double targetThresholdTicks = targetThreshold * ticksPerInch;
            public static double kP = 0.0005;
            public static double kI = 0.0;
            public static double kD = 0.003;
            public static class limits {
                public static double minLength = 0.0; //Minimum extended length of arm
                public static double maxLength = 12.3; //Maximum extended length of arm
                public static double maxFloorPickupLength = 1.0; //At max floor pickup angle, extended length
                public static double minScoringLength = 1.9; //length at lowest scoring position
                public static double transportLength = minScoringLength;
                public static double maxScoringLength = 10.0; //length at highest scoring position
            }
        }
    }
}
