package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="NCCOGS TeleOp", group="Jim")
@Disabled
public class nccogsTeleOp extends LinearOpMode {
    nccogsHardware robot = new nccogsHardware();

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize local variables
        double fwd;
        double strafe;
        double turn;
        double heading;
        double max;

        /* Initialize the robot hardware variables
         * The init() method of the hardware class does all the work here
         */
        robot.init();

        // Send telemetry message to signify the robot is waiting;
        telemetry.addData("Say","Hello Driver");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (isStopRequested()) return;

        //run the main loop until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            fwd = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;
            heading = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            //get field oriented fwd and strafe values
            double rotFwd = strafe * Math.sin(-heading) + fwd * Math.cos(-heading);
            double rotStrafe = strafe * Math.cos(-heading) - fwd * Math.sin(-heading);

            //counter strafe efficiency losses
            rotStrafe = rotStrafe * 1.1;

            //normalize motor speeds
            max = Math.max(Math.abs(rotFwd) + Math.abs(rotStrafe) + Math.abs(turn), 1);
            double fLPower = (rotFwd + rotStrafe + turn) / max;
            double rLPower = (rotFwd - rotStrafe + turn) / max;
            double fRPower = (rotFwd - rotStrafe - turn) / max;
            double rRPower = (rotFwd + rotStrafe - turn) / max;

            //set the motor speeds
            robot.fL.setPower(fLPower);
            robot.rL.setPower(rLPower);
            robot.fR.setPower(fRPower);
            robot.rR.setPower(rRPower);

        }
    }
}
