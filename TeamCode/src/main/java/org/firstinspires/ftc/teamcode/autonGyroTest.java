package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

@Autonomous(name="Gyro Test", group="JRB")
public class autonGyroTest extends LinearOpMode {
    hwMecanum robot = new hwMecanum();
    private ElapsedTime runtime = new ElapsedTime();
    private double lastAngle = 0.0;
    private double currAngle = 0.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        turn(90);
        sleep(3000);
        turnTo(-90);

    }

    public void resetAngle() {
        lastAngle = robot.getRobotYaw();
        currAngle = 0.0;
    }

    public double getAngle() {
        double yaw = robot.getRobotYaw();
        double deltaAngle = yaw - lastAngle;
        deltaAngle = normalizeAngle(deltaAngle);
        currAngle += deltaAngle;
        lastAngle = yaw;
        telemetry.addData("Gyro", String.format(Locale.ENGLISH,"%.2f",yaw));
        return currAngle;
    }

    public double normalizeAngle(double angle) {
        if (angle > 180) return angle - 360;
        if (angle < -180) return angle + 360;
        return angle;
    }

    public void turn(double degrees) {
        resetAngle();
        double error = degrees;
        while (opModeIsActive() && Math.abs(error) > 2.0) {
            double motorPower = (error < 0) ? -0.3 : 0.3;
            robot.setDrivePower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("Error", String.format(Locale.ENGLISH, "%.2f", error));
        }
        robot.setAllDrivePower(0);
    }

    public void turnTo(double degrees) {
        double yaw = robot.getRobotYaw();
        double error = degrees - yaw;
        error = normalizeAngle(error);
        turn(error);
    }
}
