package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Gyro Test", group="JRB")
public class autonGyroTest extends LinearOpMode {
    hwMecanum robot = new hwMecanum(this);
    private ElapsedTime runtime = new ElapsedTime();
    private double lastAngle = 0.0;
    private double currAngle = 0.0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();

        robot.imu.resetYaw();
        sleep(50);

        turnPID(90);  //left
        sleep(3000);
        turnToPID(-90); //180deg from prev turn

//        telemetry.update();
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
        telemetry.addData("Gyro", "%.2f",yaw);
        telemetry.update();
        return currAngle;
    }

    public double normalizeAngle(double angle) {
        if (angle > 180) return angle - 360;
        if (angle < -180) return angle + 360;
        return angle;
    }

    public void turn(double degrees) {
        telemetry.addData("Action", "Turn %.2f degrees", degrees);
//        telemetry.update();
        resetAngle();
        double error = degrees;
        while (opModeIsActive() && Math.abs(error) > Constants.Drivetrain.turnController.targetThreshold) {
            double motorPower = Constants.Auton.autonDriveSpeed * Math.signum(error);
            robot.setDrivePower(-motorPower, motorPower, -motorPower, motorPower);
            error = degrees - getAngle();
            telemetry.addData("Error", "%.2f", error);
        }
        robot.setAllDrivePower(0); //make sure it stops when we get to target
    }

    public void turnTo(double degrees) {
        telemetry.addData("Action", "Turn to %.2f degrees", degrees);
//        telemetry.update();
        double yaw = robot.getRobotYaw();
        double error = degrees - yaw;
        error = normalizeAngle(error);
        turn(error);
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + robot.getRobotYaw());
    }

    public void turnToPID(double targetAngle) {
        pidTurnController pid = new pidTurnController(targetAngle, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD);
        while (opModeIsActive() && Math.abs(targetAngle - robot.getRobotYaw()) > Constants.Drivetrain.turnController.targetThreshold) {
            double motorPower = pid.update(robot.getRobotYaw());
            telemetry.addData("Power", "%.2f", motorPower);
            telemetry.update();
            robot.setDrivePower(motorPower, -motorPower, motorPower, -motorPower);
        }
        robot.setAllDrivePower(0); //make sure it stops when we get to target
    }
}
