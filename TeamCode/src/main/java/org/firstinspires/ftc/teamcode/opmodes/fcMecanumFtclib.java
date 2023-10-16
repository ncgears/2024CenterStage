package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnController;

@TeleOp(name="FC Mecanum HW", group="JRB")
public class fcMecanumFtclib extends OpMode {
    hwMecanumFtclib robot = new hwMecanumFtclib(this);
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime elapsed = new ElapsedTime();
    Constants.Commands m_last_command = Constants.Commands.NONE;
    double m_last_command_time = 0.0;

    double drive_fwd, drive_strafe, drive_turn = 0.0; //used for holding requested drive values
    double pid_drive_target = 0; //target ticks for pid drive
    double pid_turn_target = 0; //target degrees for pid turn
    boolean pid_driving, pid_turning = false; //tracking if we are using these pid controllers

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // repeatedly until driver presses play
    @Override
    public void init_loop() {
        if(m_last_command == Constants.Commands.NONE && robot.alliance == Constants.Alliance.NONE) runCommand(Constants.Commands.DETERMINE_TEAM); //determine team
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK)) runCommand(Constants.Commands.GYRO_RESET); //listen for gyro reset request
        if(m_last_command != Constants.Commands.NONE && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            runCommand(Constants.Commands.NONE);
        }
        // Update all telemetry data
        telemetry.addData("Alliance", robot.alliance.toString());
        telemetry.addData("Last Command", m_last_command.toString());
        telemetry.addData("Robot Heading", "%.2f", robot.getRobotYaw());
    }

    // driver presses start
    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        drive_fwd = robot.driverOp.getLeftY();
        drive_strafe = robot.driverOp.getLeftX();
        drive_turn = robot.driverOp.getRightX();

        //perform the drive move if we arent turning using PID
        if(!pid_turning) robot.drive.driveFieldCentric(drive_strafe, drive_fwd, drive_turn, robot.getRobotYaw());

        if(robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //left, but not up or down
            turnToPID(-90);
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //right, but not up or down
            turnToPID(90);
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //up, but not left or right
            turnToPID(0);
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //down, but not left or right
            turnToPID(180);
        }

        if(robot.driverOp.getButton(GamepadKeys.Button.BACK)) {
            runCommand(Constants.Commands.GYRO_RESET);
        }

        // Update all telemetry data
        telemetry.addData("Alliance", robot.alliance.toString());
        telemetry.addData("OpMode", "Run Time: %.2f", runtime.seconds());
        telemetry.addData("Last Command", m_last_command.toString());
        telemetry.addData("Robot Heading", "%.2f", robot.getRobotYaw());
        telemetry.addData("Robot Drive", "fwd=%.2f, str=%.2f, turn=%.2f", drive_fwd, drive_strafe, drive_turn);
//        if(pid_driving) telemetry.addData("PID Drive", "target=%.0f, error=%.0f", pid_drive_target, drivepid.getLastError());
//        if(pid_turning) telemetry.addData("PID Turn", "target=%.0f, error=%.0f", pid_turn_target, turnpid.getLastError());
        //telemetry.update(); //this is called automatically every loop
    }

    public void runCommand(Constants.Commands command) {
        m_last_command = command;
        m_last_command_time = runtime.seconds();
        switch (command) {
            case GYRO_RESET:
                robot.imu.resetYaw();
                break;
            case DETERMINE_TEAM:
                robot.alliance = robot.determineAlliance();
                break;
            case ROBOT_RESET:
                break;
            case NONE:
            default:
        }
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + robot.getRobotYaw());
    }

    public void turnToPID(double targetAngle) {
        pid_turning = true;
        pidTurnController pid = new pidTurnController(this, targetAngle, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD);
        while (Math.abs(targetAngle - robot.getRobotYaw()) > Constants.Drivetrain.turnController.targetThreshold) {
            double motorPower = pid.update(robot.getRobotYaw());
            telemetry.addData("Power", "%.2f", motorPower);
            telemetry.update();
            robot.drive.driveFieldCentric(0, 0, motorPower, robot.getRobotYaw());
        }
        pid_turning = false;
    }
}
