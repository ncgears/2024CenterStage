package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnController;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnControllerFtclib;

@TeleOp(name="FC Mecanum HW", group="JRB")
public class fcMecanumFtclib extends OpMode {
    hwMecanumFtclib robot = new hwMecanumFtclib(this);
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime elapsed = new ElapsedTime();
    Constants.Commands m_last_command = Constants.Commands.NONE;
    double m_last_command_time = 0.0;

    double drive_fwd, drive_strafe, drive_turn = 0.0; //used for holding requested drive values
    double pid_turn_target = 0; //target degrees for pid turn
    boolean pid_turning = false; //tracking if we are using these pid controllers
    pidTurnControllerFtclib turnpid = new pidTurnControllerFtclib(this, pid_turn_target, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD);

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
//        drive_fwd = (pid_turning) ? 0.0 : robot.driverOp.getLeftY(); //if pid turning, no throttle
//        drive_strafe = (pid_turning) ? 0.0 : robot.driverOp.getLeftX(); //if pid turning, no strafing
//        drive_turn = (pid_turning) ? turnpid.update(robot.getRobotYaw()) : 0.0;
        drive_fwd = robot.driverOp.getLeftY();
        drive_strafe = robot.driverOp.getLeftX();
        drive_turn = robot.driverOp.getRightX();
        if(Math.abs(drive_turn) > 0.1) pid_turning = false; //if we attempted to turn manually, disable pid turning
        if(pid_turning) { //set new values for joysticks if we requested pid turning
            // update the pid controller
            turnpid.setTarget(pid_turn_target);
            if(turnpid.atTarget(robot.getRobotYaw())) {
                drive_turn = 0.0;
                pid_turning = false;
            } else {
                drive_turn = -turnpid.update(robot.getRobotYaw());
            }
        }
        // perform the drive
        robot.drive.driveFieldCentric(drive_strafe, drive_fwd, drive_turn, robot.getRobotYaw());

        if(robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //left, but not up or down
            runCommand(Constants.Commands.PID_TURN_90);
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //right, but not up or down
            runCommand(Constants.Commands.PID_TURN_N90);
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //up, but not left or right
            runCommand(Constants.Commands.PID_TURN_0);
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //down, but not left or right
            runCommand(Constants.Commands.PID_TURN_180);
        }

        if(robot.driverOp.getButton(GamepadKeys.Button.BACK)) {
            runCommand(Constants.Commands.GYRO_RESET);
        }

        if(m_last_command != Constants.Commands.NONE && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            runCommand(Constants.Commands.NONE);
        }
        // Update all telemetry data
        telemetry.addData("Alliance", robot.alliance.toString());
        telemetry.addData("OpMode", "Run Time: %.2f", runtime.seconds());
        telemetry.addData("Last Command", m_last_command.toString());
        telemetry.addData("Robot Heading", "%.2f", robot.getRobotYaw());
        telemetry.addData("Robot Drive", "fwd=%.2f, str=%.2f, turn=%.2f", drive_fwd, drive_strafe, drive_turn);
//        if(pid_driving) telemetry.addData("PID Drive", "target=%.0f, error=%.0f", pid_drive_target, drivepid.getLastError());
        if(pid_turning) telemetry.addData("PID Turn", "target=%.0f, error=%.0f", pid_turn_target, turnpid.getLastError());
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
            case PID_TURN_0:
                turnToPID(0);
                break;
            case PID_TURN_90:
                turnToPID(90);
                break;
            case PID_TURN_N90:
                turnToPID(-90);
                break;
            case PID_TURN_180:
                turnToPID(180);
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
        pid_turn_target = targetAngle;
//        pid_turning = true;
//        pidTurnController pid = new pidTurnController(this, targetAngle, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD);
//        while (Math.abs(targetAngle - robot.getRobotYaw()) > Constants.Drivetrain.turnController.targetThreshold) {
//            double turnPower = -pid.update(robot.getRobotYaw());
//            telemetry.addData("Power", "%.2f", turnPower);
//            telemetry.update();
//            robot.drive.driveFieldCentric(0, 0, turnPower, robot.getRobotYaw());
//        }
//        pid_turning = false;
    }
}
