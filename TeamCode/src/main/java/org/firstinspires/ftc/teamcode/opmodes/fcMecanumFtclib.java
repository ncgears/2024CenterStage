package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnControllerFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidElevatorController;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTiltController;

@TeleOp(name="FC Mecanum HW", group="JRB")
public class fcMecanumFtclib extends OpMode {
    hwMecanumFtclib robot = new hwMecanumFtclib(this);
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime elapsed = new ElapsedTime();
    Constants.Manipulator.Positions m_manip_pos = Constants.Manipulator.Positions.START;
    Constants.Manipulator.Positions m_last_manip_pos = Constants.Manipulator.Positions.SCORE_ROW1;
    String m_last_command = Constants.Commands.NONE.toString();
    double m_last_command_time = 0.0;

    double drive_fwd, drive_strafe, drive_turn = 0.0; //used for holding requested drive values
    double pid_turn_target = 0; //target degrees for pid turn
    boolean pid_turning = false; //tracking if we are using these pid controllers
    pidTurnControllerFtclib turnpid = new pidTurnControllerFtclib(this, pid_turn_target, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD);
    pidTiltController tiltpid = new pidTiltController(this, m_manip_pos.getTilt(), Constants.Manipulator.tiltController.kP, Constants.Manipulator.tiltController.kI, Constants.Manipulator.tiltController.kD, Constants.Manipulator.tiltController.kF);
    pidElevatorController elevpid = new pidElevatorController(this, m_manip_pos.getElevator(), Constants.Manipulator.elevatorController.kP, Constants.Manipulator.elevatorController.kI, Constants.Manipulator.elevatorController.kD, Constants.Manipulator.elevatorController.kF);

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // repeatedly until driver presses play
    @Override
    public void init_loop() {
        if(m_last_command == "NONE" && robot.alliance == Constants.Alliance.NONE) telemCommand("DETERMINE TEAM"); //determine team and store it
        // always listen for gyro reset button
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK) && runtime.seconds() - m_last_command_time > 0.5) {
            robot.imu.resetYaw();
            telemCommand("RESET GYRO");
        }
        // command name updates for telemetry
        if(m_last_command != "NONE" && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            telemCommand("NONE");
        }
        // Update all telemetry data
        telem(true);
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

        // always listen for gyro reset button
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK)) {
            robot.imu.resetYaw();
            telemCommand("RESET GYRO");
        }

        // automated field-relative turn functions for d-pad
        if(robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //left, but not up or down
            turnToPID(90);
            telemCommand("PID TURN FC 90");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //right, but not up or down
            turnToPID(-90);
            telemCommand("PID TURN FC -90");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //up, but not left or right
            turnToPID(0);
            telemCommand("PID TURN FC 0");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //down, but not left or right
            turnToPID(180);
            telemCommand("PID TURN FC 180");
        }

        // handle manipulator
        if (robot.driverOp.getButton(GamepadKeys.Button.Y)) { //last scoring position
            m_manip_pos = m_last_manip_pos;
            telemCommand("LAST SCORING POSITION");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.X)) { //transport
            m_manip_pos = Constants.Manipulator.Positions.TRANSPORT;
            telemCommand("TRANSPORT POSITION");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.B)) { //transport
            m_manip_pos = Constants.Manipulator.Positions.SCORE_DROP1;
            telemCommand("TRANSPORT POSITION");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.A)) { //floor pickup
            m_manip_pos = Constants.Manipulator.Positions.FLOOR_CLOSE;
            telemCommand("FLOOR PICKUP");
        }

        // Update the manipulator - these should be called every loop to make the manipulator move to target position
        moveElevator();
        moveTilt();

        // command name updates for telemetry
        if(m_last_command != "NONE" && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            telemCommand("NONE");
        }
        // Update all telemetry data
        telem(false);
    }

    public void telemCommand(String command) {
        m_last_command = command;
        m_last_command_time = runtime.seconds();
        switch (command) {
            case "DETERMINE TEAM":
                robot.alliance = robot.determineAlliance();
                break;
            case "RESET ROBOT":
                break;
            case "NONE":
            default:
        }
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + robot.getRobotYaw());
    }

    public void turnToPID(double targetAngle) {
        pid_turning = true;
        pid_turn_target = targetAngle;
    }

    public void moveTilt() {
        tiltpid.setTargetPosition(m_manip_pos);
        double power = tiltpid.update(robot.getTiltPosition());
        robot.setTiltPower(power);
    }

    public void moveElevator() {
        elevpid.setTargetPosition(m_manip_pos);
        double power = elevpid.update(robot.getElevatorPosition());
        robot.setElevatorPower(power);
    }

    private void telem(boolean idle) {
        telemetry.addData("Alliance", robot.alliance.toString());
        telemetry.addData("Last Command", m_last_command.toString());
        telemetry.addData("Robot Heading", "%.2f", robot.getRobotYaw());
        telemetry.addData("Obstacle Distance", "%.2f Inches", robot.getDistance());
        telemetry.addData("Pixel Dropper", robot.getPixelPosition().toString());
        telemetry.addData("Manipulator Position", m_manip_pos.toString());
        telemetry.addData("Tilt", "lim=%s, tgt=%.0f, pos=%d, pwr=%.2f", robot.getTiltLimitString(), tiltpid.getTarget(), robot.getTiltPosition(), robot.getTiltPower());
        telemetry.addData("Elev", "lim=%s, tgt=%.0f, pos=%d, pwr=%.2f", robot.getElevatorLimitString(), elevpid.getTarget(), robot.getElevatorPosition(), robot.getElevatorPower());
        if(idle) { //items that are only in idle
        } else {
            telemetry.addData("OpMode", "Run Time: %.2f", runtime.seconds());
            telemetry.addData("Robot Drive", "fwd=%.2f, str=%.2f, turn=%.2f", drive_fwd, drive_strafe, drive_turn);
//        if(pid_driving) telemetry.addData("PID Drive", "target=%.0f, error=%.0f", pid_drive_target, drivepid.getLastError());
            if(pid_turning) telemetry.addData("PID Turn", "target=%.0f, error=%.0f", pid_turn_target, turnpid.getLastError());
        }
        //telemetry.update(); //this is called automatically every loop
    }

}
