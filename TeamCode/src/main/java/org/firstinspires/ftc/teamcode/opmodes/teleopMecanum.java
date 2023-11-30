package org.firstinspires.ftc.teamcode.opmodes;

//import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnControllerFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidElevatorController;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTiltController;

@TeleOp(name="Mecanum Drive", group="JRB")
public class teleopMecanum extends OpMode {
    hwMecanumFtclib robot = new hwMecanumFtclib(this);
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime elapsed = new ElapsedTime();
    Constants.Manipulator.Positions m_manip_pos = Constants.Manipulator.Positions.START;
    Constants.Manipulator.Positions m_manip_prev_pos = Constants.Manipulator.Positions.START;
    boolean m_manip_momentary = false;
    Constants.Manipulator.Positions m_last_manip_pos = Constants.Manipulator.Positions.SCORE_ROW1;
    boolean m_manip_manual = false;
    String m_last_command = Constants.Commands.NONE.toString();
    double m_last_command_time = 0.0;
    double m_turn_multiplier = 1.0;

    double drive_fwd, drive_strafe, drive_turn = 0.0; //used for holding requested drive values
    double pid_turn_target = 0; //target degrees for pid turn
    boolean pid_turning = false; //tracking if we are using these pid controllers
    pidTurnControllerFtclib turnpid = new pidTurnControllerFtclib(this, pid_turn_target, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD, Constants.Drivetrain.turnController.kF);
    pidTiltController tiltpid = new pidTiltController(this, m_manip_pos.getTilt(), Constants.Manipulator.tiltController.kP, Constants.Manipulator.tiltController.kI, Constants.Manipulator.tiltController.kD, Constants.Manipulator.tiltController.kF);
    pidElevatorController elevpid = new pidElevatorController(this, m_manip_pos.getElevator(), Constants.Manipulator.elevatorController.kP, Constants.Manipulator.elevatorController.kI, Constants.Manipulator.elevatorController.kD, Constants.Manipulator.elevatorController.kF);
    boolean tilt_low_limit = false;


    boolean pressed_rb, pressed_lb, pressed_up, pressed_dn, pressed_lt, pressed_rt = false; //for debouncing button presses

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    // repeatedly until driver presses play
    @Override
    public void init_loop() {
        if(m_last_command == "NONE" || robot.alliance == Constants.Alliance.NONE) telemCommand("DETERMINE TEAM"); //determine team and store it
        // always listen for gyro reset button
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK) && runtime.seconds() - m_last_command_time > 0.5) {
            robot.imu.resetYaw();
            telemCommand("RESET GYRO");
        }
//        // Don't do this while waiting for teleop, robot doesnt get reset between auton and teleop
//        if(runtime.seconds() - m_last_command_time > 2.0) { //reset imu every 2 seconds during init
//            robot.imu.resetYaw();
//        }
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
        m_turn_multiplier = (robot.alliance == Constants.Alliance.RED) ? -1.0 : 1.0;
    }

    @Override
    public void loop() {
//        drive_fwd = (pid_turning) ? 0.0 : robot.driverOp.getLeftY(); //if pid turning, no throttle
//        drive_strafe = (pid_turning) ? 0.0 : robot.driverOp.getLeftX(); //if pid turning, no strafing
//        drive_turn = (pid_turning) ? turnpid.update(robot.getRobotYaw()) : 0.0;
        drive_fwd = robot.driverOp.getLeftY();
        drive_strafe = robot.driverOp.getLeftX();
        drive_turn = robot.driverOp.getRightX();
        if (Math.abs(drive_turn) > 0.1)
            pid_turning = false; //if we attempted to turn manually, disable pid turning
        if (pid_turning) { //set new values for joysticks if we requested pid turning
            // update the pid controller
            turnpid.setTarget(pid_turn_target);
            if (turnpid.atTarget(robot.getRobotYaw())) {
                drive_turn = 0.0;
                pid_turning = false;
            } else {
                drive_turn = -turnpid.update(robot.getRobotYaw());
            }
        }
        // perform the drive
        if (robot.getRobotYaw() == 0.0 || !Constants.Drivetrain.useFieldCentric) { //exactly 0 from the imu is unlikely, fall back to robot centric
            robot.drive.driveRobotCentric(drive_strafe, drive_fwd, drive_turn);
        } else {
            robot.drive.driveFieldCentric(drive_strafe, drive_fwd, drive_turn, robot.getRobotYaw());
        }

        /** Driver Controls */
        // always listen for gyro reset button
        if (robot.driverOp.getButton(GamepadKeys.Button.BACK)) {
            robot.imu.resetYaw();
            telemCommand("RESET GYRO");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.START)) {
            m_manip_pos = Constants.Manipulator.Positions.START;
            m_manip_manual = true;
            try {
                moveElevator();
                wait(3000);
                moveTilt();
            } catch (Exception e) {
            } finally {
                m_manip_manual = false;
            }
            telemCommand("STARTING CONFIG");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.Y)) {
            //up, but not left or right
            turnToPID(0);
            telemCommand("PID TURN FC 0");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.B)) {
            //right, but not up or down
            turnToPID(-90 * m_turn_multiplier);
            telemCommand("PID TURN FC -90");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.A)) {
            //down, but not left or right
            turnToPID(180);
            telemCommand("PID TURN FC 180");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.X)) {
            //left, but not up or down
            turnToPID(90 * m_turn_multiplier);
            telemCommand("PID TURN FC 90");
        }

        // automated field-relative turn functions for d-pad
        if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            m_manip_pos = Constants.Manipulator.Positions.CLIMB_VERT;
            telemCommand("CLIMB VERT");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            m_manip_pos = Constants.Manipulator.Positions.CLIMB_UP;
            telemCommand("CLIMB READY");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_UP) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            m_manip_pos = Constants.Manipulator.Positions.CLIMB_READY;
            telemCommand("CLIMB READY");
        } else if (robot.driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !robot.driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            m_manip_pos = Constants.Manipulator.Positions.CLIMB_LIFT;
            telemCommand("CLIMB LIFT");
        }
        /** End Driver Controls */

        /** Operator Controls */
        if (robot.operOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.5 && m_manip_momentary) { //release scoring button
            m_manip_momentary = false;
            m_manip_pos = m_manip_prev_pos;
            telemCommand("RETURN");
        } else if (robot.operOp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) >= 0.5 && !m_manip_momentary) { //press scoring button
            switch (m_manip_pos) {
                case SCORE_ROW1:
                    m_manip_prev_pos = m_manip_pos;
                    m_manip_momentary = true;
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_DROP1;
                    telemCommand("SCORING POSITION 1");
                    break;
                case SCORE_ROW2:
                    m_manip_prev_pos = m_manip_pos;
                    m_manip_momentary = true;
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_DROP2;
                    telemCommand("SCORING POSITION 2");
                    break;
                case SCORE_ROW3:
                    m_manip_prev_pos = m_manip_pos;
                    m_manip_momentary = true;
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_DROP3;
                    telemCommand("SCORING POSITION 3");
                    break;
                case TRANSPORT:
                    m_manip_prev_pos = m_manip_pos;
                    m_manip_momentary = true;
                    m_manip_pos = Constants.Manipulator.Positions.FLOOR_DESTACK;
                    telemCommand("DESTACK");
                    break;
                default:
            }
        } else if (!pressed_rb && robot.operOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) { //position up
            switch (m_manip_pos) {
                case SCORE_ROW1:
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_ROW2;
                    telemCommand("SCORING POSITION 2");
                    break;
                case SCORE_ROW2:
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_ROW3;
                    telemCommand("SCORING POSITION 3");
                    break;
                case SCORE_ROW3:
//                        telemCommand("NOTHING");
//                        break;
                default:
//                        telemCommand("NOTHING");
            }
        } else if (!pressed_lb && robot.operOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) { //position down
            switch (m_manip_pos) {
                case SCORE_ROW3:
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_ROW2;
                    telemCommand("SCORING POSITION 2");
                    break;
                case SCORE_ROW2:
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_ROW1;
                    telemCommand("SCORING POSITION 1");
                    break;
                case SCORE_ROW1:
//                    telemCommand("NOTHING");
                    break;
                default:
//                    telemCommand("NOTHING");
            }
        } else if (robot.operOp.getButton(GamepadKeys.Button.Y)) { //last scoring position
            m_manip_pos = m_last_manip_pos;
            telemCommand("LAST SCORING POSITION");
        } else if (robot.operOp.getButton(GamepadKeys.Button.X)) { //transport
            m_manip_pos = Constants.Manipulator.Positions.TRANSPORT;
            telemCommand("TRANSPORT POSITION");
        } else if (robot.operOp.getButton(GamepadKeys.Button.A)) { //floor pickup
            m_manip_pos = Constants.Manipulator.Positions.FLOOR_CLOSE;
            telemCommand("FLOOR PICKUP");
        } else if (pressed_rb && !robot.operOp.getButton(GamepadKeys.Button.RIGHT_BUMPER)) { //released the button
            pressed_rb = false;
        } else if (pressed_lb && !robot.operOp.getButton(GamepadKeys.Button.LEFT_BUMPER)) { //released the button
            pressed_lb = false;
        }

        /** DPAD_UP and DPAD_DOWN handles adjusting the tilt offset */
        if (!pressed_up && robot.operOp.getButton(GamepadKeys.Button.DPAD_UP)) { //tilt offset up
            pressed_up = true;
            tiltpid.increaseOffset();
        } else if (!pressed_dn && robot.operOp.getButton(GamepadKeys.Button.DPAD_DOWN)) { //tilt offset down
            pressed_dn = true;
            tiltpid.decreaseOffset();
        } else if (pressed_up && !robot.operOp.getButton(GamepadKeys.Button.DPAD_UP)) { //released the button
            pressed_up = false;
        } else if (pressed_dn && !robot.operOp.getButton(GamepadKeys.Button.DPAD_DOWN)) { //released the button
            pressed_dn = false;
        }

        /** Left and Right DPAD handles adjusting the elevator offset */
        if (!pressed_rt && robot.operOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) { //elev offset up
            pressed_rt = true;
            elevpid.increaseOffset();
        } else if (!pressed_lt && robot.operOp.getButton(GamepadKeys.Button.DPAD_LEFT)) { //elev offset down
            pressed_lt = true;
            elevpid.decreaseOffset();
        } else if (pressed_lt && !robot.operOp.getButton(GamepadKeys.Button.DPAD_LEFT)) { //released the button
            pressed_lt = false;
        } else if (pressed_rt && !robot.operOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) { //released the button
            pressed_rt = false;
        }

//        robot.driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(() -> {
//            switch (m_manip_pos) {
//                case SCORE_ROW1:
//                    m_manip_pos = Constants.Manipulator.Positions.SCORE_ROW2;
//                    telemCommand("SCORING POSITION 2");
//                    break;
//                case SCORE_ROW2:
//                    m_manip_pos = Constants.Manipulator.Positions.SCORE_ROW3;
//                    telemCommand("SCORING POSITION 3");
//                    break;
//                case SCORE_ROW3:
//                    telemCommand("NOTHING");
//                    break;
//                default:
//                    telemCommand("NOTHING");
//            }
//        }));
        /** End Operator Controls */

        // Update the manipulator - these should be called every loop to make the manipulator move to target position
        if(!m_manip_manual) {
            moveElevator();
            moveTilt();
        }

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
        if(tilt_low_limit) {
            tiltpid.setTarget(robot.getTiltPosition());
            tilt_low_limit = false;
        } else {
            tiltpid.setTargetPosition(m_manip_pos);
            double power = tiltpid.update(robot.getTiltPosition());
            if(power < 0 && robot.getTiltLowLimit()) {
                tiltpid.setTarget(robot.getTiltPosition());
                tilt_low_limit = true;
            }
            robot.setTiltPower(power);
        }
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
        telemetry.addData("Drone Launcher", robot.getDronePosition().toString());
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