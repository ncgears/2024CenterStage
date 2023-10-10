package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name="Test1", group="JRB")
public class autonTest1 extends OpMode {
    hwMecanumFtclib robot = new hwMecanumFtclib(this);
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime elapsed = new ElapsedTime();
    AtomicBoolean check1 = new AtomicBoolean(false);
    AtomicBoolean check2 = new AtomicBoolean(true);
    StateMachine machine = null;
    Constants.Commands m_last_command = Constants.Commands.NONE;
    double m_last_command_time = 0.0;

    double drive_fwd, drive_strafe, drive_turn = 0.0; //used for holding requested drive values
    double pid_drive_target = 0; //target ticks for pid drive
    double pid_turn_target = 0; //target degrees for pid turn
    boolean pid_driving, pid_turning = false; //tracking if we are using these pid controllers
    pidDriveController2 drivepid = new pidDriveController2(this, pid_drive_target, Constants.Drivetrain.driveController.kP, Constants.Drivetrain.driveController.kI, Constants.Drivetrain.driveController.kD);
    pidTurnController2 turnpid = new pidTurnController2(this, pid_turn_target, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD);

    enum States { DRIVING, TURNING, RESTING }
    // driver presses init
    @Override
    public void init() {
        robot.init(hardwareMap);
        machine = new StateMachineBuilder()
            .state(States.DRIVING) //create state
            .onEnter( () -> { //actions to perform when entering state
                elapsed.reset();
                pid_driving = true;
                driveInchesPID(20);
                check1.set(true);
            })
            .transition( () -> (pid_driving && (drivepid.atTarget() || elapsed.seconds() > 5)), States.TURNING ) // transition when condition check1 == true
            .onExit( () -> { //actions to perform when exiting state
                pid_driving = false;
                check2.set(false); // set check2 to false
            })
            .state(States.TURNING) //create state
            .onEnter( () -> { //actions to perform when entering state
                elapsed.reset();
                pid_turning = true;
                turnPID(90);
            })
            .onExit( () -> { //actions to perform when exiting state
                pid_turning = false;
            })
            .transition( () -> (pid_turning && (turnpid.atTarget() || elapsed.seconds() > 5)), States.RESTING ) // transition when condition check2 == false
            .state(States.RESTING) //create state
            .onEnter( () -> { //actions to perform when entering state
                pid_driving = false;
                pid_turning = false;
            })
            .build();
    }

    // repeatedly until driver presses play
    @Override
    public void init_loop() {
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK)) runCommand(Constants.Commands.GYRO_RESET);
        if(m_last_command != Constants.Commands.NONE && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            runCommand(Constants.Commands.NONE);
        }
        // Update all telemetry data
        telemetry.addData("Last Command", m_last_command.toString());
        telemetry.addData("Robot Heading", "%.2f", robot.getRobotYaw());
    }

    // driver presses start
    @Override
    public void start() {
        runtime.reset();
        machine.start();
    }

    // repeatedly until driver presses stop or interrupted
    @Override
    public void loop() {
        machine.update();

        // Monitor for buttons
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK)) runCommand(Constants.Commands.GYRO_RESET);

        // PID Driving
        drivepid.setTarget(pid_drive_target);
        turnpid.setTarget(pid_turn_target);
        drive_fwd = (pid_driving) ? drivepid.update(robot.getDriveAvgPosition()) : 0.0;
        drive_strafe = 0.0;
        drive_turn = (pid_turning) ? turnpid.update(robot.getRobotYaw()) : 0.0;
        autonDrive(drive_fwd, 0, drive_turn, robot.getRobotYaw());

        if(m_last_command != Constants.Commands.NONE && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            runCommand(Constants.Commands.NONE);
        }
        // Update all telemetry data
        telemetry.addData("OpMode", "Run Time: %.2f", runtime.seconds());
        telemetry.addData("Last Command", m_last_command.toString());
        telemetry.addData("Robot State", machine.getState().toString());
        telemetry.addData("Robot Heading", "%.2f", robot.getRobotYaw());
        telemetry.addData("Robot Drive", "fwd=%.2f, str=%.2f, turn=%.2f", drive_fwd, drive_strafe, drive_turn);
        if(pid_driving) telemetry.addData("PID Drive", "target=%.0f, error=%.0f", pid_drive_target, drivepid.getLastError());
        if(pid_turning) telemetry.addData("PID Turn", "target=%.0f, error=%.0f", pid_turn_target, turnpid.getLastError());
        //telemetry.update(); //this is called automatically every loop
    }

    // driver presses stop
    @Override
    public void stop() {
        runCommand(Constants.Commands.ROBOT_RESET);
    }

    public void teleopDrive() {
        robot.drive.driveFieldCentric(
                robot.driverOp.getLeftX(),
                robot.driverOp.getLeftY(),
                robot.driverOp.getRightX(),
                robot.getRobotYaw()
        );
    }

    public void autonDrive(double fwd, double strafe, double turn, double headingDegrees) {
        robot.drive.driveFieldCentric(fwd,strafe,turn,headingDegrees);
    }

    public void runCommand(Constants.Commands command) {
        m_last_command = command;
        m_last_command_time = runtime.seconds();
        switch (command) {
            case GYRO_RESET:
                robot.imu.resetYaw();
                break;
            case ROBOT_RESET:
                check1.set(false);
                check2.set(true);
                break;
            case NONE:
            default:
        }
    }

    public void turnPID(double degrees) {
        turnToPID(degrees + robot.getRobotYaw());
    }

    public void turnToPID(double targetAngle) {
        pid_turn_target = turnpid.update(robot.getRobotYaw());
    }

    public void driveInchesPID(double targetInches) {
        pid_drive_target = targetInches * Constants.Drivetrain.ticksPerInch + robot.getDriveAvgPosition();
    }

}
