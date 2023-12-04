package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.hwMecanumFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidDriveControllerFtclib;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidElevatorController;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTiltController;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnControllerFtclib;
import org.firstinspires.ftc.teamcode.processors.tseSaturationProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="TEST Short Auton 2", group="JRB")
//@Disabled
public class
autonShortAuton3 extends OpMode {
    boolean m_long_auton = false; //set true if this is the long auton
    hwMecanumFtclib robot = new hwMecanumFtclib(this);
    ElapsedTime runtime = new ElapsedTime();
    ElapsedTime elapsed = new ElapsedTime();
    StateMachine machine = null;
    Constants.Manipulator.Positions m_manip_pos = Constants.Manipulator.Positions.START;
    Constants.Commands m_last_command = Constants.Commands.NONE;
    double m_last_command_time = 0.0;
    double m_turn_multiplier = 1.0;

    double drive_fwd, drive_strafe, drive_turn = 0.0; //used for holding requested drive values
    double pid_drive_target = 0; //target ticks for pid drive
    double pid_turn_target = 0; //target degrees for pid turn
    boolean pid_driving, pid_turning = false; //tracking if we are using these pid controllers
    pidDriveControllerFtclib drivepid = new pidDriveControllerFtclib(this, pid_drive_target, Constants.Drivetrain.driveController.kP, Constants.Drivetrain.driveController.kI, Constants.Drivetrain.driveController.kD, Constants.Drivetrain.driveController.kF, Constants.Drivetrain.driveController.kIZone);
    pidTurnControllerFtclib turnpid = new pidTurnControllerFtclib(this, pid_turn_target, Constants.Drivetrain.turnController.kP, Constants.Drivetrain.turnController.kI, Constants.Drivetrain.turnController.kD, Constants.Drivetrain.turnController.kF, Constants.Drivetrain.turnController.kIZone);
    pidTiltController tiltpid = new pidTiltController(this, m_manip_pos.getTilt(), Constants.Manipulator.tiltController.kP, Constants.Manipulator.tiltController.kI, Constants.Manipulator.tiltController.kD, Constants.Manipulator.tiltController.kF, Constants.Manipulator.tiltController.kIZone);
    pidElevatorController elevpid = new pidElevatorController(this, m_manip_pos.getElevator(), Constants.Manipulator.elevatorController.kP, Constants.Manipulator.elevatorController.kI, Constants.Manipulator.elevatorController.kD, Constants.Manipulator.elevatorController.kF, Constants.Manipulator.elevatorController.kIZone);

    private tseSaturationProcessor visionProcessor;
    private VisionPortal visionPortal;
    boolean searchTSE = false;
    boolean strafing = false;
    tseSaturationProcessor.Selected m_tse = tseSaturationProcessor.Selected.NONE;

    boolean isDone_pixel1, isDone_pixel2, isDone_pixel3 = false;

    // States for the finite state machine
    enum States {
        DETERMINE_TEAM, //Determine Alliance and set robot.alliance
        FIND_TSE, //Find the TSE and set m_tse
        MANIP_TRANSPORT, //Manipulator to the TRANSPORT position
        DRIVE_PIXEL1, //Drive to the MIDDLE pixel
        DROP_PIXEL1, //Drop the MIDDLE pixel (conditionally)
        DRIVE_SPIKE, //Drive to the center of the spike
        DECIDE_PIXEL1,
        TURN_PIXEL1,
        DRIVE_SPIKE2,
        TURN_GOAL, //Turn toward Backstage (alliance specific)
        DECIDE_PIXEL2,
        DRIVE_PIXEL2, //Backward to the 2nd Pixel
        DROP_PIXEL2, //Drop the RIGHT pixel (conditionally)
        DRIVE_PIXEL3, //Forward to the 3rd Pixel
        DROP_PIXEL3, //Drop the LEFT pixel (conditionally)
        TURN_GOAL2,
        DRIVE_MIDPOINT, //Drive to the midpoint
        MANIP_SCORE, //Manipulator to SCORE_AUTON position
        DRIVE_GOAL, //Drive to the backstage
        MANIP_DROP, //Manipulator to SCORE_AUTONDROP position
        CLEAR_GOAL,
        STRAFE_CLEAR, //Clear the backstage by moving left/right (alliance specific)
        RESTING //Doing nothing
    }

    // driver presses init
    @Override
    public void init() {
        robot.init(hardwareMap);
        visionProcessor = new tseSaturationProcessor();
        try {
//            visionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Webcam 1"), visionProcessor);
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(visionProcessor)
                    .setCameraResolution(new Size(800, 600))
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();
        } catch (Exception e) {
        }

        machine = new StateMachineBuilder()
                .state(States.DETERMINE_TEAM) //determines if we are red alliance/blue alliance
                .onEnter( () -> {
                    if(robot.alliance == Constants.Alliance.NONE) runCommand(Constants.Commands.DETERMINE_TEAM);
                })
                .onExit( () -> {
                    m_turn_multiplier = (robot.alliance == Constants.Alliance.RED) ? -1.0 : 1.0; //If red alliance, turns are reversed
                    robot.playAudio(String.format("%s", robot.alliance.toString()),500);
                })
//                .transitionWithPointerState( () -> (robot.alliance != Constants.Alliance.NONE), States.STRAFE_CLEAR) //TODO: Comment this
                .transition( () -> (robot.alliance != Constants.Alliance.NONE))
                .state(States.FIND_TSE) //Finds the team supplied element
                .onEnter( () -> {
                    visionPortal.resumeStreaming();
                    //The TSE processor continuously tries to set the selected window
                    searchTSE = true;
                    m_tse = visionProcessor.getSelection();
                    if(m_tse != tseSaturationProcessor.Selected.NONE) { //Unknown TSE location, try to find it
                        telemetry.addLine("Vision Processor identified TSE");
                    }
                })
                .onExit( () -> {
                    searchTSE = false;
                    visionPortal.stopStreaming(); //stop streaming once we we know TSE location
                    robot.playAudio(String.format("%s",m_tse.toString()),500);
                })
//                .transitionWithPointerState( () -> (m_tse != tseSaturationProcessor.Selected.NONE), States.DRIVE_SPIKE) //TODO: Comment this
                .transition( () -> (m_tse != tseSaturationProcessor.Selected.NONE))
                .state(States.MANIP_TRANSPORT)
                .onEnter( () -> {
                    m_manip_pos = Constants.Manipulator.Positions.TRANSPORT;
                })
                .transition(() -> (true))
                .state(States.DRIVE_SPIKE) //create state
                .onEnter( () -> { //actions to perform when entering state
                    elapsed.reset();
                    double distance = (m_tse != tseSaturationProcessor.Selected.MIDDLE) ? 25 : 22;
                    driveInchesPID(distance);
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_driving = false;
                })
//                .transitionWithPointerState( () -> ((pid_driving && drivepid.atTarget()) && m_tse != tseSaturationProcessor.Selected.MIDDLE), States.TURN_GOAL)
                .transition( () -> (pid_driving && drivepid.atTarget()) )
                .state(States.DECIDE_PIXEL1)
                .transitionWithPointerState( () -> (m_tse != tseSaturationProcessor.Selected.MIDDLE), States.TURN_GOAL)
                .transition( () -> (true) )
                .state(States.TURN_PIXEL1) //create state
                .onEnter( () -> { //actions to perform when entering state
                    elapsed.reset();
                    robot.playAudio("Turn pixel 1",500);
                    turnToPID((180 * m_turn_multiplier));
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_turning = false;
                })
//                .transitionWithPointerState( () -> (pid_turning && turnpid.atTarget(robot.getRobotYaw())), States.RESTING) //TODO: Comment this
                .transition( () -> (elapsed.seconds() >= 3.0 && pid_turning && turnpid.atTarget(robot.getRobotYaw())) )
                .transitionTimed(4.0)
                .state(States.DRIVE_PIXEL1) //create state
                .onEnter( () -> { //actions to perform when entering state
//                    robot.playAudio("Drive Spike",500);
                    elapsed.reset();
                    driveInchesPID(-8);
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_driving = false;
                })
//                .transitionWithPointerState( () -> (pid_driving && drivepid.atTarget()), States.RESTING) //TODO: Comment this
                .transition( () -> (pid_driving && drivepid.atTarget()) )
                .state(States.DROP_PIXEL1)
                .onEnter( () -> {
                    if(m_tse == tseSaturationProcessor.Selected.MIDDLE) {
                        robot.playAudio("Drop Pixel", 500);
                        robot.setPixelPosition(Constants.PixelDropper.Positions.UP);
                    } else isDone_pixel1 = true;
                })
                .transition( () -> (isDone_pixel1))
                .transitionTimed(0.3)
                .state(States.DRIVE_SPIKE2) //create state
                .onEnter( () -> { //actions to perform when entering state
//                    robot.playAudio("Drive Spike",500);
                    elapsed.reset();
                    driveInchesPID(3.75);
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_driving = false;
                })
//                .transitionWithPointerState( () -> (pid_driving && drivepid.atTarget()), States.RESTING) //TODO: Comment this
                .transition( () -> (pid_driving && drivepid.atTarget()) )
                .state(States.TURN_GOAL) //create state
                .onEnter( () -> { //actions to perform when entering state
                    elapsed.reset();
                    robot.playAudio("Turn backstage",500);
                    turnToPID((90 * m_turn_multiplier));
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_turning = false;
                })
//                .transitionWithPointerState( () -> (pid_turning && turnpid.atTarget(robot.getRobotYaw())), States.RESTING) //TODO: Comment this
                .transition( () -> (elapsed.seconds() >= 2.5 && pid_turning && turnpid.atTarget(robot.getRobotYaw())) )
                .transitionTimed(3.0)
                .state(States.DECIDE_PIXEL2)
                .transitionWithPointerState( () -> ((robot.alliance == Constants.Alliance.RED && m_tse == tseSaturationProcessor.Selected.LEFT) || (robot.alliance == Constants.Alliance.BLUE && m_tse == tseSaturationProcessor.Selected.RIGHT)), States.DRIVE_PIXEL2)
                .transitionWithPointerState( () -> (true), States.DRIVE_PIXEL3)
                .state(States.DRIVE_PIXEL2) //create state
                .onEnter( () -> { //actions to perform when entering state
                    elapsed.reset();
                    robot.playAudio("Drive Pixel 2",500);
                    driveInchesPID(-8);
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_driving = false;
                })
//                .transitionWithPointerState( () -> (pid_driving && drivepid.atTarget()), States.RESTING) //TODO: Comment this
                .transition( () -> (pid_driving && drivepid.atTarget()) )
                .state(States.DROP_PIXEL2)
                .onEnter( () -> {
                    if((robot.alliance == Constants.Alliance.BLUE && m_tse == tseSaturationProcessor.Selected.RIGHT)
                    || (robot.alliance == Constants.Alliance.RED && m_tse == tseSaturationProcessor.Selected.LEFT)) {
                        robot.playAudio("Drop Pixel", 500);
                        robot.setPixelPosition(Constants.PixelDropper.Positions.UP);
                    } else isDone_pixel2 = true;
                })
                .transition( () -> (isDone_pixel2))
                .transitionTimed(0.3)
                .state(States.DRIVE_PIXEL3) //create state
                .onEnter( () -> { //actions to perform when entering state
                    robot.playAudio("Drive Pixel 3",500);
                    elapsed.reset();
                    if((robot.alliance == Constants.Alliance.BLUE && m_tse == tseSaturationProcessor.Selected.RIGHT)
                    || (robot.alliance == Constants.Alliance.RED && m_tse == tseSaturationProcessor.Selected.LEFT)) {
                        driveInchesPID(15);
                    } else {
                        driveInchesPID(10);
                    }
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_driving = false;
                })
//                .transitionWithPointerState( () -> (pid_driving && drivepid.atTarget()), States.RESTING) //TODO: Comment this
                .transition( () -> (pid_driving && drivepid.atTarget()) )
                .state(States.DROP_PIXEL3)
                .onEnter( () -> {
                    elapsed.reset();
                    if((robot.alliance == Constants.Alliance.BLUE && m_tse == tseSaturationProcessor.Selected.LEFT)
                            || (robot.alliance == Constants.Alliance.RED && m_tse == tseSaturationProcessor.Selected.RIGHT)) {
                        robot.playAudio("Drop Pixel", 500);
                        robot.setPixelPosition(Constants.PixelDropper.Positions.UP);
                    } else isDone_pixel2 = true;
                })
                .transition( () -> (isDone_pixel3))
                .transitionTimed(0.3)
                .state(States.DRIVE_MIDPOINT) //create state
                .onEnter( () -> { //actions to perform when entering state
                    robot.playAudio("Drive midpoint",500);
                    elapsed.reset();
                    double distance = 8;
                    distance += (m_long_auton) ? 48 : 0; //if a long auton, add another 48 inches here
                    driveInchesPID(distance); //drive past the bars
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_driving = false;
                })
                .transition( () -> (pid_driving && drivepid.atTarget()) )
                .state(States.TURN_GOAL2) //create state
                .onEnter( () -> { //actions to perform when entering state
                    elapsed.reset();
                    robot.playAudio("Turn backstage",500);
                    turnToPID((90 * m_turn_multiplier));
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_turning = false;
                })
                .transition( () -> (elapsed.seconds() >= 1.5 && pid_turning && turnpid.atTarget(robot.getRobotYaw())) )
                .transitionTimed(2.0)
                .state(States.MANIP_SCORE)
                .onEnter( () -> {
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_AUTO;
                })
                .transitionTimed(1.5)
                .state(States.DRIVE_GOAL) //create state
                .onEnter( () -> { //actions to perform when entering state
                    robot.playAudio("Drive to backstage",500);
                    elapsed.reset();
                    driveInchesPID(6); //drive to the goal
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_driving = false;
                })
                .transition( () -> (robot.getDistance() <= 6.0 || pid_driving && drivepid.atTarget()) )
                .state(States.MANIP_DROP)
                .onEnter( () -> {
                    robot.playAudio("Score Pixels",500);
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_AUTODROP;
                })
                .onExit( () -> {
                    m_manip_pos = Constants.Manipulator.Positions.SCORE_AUTO;
                })
                .transitionTimed(1.5)
                .state(States.CLEAR_GOAL) //create state
                .onEnter( () -> { //actions to perform when entering state
                    elapsed.reset();
                    robot.playAudio("Get out the way",500);
                    driveInchesPID(-6); //drive to the goal
                })
                .onExit( () -> { //actions to perform when exiting state
                    pid_driving = false;
                })
                .transitionWithPointerState( () -> (m_long_auton), States.RESTING)
                .transition( () -> (pid_driving && drivepid.atTarget()) )
                .state(States.STRAFE_CLEAR)
                .onEnter( () -> {
                    strafing = true;
                })
                .onExit( () -> {
                    strafing = false;
                })
                .transitionTimed(1.5)
                .state(States.RESTING) //create state
                .onEnter( () -> { //actions to perform when entering state
                    pid_driving = false;
                    pid_turning = false;
                    strafing = false;
                    robot.playAudio("Resting",500);
                })
                .build();
    }

    // repeatedly until driver presses play
    @Override
    public void init_loop() {
        if(m_last_command == Constants.Commands.NONE || robot.alliance == Constants.Alliance.NONE) runCommand(Constants.Commands.DETERMINE_TEAM); //determine team
        if(robot.driverOp.getButton(GamepadKeys.Button.BACK) && runtime.seconds() - m_last_command_time > 1) runCommand(Constants.Commands.GYRO_RESET); //listen for gyro reset request
        if(robot.driverOp.getButton(GamepadKeys.Button.START) && runtime.seconds() - m_last_command_time > 1) runCommand(Constants.Commands.TOGGLE_PIXEL);
        if(runtime.seconds() - m_last_command_time > 2.0) { //reset imu every 2 seconds during init
            robot.imu.resetYaw();
        }
        if(m_last_command != Constants.Commands.NONE && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            runCommand(Constants.Commands.NONE);
        }
        // Update all telemetry data
        telem(true);
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

        // Monitor for TSE
        if(searchTSE) m_tse = visionProcessor.getSelection();

        // PID Driving
        drivepid.setTarget(pid_drive_target);
        turnpid.setTarget(pid_turn_target);
        drive_fwd = (pid_driving) ? drivepid.update(robot.getDriveAvgPosition()) : 0.0;
        drive_strafe = (strafing) ? -Constants.Auton.autonStrafeSpeed * m_turn_multiplier : 0.0;
        drive_turn = (pid_turning) ? -turnpid.update(robot.getRobotYaw()) : 0.0;
        autonDrive(drive_fwd, drive_strafe, drive_turn, robot.getRobotYaw());

//        if (pid_turning && turnpid.atTarget(robot.getRobotYaw())) pid_turning = false;
//        if (pid_driving && drivepid.atTarget()) pid_driving = false;

        if(m_last_command != Constants.Commands.NONE && runtime.seconds() - m_last_command_time > 2) { //reset the last command after 2 seconds
            runCommand(Constants.Commands.NONE);
        }

        //handle moving the manipulator
        moveElevator();
        moveTilt();

        // Update all telemetry data
        telem(false);
    }

    // driver presses stop
    @Override
    public void stop() {
        runCommand(Constants.Commands.ROBOT_RESET);
    }

    public void autonDrive(double fwd, double strafe, double turn, double headingDegrees) {
//        robot.drive.driveFieldCentric(strafe,fwd,turn,headingDegrees);
        robot.drive.driveRobotCentric(strafe,fwd,turn);
    }

    public boolean isPidDriving() { return pid_driving; }
    public boolean isPidTurning() { return pid_turning; }

    public void runCommand(Constants.Commands command) {
        m_last_command = command;
        m_last_command_time = runtime.seconds();
        switch (command) {
            case GYRO_RESET:
                robot.imu.resetYaw();
                break;
            case TOGGLE_PIXEL:
                if(robot.getPixelPosition() == Constants.PixelDropper.Positions.DOWN) {
                    robot.setPixelPosition(Constants.PixelDropper.Positions.UP);
                } else {
                    robot.setPixelPosition(Constants.PixelDropper.Positions.DOWN);
                }
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
        pid_turn_target = targetAngle;
    }

    public void driveInchesPID(double targetInches) {
        pid_driving = true;
        robot.resetAllDriveEncoder();
        pid_drive_target = targetInches * Constants.Drivetrain.driveController.ticksPerInch + robot.getDriveAvgPosition();
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
        telemetry.addData("TSE Location", m_tse.toString());
        telemetry.addData("Pixel Dropper", robot.getPixelPosition().toString());
        telemetry.addData("Manipulator Position", m_manip_pos.toString());
        telemetry.addData("Auton State", machine.getState().toString());
        robot.getDriveAvgPosition(); //put the encoder data on the telem
        telemetry.addData("Tilt", "lim=%s, tgt=%.0f, pos=%d, pwr=%.2f", robot.getTiltLimitString(), tiltpid.getTarget(), robot.getTiltPosition(), robot.getTiltPower());
        telemetry.addData("Elev", "lim=%s, tgt=%.0f, pos=%d, pwr=%.2f", robot.getElevatorLimitString(), elevpid.getTarget(), robot.getElevatorPosition(), robot.getElevatorPower());
        if(idle) { //items that are only in idle
        } else {
            telemetry.addData("OpMode", "Run Time: %.2f", runtime.seconds());
            telemetry.addData("Robot Drive", "fwd=%.2f, str=%.2f, turn=%.2f", drive_fwd, drive_strafe, drive_turn);
            if(pid_driving) telemetry.addData("PID Drive", "target=%.0f, error=%.0f", pid_drive_target, drivepid.getLastError());
            if(pid_turning) telemetry.addData("PID Turn", "target=%.0f, error=%.0f", pid_turn_target, turnpid.getLastError());
        }
        //telemetry.update(); //this is called automatically every loop
    }
}
