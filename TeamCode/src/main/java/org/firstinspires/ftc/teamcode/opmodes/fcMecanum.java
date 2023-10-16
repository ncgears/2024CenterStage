package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.pidcontrollers.pidTurnController;

@TeleOp(name="FC Mecanum", group="JRB")
public class fcMecanum extends OpMode {
    DcMotor fL, fR, rL, rR   = null;
    MecanumDrive drive = null;
    GamepadEx driverOp = null;
    Motor m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr = null;
    IMU imu = null;

    @Override
    public void init() {
        driverOp = new GamepadEx(gamepad1);
        imu = hardwareMap.get(IMU.class, "imu");
        m_motor_fl = new Motor(hardwareMap, "FL DRIVE");
        m_motor_fr = new Motor(hardwareMap, "FR DRIVE");
        m_motor_rl = new Motor(hardwareMap, "RL DRIVE");
        m_motor_rr = new Motor(hardwareMap, "RR DRIVE");
        Motor[] m_motors = {m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr};

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        fL = m_motor_fl.motor;
        fR = m_motor_fr.motor;
        rL = m_motor_rl.motor;
        rR = m_motor_rr.motor;
        drive = new MecanumDrive(m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr);

        // Invert motors as needed so forward is the correct direction
        m_motor_fl.setInverted(true);
        m_motor_fr.setInverted(true);
        m_motor_rl.setInverted(false);
        m_motor_rr.setInverted(false);

        // Set the runmode for each motor
        for (Motor m : m_motors) {
            m.setRunMode(Motor.RunMode.RawPower);
        }

        // Reset the encoders for each motor
        for (Motor m : m_motors) {
            m.resetEncoder();
        }

        // Adjust the orientation parameters of the IMU
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);

        // Define and initialize ALL installed servos.

        telemetry.addData("Robot", "Hardware Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                driverOp.getRightX(),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );

        if(driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //left, but not up or down
        } else if (driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT) && !driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !driverOp.getButton(GamepadKeys.Button.DPAD_UP)) {
            //right, but not up or down
        } else if (driverOp.getButton(GamepadKeys.Button.DPAD_UP) && !driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //up, but not left or right
        } else if (driverOp.getButton(GamepadKeys.Button.DPAD_DOWN) && !driverOp.getButton(GamepadKeys.Button.DPAD_LEFT) && !driverOp.getButton(GamepadKeys.Button.DPAD_RIGHT)) {
            //down, but not left or right
        }

        if(driverOp.getButton(GamepadKeys.Button.BACK)) {
            imu.resetYaw();
            telemetry.addData("Robot", "Gyro Reset");
        }

        // Update all telemetry data
        telemetry.addData("Heading", "%.2f", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        //telemetry.update(); //this is called automatically every loop
    }
}
