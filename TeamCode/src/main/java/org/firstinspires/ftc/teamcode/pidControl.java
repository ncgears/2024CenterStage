package org.firstinspires.ftc.teamcode;

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

@TeleOp(name="PID Control", group="Jim")
public class pidControl extends OpMode {
    DcMotor fL, fR, rL, rR   = null;
    MecanumDrive drive = null;
    GamepadEx driverOp = null;
    Motor m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr = null;
    IMU imu = null;

	// PID Tuning
	double integralSum, kP, kI, kD = 0;
	// PID Controller
	ElapsedTime timer = new ElapsedTime();
	private double lastError = 0;


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
        m_motor_fl.setInverted(false);
        m_motor_fr.setInverted(false);
        m_motor_rl.setInverted(true);
        m_motor_rr.setInverted(true);

        // Disable RUN_USING_ENCODER as it limits output to ~80%
        for (Motor m : m_motors) {
            m.setMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
        }

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

        if(driverOp.getButton(GamepadKeys.Button.BACK)) {
            imu.resetYaw();
            telemetry.addData("Robot", "Gyro Reset");
        }

        // Update all telemetry data
        telemetry.addData("Heading", String.format("%.2f",imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        //telemetry.update(); //this is called automatically every loop
    }

    public double PIDControl(double reference, double state) {
		double error = reference - state;
		integralSum += error * timer.seconds();
		double derivative = (error - lastError) / timer.seconds();
		lastError = error;
		timer.reset();
		double output = (error * kP) + (derivative * kD) + (integralSum * kI);
		return output;
	}
}
