/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class hwMecanum {

    /* Public OpMode members. */

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor m_motor_fl   = null;
    public DcMotor m_motor_fr  = null;
    public DcMotor m_motor_rl = null;
    public DcMotor m_motor_rr = null;
    public IMU imu = null;


    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    /* Local OpMode members. */
    HardwareMap hwMap = null;
//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public hwMecanum() {
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        m_motor_fl = hwMap.get(DcMotor.class, "FL DRIVE");
        m_motor_fr = hwMap.get(DcMotor.class, "FR DRIVE");
        m_motor_rl = hwMap.get(DcMotor.class, "RL DRIVE");
        m_motor_rr = hwMap.get(DcMotor.class, "RR DRIVE");
        DcMotor[] m_motors = {m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr};
        imu = hwMap.get(IMU.class, "imu");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        m_motor_fl.setDirection(DcMotor.Direction.REVERSE);
        m_motor_fr.setDirection(DcMotor.Direction.REVERSE);
        m_motor_rl.setDirection(DcMotor.Direction.FORWARD);
        m_motor_rr.setDirection(DcMotor.Direction.FORWARD);

        // Setup the motors
        setAllDrivePower(0);
        for (DcMotor m : m_motors) {
            // Encoder setup
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); //greater speed
            // m.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //greater accuracy (80% speed)
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            // Brake/Coast Mode
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Adjust the orientation parameters of the IMU
        IMU.Parameters imuParams = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(imuParams);

        BlocksOpModeCompanion.telemetry.addData(">", "Hardware Initialized");
        BlocksOpModeCompanion.telemetry.update();
    }

    public void setAllDrivePower(double p) {
        setDrivePower(p,p,p,p);
    }

    public void setDrivePower(double fl, double fr, double rl, double rr) {
        m_motor_fl.setPower(fl);
        m_motor_fr.setPower(fr);
        m_motor_rl.setPower(rl);
        m_motor_rr.setPower(rr);
    }

    public double getRobotYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
