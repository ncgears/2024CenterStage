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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class nccogsHardware2 {

    /* Public OpMode members. */

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor fL, fR, rL, rR   = null;
    public MecanumDrive drive = null;
    public RevIMU imu = null;
    public GamepadEx driverOp, operOp = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    Motor m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr = null;

//    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public nccogsHardware2() {
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        driverOp = new GamepadEx(gamepad1);
        operOp = new GamepadEx(gamepad2);

        imu = new RevIMU(hwMap, "imu");

        m_motor_fl = new Motor(hwMap, "FL DRIVE");
        m_motor_fr = new Motor(hwMap, "FR DRIVE");
        m_motor_rl = new Motor(hwMap, "RL DRIVE");
        m_motor_rr = new Motor(hwMap, "RR DRIVE");
        Motor[] m_motors = {m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr};

        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        fL = m_motor_fl.motor;
        fR = m_motor_fr.motor;
        rL = m_motor_rl.motor;
        rR = m_motor_rr.motor;
        drive = new MecanumDrive(m_motor_fl, m_motor_fr, m_motor_rl, m_motor_rr);

        // Invert motors as needed so forward is the correct direction
        m_motor_fl.setInverted(true);
        m_motor_fr.setInverted(false);
        m_motor_rl.setInverted(false);
        m_motor_rr.setInverted(true);

        // Set the runmode for each motor
        for (Motor m : m_motors) {
            m.setRunMode(Motor.RunMode.RawPower);
        }

        // Reset the encoders for each motor
        for (Motor m : m_motors) {
            m.resetEncoder();
        }

        // Initialize the IMU
        imu.init();

        // Define and initialize ALL installed servos.

        BlocksOpModeCompanion.telemetry.addData(">", "Hardware Initialized");
        BlocksOpModeCompanion.telemetry.update();

    }
}
