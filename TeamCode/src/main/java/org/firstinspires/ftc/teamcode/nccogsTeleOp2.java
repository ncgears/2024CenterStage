package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="NCCOGS TeleOp2", group="Jim")
public class nccogsTeleOp2 extends OpMode {
    nccogsHardware2 robot = new nccogsHardware2();

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        robot.drive.driveFieldCentric(
                robot.driverOp.getLeftX(),
                robot.driverOp.getLeftY(),
                robot.driverOp.getRightX(),
                robot.imu.getHeading()
        );
    }
}
