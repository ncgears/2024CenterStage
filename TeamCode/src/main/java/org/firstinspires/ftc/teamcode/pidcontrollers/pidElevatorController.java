package org.firstinspires.ftc.teamcode.pidcontrollers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class pidElevatorController {
    private double targetTicks;
    private double kP, kI, kD;
    private double accumulatedError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError, lastTime = 0.0;
    private OpMode myOpMode = null;

    public pidElevatorController(OpMode opmode, double target, double p, double i, double d) {
        myOpMode = opmode;
        targetTicks = target;
        kP = p;
        kI = i;
        kD = d;
    }

    /**
     * The update function of the elevator controller returns a motor power for the elevator motor.
     * @param currentTicks current encoder position of the elevator encoder
     * @return motor power for the elevator motor
     */
    public double update(double currentTicks) {
        //P - Proportional - This determines the error that we will multiply by our constant to set power
        double error = targetTicks - currentTicks;
//        myOpMode.telemetry.addData("drive error", "%.1f", error);
//        RobotLog.d(String.format("drive error = %.2f", error));
//        myOpMode.telemetry.update();
        //I - Integral - This accumulates the error over time to correct for not getting to the set point
        accumulatedError += error;
        //if we reach the threshold, reset accumulated error to stop adding it
        if (atTarget(error)) {
            accumulatedError = 0;
        }
        accumulatedError = Math.abs(accumulatedError) * Math.signum(error);

        //D - Deriviative - This slows down the robot when its moving too rapidly
        double slope = 0.0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //Motor Power calculation
        double motorPower = 0.1 * Math.signum(error) + 0.9 * Math.tanh(
                (kP * error) + (kI * accumulatedError) + (kD * slope)
        );
        return motorPower;
    }

    public double getTarget() {
        return targetTicks;
    }

    public void setTargetPosition(Constants.Manipulator.Positions position) {
        setTargetInches(position.getElevator());
    }

    public void setTargetInches(double inches) {
        setTarget(inches * Constants.Manipulator.elevatorController.ticksPerInch);
    }

    public void setTarget(double ticks) {
        double calcTarget = ticks;
        if(calcTarget < Constants.Manipulator.elevatorController.limits.minLength * Constants.Manipulator.elevatorController.ticksPerInch) calcTarget = Constants.Manipulator.elevatorController.limits.minLength * Constants.Manipulator.elevatorController.ticksPerInch;
        if(calcTarget > Constants.Manipulator.elevatorController.limits.maxLength * Constants.Manipulator.elevatorController.ticksPerInch) calcTarget = Constants.Manipulator.elevatorController.limits.maxLength * Constants.Manipulator.elevatorController.ticksPerInch;
        targetTicks = calcTarget;
    }

    public double getLastError() {
        return lastError;
    }

    public boolean atTarget(double error) {
        return (Math.abs(error) < Constants.Manipulator.elevatorController.targetThresholdTicks);
    }
    public boolean atTarget() {
        return (Math.abs(lastError) < Constants.Manipulator.elevatorController.targetThresholdTicks);
    }
}
