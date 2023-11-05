package org.firstinspires.ftc.teamcode.pidcontrollers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;

public class pidTiltController {
    private double targetTicks;
    private double kP, kI, kD, kF;
    private double accumulatedError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError, lastTime = 0.0;
    private OpMode myOpMode = null;

    public pidTiltController(OpMode opmode, double target, double p, double i, double d, double f) {
        myOpMode = opmode;
        targetTicks = target;
        kP = p;
        kI = i;
        kD = d;
        kF = f;
    }

    /**
     * The update function of the elevator controller returns a motor power for the elevator motor.
     * @param currentTicks current encoder position of the elevator encoder
     * @return motor power for the elevator motor
     */
    public double update(double currentTicks) {
        //P - Proportional - This determines the error that we will multiply by our constant to set power
        double error = targetTicks - currentTicks;
        //I - Integral - This accumulates the error over time to correct for not getting to the set point
        accumulatedError += error;
        accumulatedError = accumulatedError + (error * (timer.milliseconds() - lastTime));
        if (atTarget(error)) { //if we reach the threshold, reset accumulated error to stop adding it
            accumulatedError = 0;
        }

        //D - Deriviative - This slows down the robot when its moving too rapidly
        double slope = 0.0;
        if (lastTime > 0) {
            slope = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastTime = timer.milliseconds();
        lastError = error;

        //Motor Power calculation
        double motorPower = kF * Math.signum(error) + (1.0 - kF) * Math.tanh(
                (kP * error) + (kI * accumulatedError) + (kD * slope)
        );
        return motorPower;
    }

    public double getTarget() {
        return targetTicks;
    }

    public void setTargetPosition(Constants.Manipulator.Positions position) {
        setTarget(position.getTilt());
    }

    public void setTarget(double ticks) {
        double calcTarget = ticks;
        calcTarget = Math.max(calcTarget,Constants.Manipulator.tiltController.limits.minTicks); //make sure we are above min
        calcTarget = Math.min(calcTarget,Constants.Manipulator.tiltController.limits.maxTicks); //make sure we are below max
        targetTicks = calcTarget;
    }

    public double getLastError() {
        return lastError;
    }

    public boolean atTarget(double error) {
        return (Math.abs(error) < Constants.Manipulator.tiltController.targetThresholdTicks);
    }
    public boolean atTarget() {
        return (Math.abs(lastError) < Constants.Manipulator.tiltController.targetThresholdTicks);
    }
}
