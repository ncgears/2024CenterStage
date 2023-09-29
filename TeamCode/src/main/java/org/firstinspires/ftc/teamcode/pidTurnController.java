package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class pidTurnController {
    private double targetAngle;
    private double kP, kI, kD;
    private double accumulatedError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError, lastTime = 0.0;

    public pidTurnController(double target, double p, double i, double d) {
        targetAngle = target;
        kP = p;
        kI = i;
        kD = d;
    }
    public double update(double currentAngle) {
        //P - Proportional - This determines the error that we will multiply by our constant to set power
        double error = targetAngle - currentAngle;
        error %= 360;
        error += 360;
        error %= 360;
        if (error > 180) error -= 360;

        //I - Integral - This accumulates the error over time to correct for not getting to the set point
        accumulatedError += error;
        if (Math.abs(error) < 1) {
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
}
