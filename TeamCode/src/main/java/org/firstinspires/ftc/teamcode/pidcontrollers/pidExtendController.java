package org.firstinspires.ftc.teamcode.pidcontrollers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Constant;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Constants;

public class pidExtendController {
    private double targetTicks;
    private double kP, kI, kD;
    private double accumulatedError = 0.0;
    private ElapsedTime timer = new ElapsedTime();
    private double lastError, lastTime = 0.0;
    private OpMode myOpMode = null;

    public pidExtendController(OpMode opmode, double target, double p, double i, double d) {
        myOpMode = opmode;
        targetTicks = target;
        kP = p;
        kI = i;
        kD = d;
    }

    /**
     * The update function of the extend controller returns a motor power for the extend motor.
     * @param currentTicks current encoder position of the extend encoder
     * @return motor power for the extend motor
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

    /**
     * Sets the extension target based on the angle of the elevator.
     * @param angle angle of the elevator
     * @param targetBased if true, calculated target is based on the target angle rather than actual angle
     */
    public void setTargetForAngle(double angle, boolean targetBased) {
        double calcTarget = 0.0;
        if(angle < Constants.Manipulator.elevatorController.limits.maxFloorPickupAngle) {
            calcTarget = angle * (Constants.Manipulator.extendController.limits.maxFloorPickupLength / Constants.Manipulator.elevatorController.limits.maxFloorPickupAngle);
        } else if (targetBased && angle == Constants.Manipulator.elevatorController.limits.transportAngle) {
            calcTarget = Constants.Manipulator.extendController.limits.transportLength;
        } else if (angle >= Constants.Manipulator.elevatorController.limits.minScoringAngle - Constants.Manipulator.elevatorController.targetThreshold
                && angle <= Constants.Manipulator.elevatorController.limits.maxScoringAngle + Constants.Manipulator.elevatorController.targetThreshold) {
            //calc length for angles between min and max
            //TODO: Work out the math here for distance to angle ratio
            double len = (Constants.Manipulator.extendController.limits.maxScoringLength - Constants.Manipulator.extendController.limits.minScoringLength);
            double ang = (Constants.Manipulator.elevatorController.limits.maxScoringAngle - Constants.Manipulator.elevatorController.limits.minScoringAngle);
            double lenPerAng = len/ang;
            calcTarget = lenPerAng * (angle - Constants.Manipulator.elevatorController.limits.minScoringAngle);
        } else if (angle > Constants.Manipulator.elevatorController.limits.maxScoringAngle + (Constants.Manipulator.elevatorController.targetThreshold * 3)) {
            //this would be moving to climbing position
            calcTarget = 0.0; //TODO: this should be the length to be able to clear the bar, for now, set it to 0
        } else {
            calcTarget = 0.0; //We shouldn't get here normally, but just in case, lets set length to 0
        }
        setTargetInches(calcTarget);
    }

    public void setTargetInches(double inches) {
        setTarget(inches * Constants.Manipulator.extendController.ticksPerInch);
    }

    public void setTarget(double ticks) {
        double calcTarget = ticks;
        if(calcTarget < Constants.Manipulator.extendController.limits.minLength * Constants.Manipulator.extendController.ticksPerInch) calcTarget = Constants.Manipulator.extendController.limits.minLength * Constants.Manipulator.extendController.ticksPerInch;
        if(calcTarget > Constants.Manipulator.extendController.limits.maxLength * Constants.Manipulator.extendController.ticksPerInch) calcTarget = Constants.Manipulator.extendController.limits.maxLength * Constants.Manipulator.extendController.ticksPerInch;
        targetTicks = calcTarget;
    }

    public double getLastError() {
        return lastError;
    }

    public boolean atTarget(double error) {
        return (Math.abs(error) < Constants.Manipulator.extendController.targetThresholdTicks);
    }
    public boolean atTarget() {
        return (Math.abs(lastError) < Constants.Manipulator.extendController.targetThresholdTicks);
    }
}
