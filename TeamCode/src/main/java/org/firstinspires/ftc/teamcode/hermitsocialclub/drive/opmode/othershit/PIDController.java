package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController extends LinearOpMode {


    /*

     * Proportional Integral Derivative Controller w/ Low pass filter and anti-windup

     */

    double Kp = 5;
    double Ki = 0;
    double Kd = 1;

    DcMotor linears;


    double reference = 1;
    double lastReference = reference;
    double integralSum = 0;

    double lastError = 0;

    double maxIntegralSum;

    double a = 0.8; // a can be anything from 0 < a < 1
    double previousFilterEstimate = 0;
    double currentFilterEstimate = 0;
    double targetPos;

    // Elapsed timer class from SDK, please use it, it's epic
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

// while (linears.getCurrentPosition()<targetPos) {

        while (linears.getCurrentPosition() < targetPos) {

            // obtain the encoder position
            double encoderPosition = linears.getCurrentPosition();
            // calculate the error
            double error = reference - encoderPosition;

            double errorChange = (error - lastError);

            // filter out hight frequency noise to increase derivative performance
            currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
            previousFilterEstimate = currentFilterEstimate;

            // rate of change of the error
            double derivative = currentFilterEstimate / timer.seconds();

            // sum of all error over time
            integralSum = integralSum + (error * timer.seconds());


            // max out integral sum
            if (integralSum > maxIntegralSum) {
                integralSum = maxIntegralSum;
            }

            if (integralSum < -maxIntegralSum) {
                integralSum = -maxIntegralSum;
            }

            // reset integral sum upon setpoint changes
            if (reference != lastReference) {
                integralSum = 0;
            }

            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            linears.setPower(out);

            lastError = error;

            lastReference = reference;

            // reset the timer for next time
            timer.reset();

        }
    }
}