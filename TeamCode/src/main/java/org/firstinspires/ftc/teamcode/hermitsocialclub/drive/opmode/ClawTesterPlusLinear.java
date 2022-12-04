package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "ClawTesterPlusLinear")
public class ClawTesterPlusLinear extends LinearOpMode {

    public Servo claw = null;
    public DcMotor linear = null;
    double linearPower;

    @Override
    public void runOpMode() throws InterruptedException {

        claw = hardwareMap.get(Servo.class,"claw");
        linear = hardwareMap.get(DcMotor.class,"linear");

        waitForStart();
        if (isStopRequested())return;

        while (opModeIsActive()) {
            if (gamepad1.a) {
                claw.setPosition(1);
            } else if (gamepad1.b) {
                claw.setPosition(0);
            } else if (gamepad1.x) {
                claw.setPosition(0.5);
            }

            linearPower = gamepad1.left_stick_y;

            if (Math.abs(linearPower) < (0.05)) {
                linear.setPower(0.05);
            } else {
                linear.setPower(-linearPower);
            }
        }
    }
}
