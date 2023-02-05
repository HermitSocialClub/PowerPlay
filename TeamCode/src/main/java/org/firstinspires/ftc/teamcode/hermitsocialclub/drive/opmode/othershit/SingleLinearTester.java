package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "SingleLinearTester")
public class SingleLinearTester extends LinearOpMode {

    DcMotor linear1;
   // DcMotor linear2;
    double linearPower;

    @Override
    public void runOpMode() throws InterruptedException {

        linear1 = hardwareMap.get(DcMotor.class,"linear1");
        linear1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // linear2 = hardwareMap.get(DcMotor.class,"linear2");
      //  linear2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (isStopRequested())return;

        while (opModeIsActive()){

            linearPower = gamepad1.left_stick_y;

            if (Math.abs(linearPower) < (0.05)){
                linear1.setPower(0.05);
               // linear2.setPower(0.05);
            }
            else {
                linear1.setPower(-linearPower);
               // linear2.setPower(-linearPower);
            }

        }

    }
}
