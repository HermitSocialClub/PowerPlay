package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp (name = "RevCoreHexMotorTest")
public class RevCoreHexMotorTest extends LinearOpMode {

    public DcMotor tester = null;

    @Override
    public void runOpMode() throws InterruptedException {

        tester = hardwareMap.get(DcMotor.class,"tester");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            tester.setPower(gamepad1.right_stick_y);
            telemetry.addData("position", tester.getCurrentPosition());

        }

    }
}
