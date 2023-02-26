package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "linearautotest")
public class LinearPID extends LinearOpMode {

    DcMotor linears;

    @Override
    public void runOpMode() throws InterruptedException {

        linears = hardwareMap.get(DcMotor.class,"linear");
        linears.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linears.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        if (isStopRequested())return;

        linearsMoveAuto(3000);

    }

    public void linearsMoveAuto (int distance){
        linears.setTargetPosition(linears.getCurrentPosition()+distance);
        // 3 below
        linears.setPower(0.7);
    }
}

