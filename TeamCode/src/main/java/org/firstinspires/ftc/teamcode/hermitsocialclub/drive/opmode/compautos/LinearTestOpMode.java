package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;

@Autonomous (name = "LinearTestOpMode")
public class LinearTestOpMode extends LinearOpMode {
    @Override

    public void runOpMode() throws InterruptedException {

        bigWheelOdoMecanum drive = new bigWheelOdoMecanum(hardwareMap);
        drive.linears.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        int initposition = drive.linears.getCurrentPosition();
        drive.linears.setTargetPosition(initposition + 4300);
        drive.linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        if(isStopRequested()) return;


    }
}
