package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "FourBarTester")
public class FourBarTester extends LinearOpMode {

    DcMotor fourBar;

    @Override
    public void runOpMode() throws InterruptedException {

        fourBar = hardwareMap.dcMotor.get("fourBar");

    }
}
