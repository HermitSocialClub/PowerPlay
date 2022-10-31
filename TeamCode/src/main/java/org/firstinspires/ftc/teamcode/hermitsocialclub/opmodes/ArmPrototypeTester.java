package org.firstinspires.ftc.teamcode.hermitsocialclub.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="Coach's Arm Prototype", group="Pushbot")
public class ArmPrototypeTester extends LinearOpMode {
    //HardwarePushbot robot           = new HardwarePushbot();

    public DcMotor linear = null;
    //HardwareMap hwMap           =  new HardwareMap();

    double linearPower;

    @Override

    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        linear = hardwareMap.get(DcMotor.class,"linear");
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // if (gamepad1.left_stick_y>-1)
            //  linear.setPower(gamepad1.left_stick_y);

            linearPower = gamepad1.left_stick_y;

            linear.setPower(linearPower);

            if (linearPower < 0.05)
                linear.setPower(0);

        }
    }
}

