package org.firstinspires.ftc.teamcode.hermitsocialclub.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp (name = "Meet0Tele", group = "Pushbot")
public class Meet0Tele extends LinearOpMode {

    public DcMotor right_drive = null;
    public DcMotor left_drive = null;
    public DcMotor right_drive_2 = null;
    public DcMotor left_drive_2 = null;
    public DcMotor linearFlippper = null;
    public DcMotor linearExtender = null;
    public Servo claw = null;

    //HardwareMap hwMap           =  new HardwareMap();

    double driveRightPower;
    double driveLeftPower;
    double driveRight2Power;
    double driveLeft2Power;
    double y;
    double x;
    double rx;
    double linearExtenderPower;
    double linearFlipperPower;
    double clawPower;

    @Override
    public void runOpMode() throws InterruptedException {

        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        right_drive_2.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
        left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");
        linearExtender = hardwareMap.get(DcMotor.class,"linearExtender");
        linearFlippper = hardwareMap.get(DcMotor.class,"linearFlipper");
        claw = hardwareMap.get(Servo.class,"claw");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            x = gamepad1.left_stick_x * 1.1;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            driveRightPower = (y-x-rx) / denominator;
            driveRight2Power = (y+x-rx) / denominator;
            driveLeftPower = (y+x+rx) / denominator;
            driveLeft2Power = (y-x+rx) / denominator;

            right_drive_2.setPower(driveRight2Power);
            right_drive.setPower(driveRightPower);
            left_drive_2.setPower(driveLeft2Power);
            left_drive.setPower(driveLeftPower);

            linearFlipperPower = gamepad2.left_stick_y;
            linearExtenderPower = gamepad2.right_stick_y;

            linearExtender.setPower(linearExtenderPower);
            linearFlippper.setPower(linearFlipperPower);

            claw.setPosition(gamepad2.left_trigger);

        }

    }
}
