package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compteles;

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
    double x2;
  //  double linearExtenderPower;
    //double linearFlipperPower;
    boolean yesClaw;
    double clawPosition;

    @Override
    public void runOpMode() throws InterruptedException {

        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        right_drive_2.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");
        left_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // linearExtender = hardwareMap.get(DcMotor.class,"linearExtender");
    /*    linearFlippper = hardwareMap.get(DcMotor.class,"linearFlipper");
        linearFlippper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linearFlippper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);*/
        claw = hardwareMap.get(Servo.class,"claw");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            x = gamepad1.left_stick_x * 1;
            x2 = gamepad1.left_stick_x * 1.5;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double denominator2 = Math.max(Math.abs(y) + Math.abs(x2) + Math.abs(rx), 1);
            driveRightPower = (y-x2-rx) / denominator2;
            driveRight2Power = (y+x-rx) / denominator;
            driveLeftPower = (y+x+rx) / denominator;
            driveLeft2Power = (y-x2+rx) / denominator2;

            right_drive_2.setPower(driveRight2Power);
            right_drive.setPower(driveRightPower);
            left_drive_2.setPower(driveLeft2Power);
            left_drive.setPower(driveLeftPower);

           // linearFlipperPower = gamepad2.left_stick_y;
         //   linearExtenderPower = gamepad2.right_stick_y;

            //linearExtender.setPower(linearExtenderPower * 0.9);
            //linearFlippper.setPower(linearFlipperPower * 0.9);

           /* if (linearFlipperPower < 0.05){
                linearFlippper.setPower(0.025);
            }*/

           // claw.setPosition(gamepad2.left_trigger);

            if (gamepad2.a){
                claw.setPosition(0.2);
            } else
                claw.setPosition(0);

            if (gamepad2.b){
                claw.setPosition(0);
            }

           /* if (gamepad2.left_bumper){
                yesClaw = true;
            }else {
                yesClaw = false;
            }

            if (yesClaw){
                clawPosition = 0.3;
                claw.setPosition(clawPosition);
            }else {
                clawPosition = 0;
                claw.setPosition(0);
            }*/



        }

    }
}
