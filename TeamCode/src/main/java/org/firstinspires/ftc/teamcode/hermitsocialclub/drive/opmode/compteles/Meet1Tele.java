package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compteles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;


@TeleOp (name = "Meet1Tele", group = "Pushbot")
public class Meet1Tele extends LinearOpMode {
    public DcMotor linear = null;
   // public CRServo intake = null;


    //HardwareMap hwMap           =  new HardwareMap();
    bigWheelOdoMecanum drive;
    double linearPower;
    boolean yesClaw;
    double clawPosition;
    public Servo claw;

    ElapsedTime runtime = new ElapsedTime();
    private boolean lastAMash = false;
    private boolean lastBMash = false;
    public boolean reverseDirections = false;
    public double reverseMod = 1;
    public boolean precisionMode = false;
    public double precisionModifier = 0.9;
    double driveRightPower;
    double driveLeftPower;
    double driveRight2Power;
    double driveLeft2Power;
    double y;
    double x;
    double rx;
    double x2;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new bigWheelOdoMecanum(hardwareMap);
//        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
//        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
//        right_drive_2.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
//        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");
//        left_drive_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linear = hardwareMap.get(DcMotor.class,"linear");
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // intake = hardwareMap.get(CRServo.class,"intake");
        claw = hardwareMap.get(Servo.class,"claw");


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            if (!lastAMash && gamepad1.a) {

                if (precisionMode) {
                    precisionMode = false;
                    precisionModifier = 1;
                    // telemetry.addLine("Precision Mode DEACTIVATED!");
                    telemetry.speak("Precision Mode DEACTIVATED");
                    telemetry.addLine("precision mode deactivate");

                } else {
                    precisionMode = true;
                    precisionModifier = 0.5;
                    //telemetry.addLine("Precision Mode ACTIVATED!");
                    telemetry.speak("Precision Mode ACTIVATED");
                    telemetry.addLine("precision mode activate");

                }
                runtime.reset();

            }
            lastAMash = gamepad1.a;

            if (!lastBMash && gamepad1.b) {

                if (reverseDirections) {
                    reverseDirections = false;
                    reverseMod = 1;
                    // telemetry.addLine("Precision Mode DEACTIVATED!");
                    telemetry.speak("Reverse Mode DEACTIVATED");
                    telemetry.addLine("Reverse mode deactivate");

                } else {
                    reverseDirections = true;
                    reverseMod = -1;
                    //telemetry.addLine("Precision Mode ACTIVATED!");
                    telemetry.speak("reverse Mode ACTIVATED");
                    telemetry.addLine("reverse mode activate");

                }
                runtime.reset();

            }
            lastAMash = gamepad1.b;

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            -(((Math.abs(gamepad1.left_stick_y) <.2) ? 0 : gamepad1.left_stick_y-.2)/.8)*precisionModifier,
//                            -(((Math.abs(gamepad1.left_stick_x) <.2) ? 0 : gamepad1.left_stick_x-.2)/.8)*precisionModifier,
//                            -(((Math.abs(gamepad1.right_stick_x) <.2) ? 0 : gamepad1.right_stick_x-.2)/.8)*precisionModifier
//                    )
//            );

            x = gamepad1.left_stick_x * 1;
            x2 = gamepad1.left_stick_x * 1.5;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double denominator2 = Math.max(Math.abs(y) + Math.abs(x2) + Math.abs(rx), 1);

            if (precisionMode == false && reverseDirections == false) {
                driveRightPower = (((y - x2 - rx) / denominator2) * 0.85);
                driveRight2Power = (((y + x - rx) / denominator) * 0.8);
                driveLeftPower = ((y + x + rx) / denominator);
                driveLeft2Power = (((y - x2 + rx) / denominator2) * 0.8);
            } else if (precisionMode == true && reverseDirections == false) {
                driveRightPower = (((y - x2 - rx) / denominator2) * 0.95)*precisionModifier;
                driveRight2Power = (((y + x - rx) / denominator) * 0.9)*precisionModifier;
                driveLeftPower = ((y + x + rx) / denominator)*precisionModifier;
                driveLeft2Power = (((y - x2 + rx) / denominator2) * 0.9)*precisionModifier;
            }else if (precisionMode == false && reverseDirections == true) {
                driveRightPower = (((y - x2 - rx) / denominator2) * 0.85)*reverseMod;
                driveRight2Power = (((y + x - rx) / denominator) * 0.8)*reverseMod;
                driveLeftPower = ((y + x + rx) / denominator)*reverseMod;
                driveLeft2Power = (((y - x2 + rx) / denominator2) * 0.8)*reverseMod;
            } else if (precisionMode == true && reverseDirections == true) {
                driveRightPower = (((y - x2 - rx) / denominator2) * 0.95)*precisionModifier*reverseMod;
                driveRight2Power = (((y + x - rx) / denominator) * 0.9)*precisionModifier*reverseMod;
                driveLeftPower = ((y + x + rx) / denominator)*precisionModifier*reverseMod;
                driveLeft2Power = (((y - x2 + rx) / denominator2) * 0.9)*precisionModifier*reverseMod;
            }

            drive.rightRear.setPower(driveRight2Power);
            drive.rightFront.setPower(driveRightPower);
            drive.leftRear.setPower(driveLeft2Power);
            drive.leftFront.setPower(driveLeftPower);

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            linearPower = gamepad2.right_stick_y;

           // linear.setPower(linearPower);

            if (Math.abs(linearPower)<0.02){
                linear.setPower(0);
            } else {
                linear.setPower(-linearPower);
            }

//            if (Math.abs(linearPower) < (0.03)){
//                linear.setPower(0.003);
//            }
//            else {
//                linear.setPower(-linearPower);
//            }

//            if (gamepad2.left_trigger > 0.5) {
//                intake.setPower(0.5);
//            }
//
//            else if (gamepad2.right_trigger > 0.5){
//                intake.setPower(-0.5);
//            } else {
//                intake.setPower(0);
//            }

//            if (gamepad2.a) {
//                claw.setPosition(0);
//            } else if (gamepad2.b) {
//                claw.setPosition(0.65);
//            }

            if (gamepad2.left_bumper){
               // claw.setPower(-1);
                claw.setPosition(0);
            } else if (gamepad2.right_bumper) {
                //claw.setPower(0.4);
               claw.setPosition(0.7);
            } else {
               // claw.setPower(0);
                claw.setPosition(0);
            }

            if (gamepad2.left_trigger > 0.03){
                drive.fourBar.setPosition(0.2);
            } else if (gamepad2.right_trigger > 0.03) {
                drive.fourBar.setPosition(1);
            } else {
                drive.fourBar.setPosition(0.2);
            }

            if (gamepad2.a){
                drive.fourBar.setPosition(0);
            } if (gamepad2.b){
                drive.fourBar.setPosition(1);
            }if (gamepad2.x){
                drive.fourBar.setPosition(0.5);
            }



          //  claw.setPower(gamepad2.left_stick_x);



       /*     if (gamepad2.a) {
                intake.setPower(1);
            } //else intake.setPower(0.5);

            if (gamepad2.b){
                intake.setPower(-1);
            }//else intake.setPower(0.5);

           else intake.setPower(0);*/

        }

    }
}
