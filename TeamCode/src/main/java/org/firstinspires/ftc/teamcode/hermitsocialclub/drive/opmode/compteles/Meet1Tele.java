package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compteles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    private boolean lastXMash = false;
    public boolean reverseDirections = false;
    public double reverseMod = 1;
//    public boolean precisionMode = false;
    public boolean lockedStrafe = false;
//    public double precisionModifier = 0.9;
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
        drive.setPoseEstimate(new Pose2d(0, 0, 0));
        ElapsedTime opmodeRunTime = new ElapsedTime();


        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){
            opmodeRunTime.reset();
//            if (gamepad1.a) {
//                precisionMode = !precisionMode;
//                precisionModifier = precisionMode ? 1 : 0.5;
//                telemetry.speak((precisionMode ? "precision" : "speed"));
//                telemetry.addLine((precisionMode ? "ON" : "OFF"));
//            }
//            if (gamepad1.a) {
//
//                if (precisionMode) {
//                    precisionMode = false;
//                    precisionModifier = 1;
//                    // telemetry.addLine("Precision Mode DEACTIVATED!");
//                    telemetry.speak("speed");
//                    telemetry.addLine("precision mode deactivate");
//
//                } else {
//                    precisionMode = true;
//                    precisionModifier = 0.5;
//                    //telemetry.addLine("Precision Mode ACTIVATED!");
//                    telemetry.speak("precision");
//                    telemetry.addLine("precision mode activate");
//
//                }
//                runtime.reset();
//
//            }
//            lastAMash = gamepad1.a;

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
//            if (gamepad1.x) {
//                lockedStrafe = !lockedStrafe;
//                telemetry.speak("locked mode is " + (lockedStrafe ? "ON" : "OFF"));
//                telemetry.addLine("locked mode is " + (lockedStrafe ? "ON" : "OFF"));
//            }
//            if (!lockedStrafe) {
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -(((Math.abs(gamepad1.left_stick_y) < .2) ? 0 : gamepad1.left_stick_y) / .72) * (gamepad1.right_trigger > 0.05 ? 0.8 : 0.4),
                                -(((Math.abs(gamepad1.left_stick_x) < .2) ? 0 : gamepad1.left_stick_x) / .72) * (gamepad1.right_trigger > 0.05 ? 0.8 : 0.4),
                                -(((Math.abs(gamepad1.right_stick_x) < .2) ? 0 : gamepad1.right_stick_x) / .72) * 0.7 * (gamepad1.right_trigger > 0.05 ? 0.8 : 0.4)
                        )
                );
//            }
//            else {
//                if (Math.abs(gamepad1.left_stick_y) <= Math.abs(gamepad1.left_stick_x)) {
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                0,
//                                    -(((Math.abs(gamepad1.left_stick_x) < .2) ? 0 : gamepad1.left_stick_x - .2) / .8) * (gamepad1.right_trigger > 0.05 ? 0.8 : 0.4),
//                                    -(((Math.abs(gamepad1.right_stick_x) < .2) ? 0 : gamepad1.right_stick_x - .2) / .8) * 0.5 * (gamepad1.right_trigger > 0.05 ? 0.8 : 0.4)
//                            )
//                    );
//                }
//                else{
//                    drive.setWeightedDrivePower(
//                            new Pose2d(
//                                    -(((Math.abs(gamepad1.left_stick_y) < .2) ? 0 : gamepad1.left_stick_y - .2) / .8) * (gamepad1.right_trigger > 0.05 ? 0.8 : 0.4),
//                                    0,
//                                    -(((Math.abs(gamepad1.right_stick_x) < .2) ? 0 : gamepad1.right_stick_x - .2) / .8) * (gamepad1.right_trigger > 0.05 ? 0.8 : 0.4)
//                            )
//                    );
//                }
//            }
            // Read pose
            Pose2d poseEstimate = drive.getPoseEstimate();

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
//            Vector2d input = new Vector2d(
//                           -(((Math.abs(gamepad1.left_stick_y) <.2) ? 0 : gamepad1.left_stick_y-.2)/.8)*precisionModifier,
//                           -(((Math.abs(gamepad1.left_stick_x) <.2) ? 0 : gamepad1.left_stick_x-.2)/.8)*precisionModifier
//            );
            double rotation;
//            double current = poseEstimate.getHeading();
            double target;
//            if (current < 0) {
//                current += 360;
//            }

//            if (gamepad1.dpad_left) {
//                rotation = 90 - current;
//            }
//            else if (gamepad1.dpad_right) {
//                rotation = current - 90;
//            }
//            else {
//                rotation = gamepad1.left_stick_x;
//
//            }
//            else if (gamepad1.)
            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
//            if (fieldCentric) {
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                input.rotated(-poseEstimate.getHeading()).getX(),
//                                input.rotated(-poseEstimate.getHeading()).getY(),
//                                -gamepad1.left_stick_x
//                        )
//                );
//            }
//            else {
//                drive.setWeightedDrivePower(
//                        new Pose2d(
//                                input.getX(),
//                                input.getY(),
//                                -gamepad1.left_stick_x
//                        )
//                );
//            }


            x = gamepad1.left_stick_x * 1;
            x2 = gamepad1.left_stick_x * 1.5;
            y = -gamepad1.left_stick_y;
            rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double denominator2 = Math.max(Math.abs(y) + Math.abs(x2) + Math.abs(rx), 1);

           /* if (precisionMode == false && reverseDirections == false) {
                driveRightPower = (((y - x2 - rx) / denominator2) * 0.90);
                driveRight2Power = (((y + x - rx) / denominator) * 0.8);
                driveLeftPower = ((y + x + rx) / denominator);
                driveLeft2Power = (((y - x2 + rx) / denominator2) * 0.8);
            } else if (precisionMode == true && reverseDirections == false) {
                driveRightPower = (((y - x2 - rx) / denominator2) * 0.95)*precisionModifier;
                driveRight2Power = (((y + x - rx) / denominator) * 0.9)*precisionModifier;
                driveLeftPower = ((y + x + rx) / denominator)*precisionModifier;
                driveLeft2Power = (((y - x2 + rx) / denominator2) * 0.9)*precisionModifier;
            }else if (precisionMode == false && reverseDirections == true) {
                driveRightPower = (((y - x2 - rx) / denominator2) * 0.9)*reverseMod;
                driveRight2Power = (((y + x - rx) / denominator) * 0.8)*reverseMod;
                driveLeftPower = ((y + x + rx) / denominator)*reverseMod;
                driveLeft2Power = (((y - x2 + rx) / denominator2) * 0.8)*reverseMod;
            } else if (precisionMode == true && reverseDirections == true) {
                driveRightPower = (((y - x2 - rx) / denominator2) * 0.9)*precisionModifier*reverseMod;
                driveRight2Power = (((y + x - rx) / denominator) * 0.9)*precisionModifier*reverseMod;
                driveLeftPower = ((y + x + rx) / denominator)*precisionModifier*reverseMod;
                driveLeft2Power = (((y - x2 + rx) / denominator2) * 0.9)*precisionModifier*reverseMod;
            }*/

//            drive.rightRear.setPower(driveRight2Power);
//            drive.rightFront.setPower(driveRightPower);
//            drive.leftRear.setPower(driveLeft2Power);
//            drive.leftFront.setPower(driveLeftPower);
//
            drive.update();

//            Pose2d poseEstimate = drive.getPoseEstimate();
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

            Gamepad.LedEffect rgbEffect = new Gamepad.LedEffect.Builder()
                    .addStep(1, 0, 1, 250) // Show red for 250ms
                    .addStep(0, 1, 0, 250) // Show green for 250ms
                    .addStep(1, 0, 0, 500) // Show blue for 250ms
                    .build();

            Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder()
                    .addStep(0.0, 0.0, 90000)  //  Rumble right motor 100% for 500 mSec
                   // .addStep(0.0, 0.0, 5000)  //  Pause for 300 mSec
                    .addStep(1.0,1.0,1000)
                    .build();



            if (gamepad1.dpad_down){
                gamepad1.runRumbleEffect(effect);
                gamepad1.runLedEffect(rgbEffect);
            }

            if (gamepad1.dpad_right){
                gamepad1.setLedColor(255,0,0,5000);
            }

            if (opmodeRunTime.seconds()>=0) {
                gamepad1.runRumbleEffect(effect);
//                gamepad1.rumble(5000);
//                gamepad2.rumble(5000);
//                //gamepad1.setLedColor(255,0,0,5000);
//                gamepad1.runLedEffect(rgbEffect);
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

//            if (gamepad2.left_trigger > 0.03){
//                drive.fourBar.setPosition(0.2);
//            } else if (gamepad2.right_trigger > 0.03) {
//                drive.fourBar.setPosition(1);
//            } else {
//                drive.fourBar.setPosition(0.2);
//            }

//            if (gamepad2.a){
//                drive.fourBar.setPosition(0);
//            } if (gamepad2.b){
//                drive.fourBar.setPosition(1);
//            }if (gamepad2.x){
//                drive.fourBar.setPosition(0.5);
//            }



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
