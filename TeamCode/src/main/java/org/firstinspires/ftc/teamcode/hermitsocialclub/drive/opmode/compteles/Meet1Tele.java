package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compteles;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public CRServo claw;

    ElapsedTime runtime = new ElapsedTime();
    private boolean lastAMash = false;
    public boolean precisionMode = false;
    public double precisionModifier = 0.9;

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
        claw = hardwareMap.get(CRServo.class,"claw");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()){

            if (!lastAMash && gamepad1.a) {

                if (precisionMode) {
                    precisionMode = false;
                    precisionModifier = 0.9;
                    // telemetry.addLine("Precision Mode DEACTIVATED!");
                    telemetry.speak("Precision Mode DEACTIVATED");

                } else {
                    precisionMode = true;
                    precisionModifier = 0.4;
                    //telemetry.addLine("Precision Mode ACTIVATED!");
                    telemetry.speak("Precision Mode ACTIVATED");

                }
                runtime.reset();

            }
            lastAMash = gamepad1.a;

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -(((Math.abs(gamepad1.left_stick_y) <.2) ? 0 : gamepad1.left_stick_y-.2)/.8)*precisionModifier,
                            -(((Math.abs(gamepad1.left_stick_x) <.2) ? 0 : gamepad1.left_stick_x-.2)/.8)*precisionModifier,
                            -(((Math.abs(gamepad1.right_stick_x) <.2) ? 0 : gamepad1.right_stick_x-.2)/.8)*precisionModifier
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
            linearPower = gamepad2.right_stick_y;


            if (Math.abs(linearPower) < (0.05)){
                linear.setPower(0.005);
            }
            else {
                linear.setPower(-linearPower);
            }

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
                claw.setPower(-1);
                //claw.setPosition(0);
            } else if (gamepad2.right_bumper) {
                claw.setPower(0.4);
               // claw.setPosition(0.7);
            } else {
                claw.setPower(0);
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
