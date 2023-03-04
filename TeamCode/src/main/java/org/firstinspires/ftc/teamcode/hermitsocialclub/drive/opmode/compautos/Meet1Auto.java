package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@Autonomous (name = "Meet1Auto")
public class Meet1Auto extends LinearOpMode {

    public DcMotor right_drive = null;
    public DcMotor left_drive = null;
    public DcMotor right_drive_2 = null;
    public DcMotor left_drive_2 = null;
    public DcMotor linear = null;
    public Servo intake = null;

    //Key Positions Blue
    Pose2d blueStartA5 = new Pose2d(-35,65,Math.toRadians(0));
    Vector2d parkOneTallJunctionPrep = new Vector2d(-15,65);
    Vector2d tallJunctionParkOne = new Vector2d(-12,25);
    Vector2d parkTwoThreePrep = new Vector2d(-12,12);
    Vector2d parkTwo = new Vector2d(-35,12);
    Vector2d parkThree = new Vector2d(-56,12);

    //Key Positions Red
    Pose2d redStartF5 = new Pose2d(-35,-65,Math.toRadians(1800));
    Vector2d redOneParkTallJunctionPrep = new Vector2d(-14,-65);
    Pose2d redOneParkTallJuntion = new Pose2d(-11,-25,Math.toRadians(0));
   // Vector2d

    //Switching Between Red and Blue Sides
    public static enum side {
        RED, BLUE

    }
    side colorSide = side.BLUE;

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
        linear = hardwareMap.get(DcMotor.class,"linear");
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake = hardwareMap.get(Servo.class,"intake");

        while (!isStarted()){
            if(gamepad1.a || gamepad2.a){
                if (colorSide == side.RED){
                    colorSide = side.BLUE;
                } else colorSide = side.RED;
            }
            telemetry.addData("Side",colorSide.toString());
        }

        waitForStart();

        switch (colorSide) {
            case RED: {
                //placeholder
            }
            case BLUE: {

            }
        }

    }
}
