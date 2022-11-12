package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "Meet0AutoBlue")
public class Meet0AutoBlue extends LinearOpMode {

    public DcMotor right_drive = null;
    public DcMotor left_drive = null;
    public DcMotor right_drive_2 = null;
    public DcMotor left_drive_2 = null;
    public DcMotor linearFlippper = null;
    public DcMotor linearExtender = null;
    public Servo claw = null;

    Double width = 18.0; //inches
    Integer cpr = 28; //counts per rotation
    Integer gear_ratio = 40;
    Double diameter = 4.125;
    Double cpi = (cpr * gear_ratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    Double bias = 0.8;//default 0.8
    Double meccyBias = 0.9;//change to adjust only strafing movement

    Double conversion = cpi * bias;
    Boolean exit = false;

    ElapsedTime runtime = new ElapsedTime();

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

        strafeToPosition(24,0.2);

    }
    public void strafeToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        left_drive_2.setTargetPosition(left_drive_2.getCurrentPosition() - move);
        left_drive.setTargetPosition(left_drive.getCurrentPosition() + move);
        right_drive_2.setTargetPosition(right_drive_2.getCurrentPosition() + move);
        right_drive.setTargetPosition(right_drive.getCurrentPosition() - move);
        //
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        left_drive.setPower(speed);
        left_drive_2.setPower(speed);
        right_drive.setPower(speed);
        right_drive_2.setPower(speed);
        //
        while (left_drive.isBusy() && right_drive.isBusy() && left_drive_2.isBusy() && right_drive_2.isBusy()) {
        }
        right_drive.setPower(0);
        left_drive.setPower(0);
        right_drive_2.setPower(0);
        left_drive_2.setPower(0);
        return;
    }
}