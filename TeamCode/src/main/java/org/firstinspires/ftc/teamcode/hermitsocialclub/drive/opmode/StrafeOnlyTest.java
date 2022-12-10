package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.SampleMecanumDrive;

@TeleOp (name = "StrafeOnlyTest" )
public class StrafeOnlyTest extends LinearOpMode {

    public DcMotor left_drive = null;
    public DcMotor left_drive_2 = null;
    public DcMotor right_drive = null;
    public DcMotor right_drive_2 = null;
    SampleMecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
        left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");
        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        drive = new SampleMecanumDrive(hardwareMap);


        waitForStart();
        if(isStopRequested())return;

        while (opModeIsActive()){

//            if (gamepad1.a){
//                left_drive.setPower(0.5);
//
//            }
            drive.setWeightedDrivePower(
                    new Pose2d(
                            0,
                            gamepad1.a ? 1 : gamepad1.b ? -1 : 0,
                            0
                    )
            );
            drive.update();



        telemetry.addData(" Left front", left_drive.getCurrentPosition());
        telemetry.addData(" Left back", left_drive_2.getCurrentPosition());

        telemetry.addData(" Right front", right_drive.getCurrentPosition());
        telemetry.addData(" Right back", right_drive_2.getCurrentPosition());

        telemetry.update();


        }
    }
}

