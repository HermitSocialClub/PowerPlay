package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.SampleMecanumDrive;

@TeleOp (name = "MotorEncoderTest" )
public class MotorEncoderTest extends LinearOpMode {

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
            if (gamepad1.a) {
                left_drive.setPower(0.3);
                left_drive_2.setPower(0.3);
                right_drive.setPower(0.3);
                right_drive_2.setPower(0.3);
            }
            else if (gamepad1.b){
                left_drive.setPower(-0.3);
                left_drive_2.setPower(-0.3);
                right_drive.setPower(-0.3);
                right_drive_2.setPower(-0.3);
            }
            else {
                left_drive.setPower(0);
                left_drive_2.setPower(0);
                right_drive.setPower(0);
                right_drive_2.setPower(0);
            }

//            drive.setWeightedDrivePower(
//                    new Pose2d(
//                            gamepad1.a ? 0.5 : gamepad1.b ? -0.5 : 0,
//                            0,
//                            0
//                    )
//            );
//            drive.update();

            telemetry.addData(" Left front", left_drive.getCurrentPosition());
            telemetry.addData(" Left back", left_drive_2.getCurrentPosition());

            telemetry.addData(" Right front", right_drive.getCurrentPosition());
            telemetry.addData(" Right back", right_drive_2.getCurrentPosition());

            telemetry.update();


        }
    }
}
