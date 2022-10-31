package org.firstinspires.ftc.teamcode.hermitsocialclub.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Halloween Robot", group="Pushbot")
public class HalloweenRobot extends LinearOpMode {
    //HardwarePushbot robot           = new HardwarePushbot();

    public DcMotor right_drive = null;
    public DcMotor left_drive = null;
    public DcMotor spinny_thing = null;

    //HardwareMap hwMap           =  new HardwareMap();

    double driveRightPower;
    double driveLeftPower;
    double turnPowerRight;
    double turnPowerLeft;
    double spinnyPower;

    @Override

    public void runOpMode() throws InterruptedException {
        //robot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
       // right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
     //   left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");
        spinny_thing = hardwareMap.get(DcMotor.class,"spinny_thing");

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // if (gamepad1.left_stick_y>-1)
            //  linear.setPower(gamepad1.left_stick_y);

            driveRightPower = -gamepad1.left_stick_y;
            driveLeftPower = gamepad1.left_stick_y;
           // drivePower = Math.abs(gamepad1.right_stick_x);

            right_drive.setPower(driveRightPower);
          //  right_drive_2.setPower(drivePower);
            left_drive.setPower(driveLeftPower);
           // left_drive_2.setPower(drivePower);

            if (driveRightPower > -0.05) {
                right_drive.setPower(0);
            }
           //     right_drive_2.setPower(0);

                if (driveLeftPower < 0.05) {
                    left_drive.setPower(0);
                }
             //   left_drive_2.setPower(0);

         /*   if (gamepad1.right_stick_x > 0.05){
                turnPowerLeft = gamepad1.right_stick_x;
                turnPowerRight = 0;
            }

            if (gamepad1.right_stick_x < -0.05){
                turnPowerRight = gamepad1.right_stick_x;
                turnPowerLeft = 0;
            }*/
          //  turnPowerRight = gamepad1.left_stick_y;
            //turnPowerLeft = -gamepad1.left_stick_y;

            turnPowerLeft = -gamepad1.right_stick_x;
            turnPowerRight = -gamepad1.right_stick_x;

            right_drive.setPower(turnPowerRight);
          //  right_drive_2.setPower(turnPowerRight);
            left_drive.setPower(turnPowerLeft);
           // left_drive_2.setPower(turnPowerLeft);

            if (turnPowerRight <0.05) {
                right_drive.setPower(0);
          //      right_drive_2.setPower(0);
            }

            if (turnPowerLeft <0.05){
                left_drive.setPower(0);
             //   left_drive_2.setPower(0);
            }

            //spinnyPower = gamepad1.dpad_up;

            if (gamepad1.dpad_up){
                spinny_thing.setPower(0.4);
            }

            if (gamepad1.dpad_left){
                spinny_thing.setPower(0.25);
            }

            if (gamepad1.dpad_right){
                spinny_thing.setPower(0.3);
            }

            if (gamepad1.dpad_down){
                spinny_thing.setPower(0);
            }

        }
    }
}

