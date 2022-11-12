package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.SampleMecanumDrive;

@Autonomous (name = "LocationOneRedAuto")
public class LocationOneRedAuto extends LinearOpMode {

    public DcMotor linears;
    public Servo intakeThing;

    @Override
    public void runOpMode() throws InterruptedException {

        linears = hardwareMap.get(DcMotor.class,"linears");
        intakeThing = hardwareMap.get(Servo.class,"intakeThing");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory myTrajectory = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .addDisplacementMarker(() -> {
                    linearsMove(0.2,1);
                    intakeThing.setPosition(1);
                })
                .strafeLeft(40)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(myTrajectory);
    }

    public void linearsMove (double speed, double time){
        double start = System.currentTimeMillis();
        double end = start + time * 1000;
        while (System.currentTimeMillis() < end) {
            linears.setPower(speed);
        }
    }

    public void intakeMove (double speed, double time){
        double start = System.currentTimeMillis();
        double end = start + time * 1000;
        while (System.currentTimeMillis() < end) {
            intakeThing.setPosition(speed);
        }
    }
}
