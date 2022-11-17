package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.SampleMecanumDrive;

@Autonomous (name = "LocationOneBlueAuto")
public class LocationOneBlueAuto extends LinearOpMode {

    public DcMotor linears;
    public CRServo intakeThing;

    @Override
    public void runOpMode() throws InterruptedException {

        linears = hardwareMap.get(DcMotor.class,"linears");
        intakeThing = hardwareMap.get(CRServo.class,"intakeThing");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory t1 = drive.trajectoryBuilder(new Pose2d())
                .forward(20)
                .addDisplacementMarker(() -> {
                    linearsMove(0.2,1);
                    intakeMove(-0.9,1);
                })
                .strafeRight(40)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(t1);
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
            intakeThing.setPower(speed);
        }
    }
}
