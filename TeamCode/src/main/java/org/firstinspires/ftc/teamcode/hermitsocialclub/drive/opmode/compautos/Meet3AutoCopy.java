package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;

@Autonomous (name = "Meet3AutoCopy")
public class Meet3AutoCopy extends LinearOpMode {
@Override

    public void runOpMode() throws InterruptedException {

    bigWheelOdoMecanum drive = new bigWheelOdoMecanum(hardwareMap);
    /*Pose2d startPose = new Pose2d(36,57,Math.toRadians(-90));
    drive.setPoseEstimate(startPose);
    Trajectory traj1 = drive.trajectoryBuilder(startPose)
            .splineTo(new Vector2d(14,-5), Math.toRadians(30))
                    .build();*/

    Trajectory myTrajectory=drive.trajectoryBuilder(new Pose2d())
            //.strafeLeft(10)
            .forward(46.5)
            .build();

    waitForStart();

    if(isStopRequested()) return;

    //drive.followTrajectory(traj1);
    drive.followTrajectory(myTrajectory);
    drive.turn(Math.toRadians(90));

    }
}
