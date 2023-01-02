package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;

@Autonomous (name = "Meet3AutoCopy")
public class Meet3AutoCopy extends LinearOpMode {
@Override

    public void runOpMode() throws InterruptedException {

    bigWheelOdoMecanum drive = new bigWheelOdoMecanum(hardwareMap);
   Pose2d ToFirstCone = new Pose2d(35,63,Math.toRadians(-90));
    drive.setPoseEstimate(ToFirstCone);
    Trajectory traj1 = drive.trajectoryBuilder(ToFirstCone)
            .splineTo(new Vector2d(35,26), Math.toRadians(-90))
            .splineToSplineHeading(new Pose2d(40,8,Math.toRadians(-50)),Math.toRadians(-90))
                    .build();

//    Trajectory myTrajectory=drive.trajectoryBuilder(new Pose2d())
            /*.strafeLeft(10)
            .forward(60)
            .build();*/

    waitForStart();

    if(isStopRequested()) return;

    drive.followTrajectory(traj1);
    /*drive.followTrajectory(myTrajectory);
    drive.turn(Math.toRadians(80));*/

    }
}
