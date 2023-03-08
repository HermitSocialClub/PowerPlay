package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit.AprilTagsWrapper;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit.LinearHelpers;
import org.firstinspires.ftc.teamcode.hermitsocialclub.trajectorysequence.TrajectorySequence;

@Autonomous (name = "ChampAutoMirrored")
public class ChampAutoMirrored extends LinearOpMode {
    AprilTagsWrapper vision;
    public bigWheelOdoMecanum drive;

    public LinearHelpers linearHelpers;

    @Override
    public void runOpMode() throws InterruptedException {

        bigWheelOdoMecanum drive = new bigWheelOdoMecanum(hardwareMap);
        linearHelpers = new LinearHelpers(drive, hardwareMap, telemetry);
        vision = new AprilTagsWrapper(hardwareMap, telemetry);
        vision.initialize();
        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        //april tags starts here
        while (!isStarted() && !isStopRequested()) {
            vision.runVision();
            sleep(20);
        }
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        vision.lastSnapshot();

        Pose2d starting = new Pose2d(-32.5, 63, Math.toRadians(-90));
        drive.setPoseEstimate(starting);
        Trajectory toFirstPole = drive.trajectoryBuilder(new Pose2d(starting.vec(),Math.toRadians(-90)),Math.toRadians(-90))
                .splineTo(new Vector2d(-36,28),m(-90))
                .splineToSplineHeading(new Pose2d(-34.25,6.25,m(-45)),m(-90))
                .addDisplacementMarker(1, ()->{
                    linearHelpers.setLinearHeight(2170);
                })
                .build();

        Trajectory toStackFromHighPole = drive.trajectoryBuilder(toFirstPole.end(), m(90))
                .splineToSplineHeading(new Pose2d(50, 12.5, m(0)), m(-5))
                .splineToSplineHeading(new Pose2d(60.8,15,m(2)),m(0))
                .build();

                Trajectory backToJunction = drive.trajectoryBuilder(toStackFromHighPole.end())
                        .back(1)
                        .splineToSplineHeading(new Pose2d(43,14,m(-90)),m(0))
                        .addDisplacementMarker(1,()->{
                            linearHelpers.setLinearHeight(2170);
                        })
                        .splineToSplineHeading(new Pose2d(34.25,7.75,m(-135)),m(-150))
                        .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(toFirstPole.end())
                .turn(Math.toRadians(135))
                .forward(23)
                .strafeLeft(10)
                .build();
        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(toFirstPole.end())
                .turn(Math.toRadians(135))
                .back(26)
                .strafeLeft(5)
                .build();
        TrajectorySequence middlePark = drive.trajectorySequenceBuilder(toFirstPole.end())
                .turn(Math.toRadians(135))
                .back(10)
                .build();


        waitForStart();
        if(isStopRequested()) return;

        linearHelpers.closeClaw();
        sleep(1000);
        drive.followTrajectory(toFirstPole);
        linearHelpers.setLinearHeight(1000);
        linearHelpers.openClaw();
        sleep(500);
         linearHelpers.setLinearHeight (350);  // approx 55 ticks per inch
        drive.followTrajectory(toStackFromHighPole);
        sleep(500);
        linearHelpers.closeClaw();
        sleep(500);
        linearHelpers.setLinearHeight(700);
        sleep(500);
        drive.followTrajectory(backToJunction);
        linearHelpers.setLinearHeight(1000);
        linearHelpers.openClaw();
//        sleep(500);
//        linearHelpers.setLinearHeight(280);
//        drive.followTrajectory(toStackFromHighPole);
//        sleep(500);
//        linearHelpers.closeClaw();
//        sleep(500);
//        linearHelpers.setLinearHeight(700);
//        sleep(500);
//        drive.followTrajectory(backToJunction);
//        linearHelpers.setLinearHeight(1000);
        linearHelpers.openClaw();
        linearHelpers.setLinearHeight(0);

        if (vision.tagOfInterest == null || vision.tagOfInterest.id == vision.LEFT) {

            drive.followTrajectorySequence(leftPark);
//
//            telemetry.addData("tag null or left", vision.tagOfInterest.id);
//
        } else if (vision.tagOfInterest.id == vision.MIDDLE) {

            drive.followTrajectorySequence(middlePark);
//            telemetry.addData("tag middle", vision.tagOfInterest.id);
//
        } else {
//
            drive.followTrajectorySequence(rightPark);
//
//            telemetry.addData("tag of interest right", vision.tagOfInterest.id);
//
        }
    }

    public static double m(double heading) {

        return Math.toRadians(heading);
    }

}

