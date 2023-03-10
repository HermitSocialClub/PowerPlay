package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit.AprilTagsWrapper;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit.LinearHelpers;

@Autonomous (name = "ChampAuto")
public class ChampAuto extends LinearOpMode {
    AprilTagsWrapper vision;
    public bigWheelOdoMecanum drive;

    public LinearHelpers linearHelpers;

    DcMotor linears;

    @Override
    public void runOpMode() throws InterruptedException {

        bigWheelOdoMecanum drive = new bigWheelOdoMecanum(hardwareMap);
        linearHelpers = new LinearHelpers(drive, hardwareMap, telemetry);
        vision = new AprilTagsWrapper(hardwareMap, telemetry);
        vision.initialize();
        linears = hardwareMap.get(DcMotor.class,"linear");
        linears.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linears.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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

        Pose2d starting = new Pose2d(32.5, 63, Math.toRadians(-90));
        drive.setPoseEstimate(starting);
        Trajectory toFirstPole = drive.trajectoryBuilder(new Pose2d(starting.vec(),Math.toRadians(-90)),Math.toRadians(-90))
                .splineTo(new Vector2d(36,28),m(-90))
                .splineToSplineHeading(new Pose2d(34.25,6.25,m(-135)),m(-90))
                .addDisplacementMarker(1, ()->{
                   // linearHelpers.setLinearHeight(2170);
                    linearsMoveAuto(4000);
                })
                .build();

        Trajectory toStackFromHighPole = drive.trajectoryBuilder(toFirstPole.end(), m(90))
                .splineToSplineHeading(new Pose2d(50, 12.5, m(0)), m(-5))
                .splineToSplineHeading(new Pose2d(60.75,15.5,m(3)),m(0))
                .build();

        Trajectory toStackFromHighPole2 = drive.trajectoryBuilder(toFirstPole.end(), m(90))
                .splineToSplineHeading(new Pose2d(50, 12.5, m(0)), m(-5))
                .splineToSplineHeading(new Pose2d(60.75,17,m(0)),m(0))
                .build();

        Trajectory toMediumJunctionFromStack = drive.trajectoryBuilder(toStackFromHighPole.end())
                .splineToSplineHeading(new Pose2d(40,11,m(70)),m(0))
                .splineToSplineHeading(new Pose2d(34,18,m(135)),m(-70))
                .addDisplacementMarker(()->{
                    //feel free to uncomment I just needed the build to work
//                    linearHelpers.setLinearHeight();
                })
                .build();

                Trajectory backToJunction = drive.trajectoryBuilder(toStackFromHighPole.end())
                        .back(1)
                        .splineToSplineHeading(new Pose2d(43,14,m(-90)),m(0))
                        .addDisplacementMarker(1,()->{
                            linearHelpers.setLinearHeight(2170);
                        })
                        .splineToSplineHeading(new Pose2d(34.25,6.25,m(-135)),m(-150))
                        .build();

      /*  TrajectorySequence leftPark = drive.trajectorySequenceBuilder(toFirstPole.end())
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
                .build(); */
        Trajectory toMiddlePark = drive.trajectoryBuilder(toFirstPole.end())
                .splineToSplineHeading(new Pose2d(34.25,25,m(-90)),m(-90))
                .build();
        Trajectory toRightPark = drive.trajectoryBuilder(toFirstPole.end(),m(0))
                .splineToSplineHeading(new Pose2d(25,12,m(0)),m(0))
                .splineToConstantHeading(new Vector2d(12,12),m(0))
                .build();


        waitForStart();
        if(isStopRequested()) return;

        //linearHelpers.closeClaw();
        drive.claw.setPosition(1);
        sleep(1000);
        drive.followTrajectory(toFirstPole);
        linearHelpers.setLinearHeight(1000);
       // linearHelpers.openClaw();
        drive.claw.setPosition(0);
        sleep(500);
         linearHelpers.setLinearHeight (350);  // approx 55 ticks per inch
       drive.followTrajectory(toStackFromHighPole);
       // drive.followTrajectory(toMediumJunctionFromStack);
        sleep(500);
       // linearHelpers.closeClaw();
        drive.claw.setPosition(1);
        sleep(500);
        linearHelpers.setLinearHeight(700);
        sleep(500);
        drive.followTrajectory(backToJunction);
        linearHelpers.setLinearHeight(1000);
        drive.claw.setPosition(0);
     //   linearHelpers.openClaw();
        sleep(500);
        linearHelpers.setLinearHeight(270);
        drive.followTrajectory(toStackFromHighPole2);
        sleep(600);
        drive.claw.setPosition(1);
//        linearHelpers.closeClaw();
        sleep(700);
        linearHelpers.setLinearHeight(650);
         sleep(500);
        drive.followTrajectory(backToJunction);
        linearHelpers.setLinearHeight(1000);
        drive.claw.setPosition(0);
       // linearHelpers.openClaw();
        linearHelpers.setLinearHeight(0);

        if (vision.tagOfInterest == null || vision.tagOfInterest.id == vision.LEFT) {

           // drive.followTrajectorySequence(leftPark);
            drive.followTrajectory(toStackFromHighPole);
//
            telemetry.addData("tag null or left", vision.tagOfInterest.id);
//
        } else if (vision.tagOfInterest.id == vision.MIDDLE) {

//            drive.followTrajectorySequence(middlePark);
//            telemetry.addData("tag middle", vision.tagOfInterest.id);
            drive.followTrajectory(toMiddlePark);
//
        } else {
//
        //    drive.followTrajectorySequence(rightPark);
//
            telemetry.addData("tag of interest right", vision.tagOfInterest.id);
//
            drive.followTrajectory(toRightPark);
        }
    }

    public static double m(double heading) {

        return Math.toRadians(heading);
    }

    public void linearsMoveAuto (int distance){
        linears.setTargetPosition(linears.getCurrentPosition()+distance);
        linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // 3 below
        linears.setPower(0.7);
    }

}

