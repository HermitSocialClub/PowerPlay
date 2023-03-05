package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit.AprilTagsWrapper;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit.LinearHelpers;
import org.firstinspires.ftc.teamcode.hermitsocialclub.trajectorysequence.TrajectorySequence;

@Disabled
@Autonomous (name = "ChampionshipsAuto")
public class ChampionshipsAuto extends LinearOpMode {
    AprilTagsWrapper vision;
    public bigWheelOdoMecanum drive;

    LinearHelpers linearHelpers;

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

        Pose2d starting = new Pose2d(32.5, 63, Math.toRadians(-90));
        drive.setPoseEstimate(starting);
        Trajectory toFirstPole = drive.trajectoryBuilder(new Pose2d(starting.vec(),Math.toRadians(-90)),Math.toRadians(-90))
                .splineTo(new Vector2d(36,28),m(-90))
                .splineToSplineHeading(new Pose2d(34.25,6.25,m(-135)),m(-90))
                .addDisplacementMarker(0.5, ()->{
                    linearHelpers.setLinearHeight(2170);  // previously 4000
                    //drive.claw.setPosition(1.0);  // prev 0
                    //TODO find tics per inch for linears
                })
//                .addDisplacementMarker(()->{
//                    telemetry.addData("working", false);
//                    linearHelpers.waitForLinears();
//                    telemetry.addData("working", true);
//                    sleep(1000);
//                    linearHelpers.setLinearHeight(1900);
//                    linearHelpers.openClaw();
//                    sleep(1000);
//                    linearHelpers.setLinearHeight (360);
//                })
              //  .splineToConstantHeading(new Vector2d(36,12),m(-90))
              //  .splineTo(new Vector2d(32, 8), m(-135))
              //  .splineTo(new Vector2d(36, 28), m(-135))
                .build();

        Trajectory toStackFromHighPole = drive.trajectoryBuilder(toFirstPole.end(), m(90))
               // .splineToSplineHeading(new Pose2d(60, 12, m(0)), m(-5))
                .splineToSplineHeading(new Pose2d(50, 12.5, m(0)), m(-5))
                .splineToSplineHeading(new Pose2d(60.75,14,m(2)),m(0))
                .build();
/*        TrajectorySequence backAfterFirstSequence = drive.trajectorySequenceBuilder(autonSequence.end())
                .back(0.25)
                .addDisplacementMarker(()->{
                    linearsMoveAuto(1000);
                    drive.claw.setPosition(0);
                })
                .build();
*/
                Trajectory toCones1 = drive.trajectoryBuilder(toFirstPole.end())
                       // .splineToConstantHeading(new Vector2d(32))
                        .splineToSplineHeading(new Pose2d(60,13,m(0)),m(0))
                        .build();

                Trajectory toJunction = drive.trajectoryBuilder(toCones1.end())
                        .splineToSplineHeading(new Pose2d(32,6.5,m(-135)),m(0))
                        .addDisplacementMarker(()->{
                            linearHelpers.setLinearHeight(2170);
                        })
                        .build();

                Trajectory backToJunction = drive.trajectoryBuilder(toStackFromHighPole.end())
                        .back(1)
                        .splineToSplineHeading(new Pose2d(43,14,m(-90)),m(0))
                        .addDisplacementMarker(1,()->{
                            linearHelpers.setLinearHeight(2170);
                        })
                        .splineToSplineHeading(new Pose2d(34.25,7.75,m(-135)),m(-150))
                        .build();

           /*     Trajectory toCones2 = drive.trajectoryBuilder(toJunction.end())
                        .splineToSplineHeading(new Pose2d(60,13,m(0)),m(0))
                        .addDisplacementMarker(9,()->{
                            linearsMoveAuto(-3000);
                        })
                        .addDisplacementMarker(()->{
                            drive.claw.setPosition(0.8);
                            linearsMoveAuto(3000);
                        })
                        .build();*/
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

        TrajectorySequence wait = drive.trajectorySequenceBuilder(toStackFromHighPole.end())
                .waitSeconds(0.5)
                .build();

//        Trajectory firstCone = drive.trajectoryBuilder(starting)
//                .forward(52.5)
//                .build();
//        Trajectory inchForward = drive.trajectoryBuilder(new Pose2d(firstCone.end().getX(), firstCone.end().getY(), Math.toRadians(45)))
//                .addDisplacementMarker(() -> {
//                    drive.linearsMoveUp(4200, 0.7);
//                })
//                .forward(2)
//                .build();
//        Trajectory inchBack = drive.trajectoryBuilder(inchForward.end())
//                .back(2)
//                .addDisplacementMarker(() -> {
//                    drive.linearsMoveDown(4200, 0.7);
//                })
//                .build();
//        Trajectory drivetoCone = drive.trajectoryBuilder(new Pose2d(inchBack.end().getX(), inchBack.end().getY(), Math.toRadians(-90)))
//                .forward(20)
//                .build();
//        Trajectory driveBackFromCone = drive.trajectoryBuilder(drivetoCone.end())
//                .back(20)
//                .build();
//        Trajectory turn90 = drive.trajectoryBuilder(firstCone.end())
//                        .
        waitForStart();
        if(isStopRequested()) return;
       // initialLinears = linears.getCurrentPosition();
//        drive.claw.setPosition(1.0);  // 0.8
        linearHelpers.closeClaw();
        sleep(1000);
        drive.followTrajectory(toFirstPole);
        //drive.followTrajectorySequence(linearsMoveAndStuff);
//        telemetry.addData("working", false);
//        linearHelpers.waitForLinears();
//        telemetry.addData("working", true);
        linearHelpers.setLinearHeight(1000);
//        linearHelpers.waitForLinears();
        linearHelpers.openClaw();
        sleep(500);
         linearHelpers.setLinearHeight (350);  // approx 55 ticks per inch
        drive.followTrajectory(toStackFromHighPole);
        //drive.followTrajectory(toCones1);
      //  linearHelpers.waitForLinears();
        sleep(500);
        linearHelpers.closeClaw();
        sleep(500);
        //drive.followTrajectorySequence(wait);
        linearHelpers.setLinearHeight(700);
      //  linearHelpers.waitForLinears();
        sleep(500);
        //drive.followTrajectorySequence(wait);
        drive.followTrajectory(backToJunction);
        linearHelpers.setLinearHeight(1000);
        //telemetry.addData("working", false);
    //    linearHelpers.waitForLinears();
      //  telemetry.addData("working", true);
        linearHelpers.openClaw();
        sleep(500);
        linearHelpers.setLinearHeight(280);
        drive.followTrajectory(toStackFromHighPole);
        sleep(500);
        linearHelpers.closeClaw();
        sleep(500);
        linearHelpers.setLinearHeight(700);
        sleep(500);
        drive.followTrajectory(backToJunction);
        linearHelpers.setLinearHeight(1000);
        linearHelpers.openClaw();
        linearHelpers.setLinearHeight(0);
     //   linearHelpers.waitForLinears();


       // drive.followTrajectory(toCones1);
     //   drive.followTrajectory(toJunction);
     //   drive.followTrajectory(toCones2);
    //    drive.followTrajectory(toJunction);

//        drive.followTrajectory(firstCone);
//        drive.turn(Math.toRadians(-45));
//        drive.followTrajectory(inchForward);
//        //score
//        drive.followTrajectory(inchBack);
//        drive.turn(Math.toRadians(-135));
//        drive.followTrajectory(drivetoCone);
//        drive.followTrajectory(strafeToCone);
//        drive.turn(180);
//        drive.followTrajectory(goToStack);

  /*      if (vision.tagOfInterest == null || vision.tagOfInterest.id == vision.LEFT) {

            drive.followTrajectorySequence(leftPark);
//
////            drive.followTrajectory(middlePark);
////            drive.followTrajectory(frontPark);
//            telemetry.addData("tag null or left", vision.tagOfInterest.id);
//
        } else if (vision.tagOfInterest.id == vision.MIDDLE) {

            drive.followTrajectorySequence(middlePark);
////            drive.followTrajectory(middlePark);
//            telemetry.addData("tag middle", vision.tagOfInterest.id);
//
        } else {
//
            drive.followTrajectorySequence(rightPark);
//
////            drive.followTrajectory(middlePark);
////            drive.followTrajectory(bottomPark);
//            telemetry.addData("tag of interest right", vision.tagOfInterest.id);
//
        }*/
    }
//    public void linearsMoveAuto (int distance){
//        linears.setTargetPosition(linears.getCurrentPosition()+distance);
//        linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        // 3 below
//        linears.setPower(0.9);
//    }

//    public void linearsMoveAuto (double targetPos){
//
//        linears.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        double Kp = 0.5;
//        double Ki = 0;
//        double Kd = 0.05;
//
//
//        double reference = 1;
//        double lastReference = reference;
//        double integralSum = 0;
//
//        double lastError = 0;
//
//        double maxIntegralSum = 0;
//
//        double a = 0.8; // a can be anything from 0 < a < 1
//        double previousFilterEstimate = 0;
//        double currentFilterEstimate = 0;
//
//        // Elapsed timer class from SDK, please use it, it's epic
//        ElapsedTime timer = new ElapsedTime();
//
//// while (linears.getCurrentPosition()<targetPos) {
//
//            if (linears.getCurrentPosition() < targetPos) {
//
//                // obtain the encoder position
//                double encoderPosition = linears.getCurrentPosition();
//                // calculate the error
//                double error = reference - encoderPosition;
//
//                double errorChange = (error - lastError);
//
//                // filter out hight frequency noise to increase derivative performance
//                currentFilterEstimate = (a * previousFilterEstimate) + (1 - a) * errorChange;
//                previousFilterEstimate = currentFilterEstimate;
//
//                // rate of change of the error
//                double derivative = currentFilterEstimate / timer.seconds();
//
//                // sum of all error over time
//                integralSum = integralSum + (error * timer.seconds());
//
//
//                // max out integral sum
//                if (integralSum > maxIntegralSum) {
//                    integralSum = maxIntegralSum;
//                }
//
//                if (integralSum < -maxIntegralSum) {
//                    integralSum = -maxIntegralSum;
//                }
//
//                // reset integral sum upon setpoint changes
//                if (reference != lastReference) {
//                    integralSum = 0;
//                }
//
//                double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
//
//                linears.setPower(out);
//
//                lastError = error;
//
//                lastReference = reference;
//
//                // reset the timer for next time
//                timer.reset();
//
//            } else {
//                linears.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            }
//        }

//    public void linearsToTime (double timeout, double speed){
//        linearTime.reset();
//        while (linearTime.seconds() < timeout){
//            drive.linears.setPower(speed);
//        }
//        drive.linears.setPower(0);
//
//    }
    public static double m(double heading) {
        return Math.toRadians(heading);
    }

}
//public void oldAuton () {
//    Pose2d ToFirstCone = new Pose2d(35, -63, Math.toRadians(90));
//    Pose2d ToFirstConeStrafe = new Pose2d(35,-63,Math.toRadians(0));
//    Pose2d ToFirstConeBack = new Pose2d(35, -60, Math.toRadians(-90));
//    drive.linears.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    //int initposition = drive.linears.getCurrentPosition();
//    drive.setPoseEstimate(ToFirstConeBack);
//
//
//    Trajectory traj1 = drive.trajectoryBuilder(ToFirstCone, Math.toRadians(90))
//            .splineTo(new Vector2d(35, -26), Math.toRadians(90))
//            .splineToSplineHeading(new Pose2d(40, -4, Math.toRadians(50)), Math.toRadians(90))
//            .addDisplacementMarker(5, () -> {
//                //drive.linearsMoveUp(4300,0.5);
//                drive.linearsMoveUp(1300,.5);
//            })
//            .addDisplacementMarker(13,() -> {
//                drive.linearsMoveDown(1000,.5);
//                //drive.linearsMoveDown(500,0.5);
//            })
//            .addDisplacementMarker(() -> {
//                drive.claw.setPosition(-1);
//                drive.linears.setPower(0.05);
//                //drive.linears.setPower(0);
//            })
//            .build();
//
//    Trajectory traj1butstrafe = drive.trajectoryBuilder(ToFirstConeStrafe, Math.toRadians(90))
//            .splineToSplineHeading(new Pose2d(35,-20,Math.toRadians(0)),Math.toRadians(90))
//            .splineToSplineHeading(new Pose2d(40,-4,Math.toRadians(50)),Math.toRadians(50))
//            .addDisplacementMarker(10, () -> {
//                //drive.linearsMoveUp(4300,0.5);
//                drive.linearsMoveUp(4300,.5);
//            })
//            .addDisplacementMarker(17,() -> {
//                drive.linearsMoveDown(1000,.5);
//                //drive.linearsMoveDown(500,0.5);
//            })
//            .addDisplacementMarker(() -> {
//                drive.claw.setPosition(-1);
//                drive.linears.setPower(0.05);
//                //drive.linears.setPower(0);
//            })
//            .build();
//
//    Trajectory traj1butbackfirst = drive.trajectoryBuilder(ToFirstConeBack, Math.toRadians(90))
//            .splineToSplineHeading(new Pose2d(35,-33,Math.toRadians(-90)),Math.toRadians(90))
//            .splineToSplineHeading(new Pose2d(35,-23,Math.toRadians(0)),Math.toRadians(90))
//            .splineToSplineHeading(new Pose2d(42,-4,Math.toRadians(50)),Math.toRadians(50))
//            .addDisplacementMarker(5, () -> {
//                //drive.linearsMoveUp(4300,0.5);
//                drive.linearsMoveUp(8300,.5);
//
//            })
//            .addDisplacementMarker(20,() -> {
//                // drive.linearsMoveDown(1000,.5);
//                drive.linearsMoveDown(500,0.5);
//
//            })
//            .addDisplacementMarker(() -> {
//                drive.claw.setPosition(-1);
//                drive.linears.setPower(0.05);
//                //drive.linears.setPower(0);
//            })
//            .build();
//
//    Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().vec(), Math.toRadians(50)), Math.toRadians(160))
//            .addDisplacementMarker(() -> {
//                drive.linearsMoveDown(500, .5);
//            })
//            .splineToSplineHeading(new Pose2d(20, -14, Math.toRadians(180)), Math.toRadians(180))
//            .splineToSplineHeading(new Pose2d(8, -11.5, Math.toRadians(180)), Math.toRadians(180))
//            .addDisplacementMarker(() -> {
//                drive.claw.setPosition(1);
//            })
//            .build();
//    Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), Math.toRadians(0))
//            .splineToSplineHeading(new Pose2d(20, -14, Math.toRadians(180)), Math.toRadians(350))
//            .addDisplacementMarker(()->{
//                drive.linearsMoveUp(1000,.5);
//            })
//            // .splineToSplineHeading(traj2.start(),Math.toRadians(340))
//            //      .forward(1.5)
//            .splineToSplineHeading(new Pose2d(40, -8, Math.toRadians(50)), Math.toRadians(160))
//            .addDisplacementMarker(()->{
//                drive.claw.setPosition(-1);
//            })
//            .build();
//
//    Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
//            .forward(2)
//            .build();
//
//    Trajectory middlePark = drive.trajectoryBuilder(traj3.end())
//            .splineToSplineHeading(new Pose2d(35,-13,Math.toRadians(90)),Math.toRadians(90))
//            .build();
//
//    Trajectory frontPark = drive.trajectoryBuilder(middlePark.end())
//            .splineToConstantHeading(new Vector2d(60,-10),Math.toRadians(0))
//            .build();
//
//    Trajectory bottomPark = drive.trajectoryBuilder(middlePark.end())
//            .splineToConstantHeading(new Vector2d(15,-10),Math.toRadians(0))
//            .build();
//
////    Trajectory myTrajectory=drive.trajectoryBuilder(new Pose2d())
//            /*.strafeLeft(10)
//            .forward(60)
//            .build();*/
//
//    waitForStart();
//
//    if (isStopRequested()) return;
//    resetRuntime();
//    drive.claw.setPosition(1);
//    while (getRuntime() < .500){}
//    drive.followTrajectory(traj1butbackfirst);
//
//    // drive.followTrajectory(traj1butstrafe);
//    //drive.followTrajectory(traj1);
//    // drive.turn(Math.toRadians(180));
////        drive.followTrajectory(traj2);
////        drive.followTrajectory(traj3);
//    // drive.followTrajectory(traj4);
//    // drive.turn(Math.toRadians(190));
////        drive.followTrajectory(traj2);
////        drive.followTrajectory(traj3);
//    /*drive.followTrajectory(myTrajectory);
//    drive.turn(Math.toRadians(80));*/
//}

