package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hermitsocialclub.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;
import org.firstinspires.ftc.teamcode.hermitsocialclub.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "ILTBottomBlueSide")
public class ILTBottomBlueSide extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public bigWheelOdoMecanum drive;

    private ElapsedTime linearTime;

    static final double FEET_PER_METER = 3.28084;

    int cpr = 28; //counts per rotation
    int gear_ratio = 40;
    double diameter = 4.125;
  //  int initialLinears;
    double cpi = (cpr * gear_ratio) / (Math.PI * diameter); //counts per inch, 28cpr * gear ratio / (2 * pi * diameter (in inches, in the center))
    double bias = 0.8;//default 0.8
    double meccyBias = 0.9;//change to adjust only strafing movement
    double speed = .5;

    Double conversion = cpi * bias;
    Boolean exit = false;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    // need to update because we need our exact location!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    DcMotor linears;

    // UNITS ARE METERS - need to update for our tag size
    double tagsize = 0.04;  // was .166

    // insert our tags of interest here - three or two w default if not seen
    //int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    //Key Positions Blue

//    Pose2d toFirstCone2 = new Pose2d(-15,62,-90);

    Trajectory partOne;

    @Override
    public void runOpMode() throws InterruptedException {

//        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
//        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
//        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
//        right_drive_2.setDirection(DcMotorSimple.Direction.REVERSE);
//        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
//        left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");
        linears = hardwareMap.get(DcMotor.class,"linear");
        linears.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linears.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        bigWheelOdoMecanum drive = new bigWheelOdoMecanum(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        //april tags starts here
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }


       //oldAUTON()
        Pose2d starting = new Pose2d(-36, -65, Math.toRadians(90));
//        double forwarddistance = 65;
        drive.setPoseEstimate(starting);
        TrajectorySequence autonSequence = drive.trajectorySequenceBuilder(starting)
                .forward(52.5)
                .addDisplacementMarker(20, () -> {
                    linearsMoveAuto(4000);
                })
                .turn(Math.toRadians(45))
                .forward(5)
                .addDisplacementMarker(() -> {
                    drive.claw.setPosition(0);
                })
                .waitSeconds(0.5)
                .back(5)
                .waitSeconds(1)
//                .turn(Math.toRadians(-135))
//                .addDisplacementMarker( () -> {
//                    linearsMoveAuto(-3000);
//                })
//                .forward(23)
//                .waitSeconds(1)
//                .addDisplacementMarker( () -> {
//                    drive.claw.setPosition(0.7);
// //                   linearsMoveAuto(1500);
//                })
//                .back(23)
//                .turn(Math.toRadians(135))
//                .addDisplacementMarker(() -> {
//                    linearsMoveAuto(3000);
//                })
//                .forward(3)
//                .addDisplacementMarker(()->{
//                    drive.claw.setPosition(0);
//                })
//                .waitSeconds(2)
//                .back(3)*/
//                .waitSeconds(1)
//                .turn(Math.toRadians(135))
//                .forward(23)
//                .waitSeconds(3)
//                .back(23)
//                .turn(Math.toRadians(-135))
//                .forward(3)
//                .waitSeconds(2)
//                .back(3)
//                .waitSeconds(1)
                .build();

                TrajectorySequence leftPark = drive.trajectorySequenceBuilder(autonSequence.end())
                        .turn(Math.toRadians(-135))
                        .forward(23)
                        .strafeRight(10)
                        .build();

                TrajectorySequence rightPark = drive.trajectorySequenceBuilder(autonSequence.end())
                        .turn(Math.toRadians(-135))
                        .back(26)
                        .strafeRight(5)
                        .build();

                TrajectorySequence middlePark = drive.trajectorySequenceBuilder(autonSequence.end())
                        .turn(Math.toRadians(-45))
                        .back(10)
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
        drive.claw.setPosition(0.7);
        drive.followTrajectorySequence(autonSequence);

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

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {

            drive.followTrajectorySequence(rightPark);

//            drive.followTrajectory(middlePark);
//            drive.followTrajectory(frontPark);
            telemetry.addData("tag null or left", tagOfInterest.id);

        } else if (tagOfInterest.id == MIDDLE) {

            drive.followTrajectorySequence(middlePark);
//            drive.followTrajectory(middlePark);
            telemetry.addData("tag middle", tagOfInterest.id);

        } else {

            drive.followTrajectorySequence(leftPark);

//            drive.followTrajectory(middlePark);
//            drive.followTrajectory(bottomPark);
            telemetry.addData("tag of interest right", tagOfInterest.id);

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public void linearsMoveAuto (int distance){
        linears.setTargetPosition(linears.getCurrentPosition()+distance);
        linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // 3 below
        linears.setPower(0.7);
    }

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

    public void linearsToTime (double timeout, double speed){
        linearTime.reset();
        while (linearTime.seconds() < timeout){
            drive.linears.setPower(speed);
        }
        drive.linears.setPower(0);

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

