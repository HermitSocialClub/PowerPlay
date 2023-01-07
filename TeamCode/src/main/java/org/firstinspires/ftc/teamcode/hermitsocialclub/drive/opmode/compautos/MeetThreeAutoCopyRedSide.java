package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hermitsocialclub.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Meet3AutoCopyRedSide")
public class MeetThreeAutoCopyRedSide extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public bigWheelOdoMecanum drive;

    private ElapsedTime linearTime;

    static final double FEET_PER_METER = 3.28084;

    int cpr = 28; //counts per rotation
    int gear_ratio = 40;
    double diameter = 4.125;
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


        Pose2d ToFirstCone = new Pose2d(35, -63, Math.toRadians(90));
        Pose2d ToFirstConeStrafe = new Pose2d(35,-63,Math.toRadians(0));
        Pose2d ToFirstConeBack = new Pose2d(35, -60, Math.toRadians(-90));
        drive.linears.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //int initposition = drive.linears.getCurrentPosition();
        drive.setPoseEstimate(ToFirstConeBack);


        Trajectory traj1 = drive.trajectoryBuilder(ToFirstCone, Math.toRadians(90))
                .splineTo(new Vector2d(35, -26), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(40, -4, Math.toRadians(50)), Math.toRadians(90))
                .addDisplacementMarker(5, () -> {
                    //drive.linearsMoveUp(4300,0.5);
                    drive.linearsMoveUp(1300,.5);
                })
                .addDisplacementMarker(13,() -> {
                    drive.linearsMoveDown(1000,.5);
                    //drive.linearsMoveDown(500,0.5);
                })
                .addDisplacementMarker(() -> {
                    drive.claw.setPower(-1);
                    drive.linears.setPower(0.05);
                    //drive.linears.setPower(0);
                })
                .build();

        Trajectory traj1butstrafe = drive.trajectoryBuilder(ToFirstConeStrafe, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(35,-20,Math.toRadians(0)),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(40,-4,Math.toRadians(50)),Math.toRadians(50))
                .addDisplacementMarker(10, () -> {
                    //drive.linearsMoveUp(4300,0.5);
                    drive.linearsMoveUp(4300,.5);
                })
                .addDisplacementMarker(17,() -> {
                    drive.linearsMoveDown(1000,.5);
                    //drive.linearsMoveDown(500,0.5);
                })
                .addDisplacementMarker(() -> {
                    drive.claw.setPower(-1);
                    drive.linears.setPower(0.05);
                    //drive.linears.setPower(0);
                })
                .build();

        Trajectory traj1butbackfirst = drive.trajectoryBuilder(ToFirstConeBack, Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(35,-33,Math.toRadians(-90)),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(35,-23,Math.toRadians(0)),Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(42,-4,Math.toRadians(50)),Math.toRadians(50))
                .addDisplacementMarker(5, () -> {
                    //drive.linearsMoveUp(4300,0.5);
                    drive.linearsMoveUp(8300,.5);

                })
                .addDisplacementMarker(20,() -> {
                   // drive.linearsMoveDown(1000,.5);
                    drive.linearsMoveDown(500,0.5);

                })
                .addDisplacementMarker(() -> {
                    drive.claw.setPower(-1);
                    drive.linears.setPower(0.05);
                    //drive.linears.setPower(0);
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(traj1.end().vec(), Math.toRadians(50)), Math.toRadians(160))
                .addDisplacementMarker(() -> {
                    drive.linearsMoveDown(500, .5);
                })
                .splineToSplineHeading(new Pose2d(20, -14, Math.toRadians(180)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(8, -11.5, Math.toRadians(180)), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    drive.claw.setPower(1);
                })
                .build();
        Trajectory traj3 = drive.trajectoryBuilder(traj2.end(), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(20, -14, Math.toRadians(180)), Math.toRadians(350))
                .addDisplacementMarker(()->{
                    drive.linearsMoveUp(1000,.5);
                })
                // .splineToSplineHeading(traj2.start(),Math.toRadians(340))
                //      .forward(1.5)
                .splineToSplineHeading(new Pose2d(40, -8, Math.toRadians(50)), Math.toRadians(160))
                .addDisplacementMarker(()->{
                    drive.claw.setPower(-1);
                })
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end())
                .forward(2)
                .build();

        Trajectory middlePark = drive.trajectoryBuilder(traj3.end())
                .splineToSplineHeading(new Pose2d(35,-13,Math.toRadians(90)),Math.toRadians(90))
                .build();

        Trajectory frontPark = drive.trajectoryBuilder(middlePark.end())
                .splineToConstantHeading(new Vector2d(60,-10),Math.toRadians(0))
                .build();

        Trajectory bottomPark = drive.trajectoryBuilder(middlePark.end())
                .splineToConstantHeading(new Vector2d(15,-10),Math.toRadians(0))
                .build();

//    Trajectory myTrajectory=drive.trajectoryBuilder(new Pose2d())
            /*.strafeLeft(10)
            .forward(60)
            .build();*/

        waitForStart();

        if (isStopRequested()) return;
        resetRuntime();
        drive.claw.setPower(1);
        while (getRuntime() < .500){}
        drive.followTrajectory(traj1butbackfirst);

       // drive.followTrajectory(traj1butstrafe);
        //drive.followTrajectory(traj1);
        // drive.turn(Math.toRadians(180));
//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
        // drive.followTrajectory(traj4);
        // drive.turn(Math.toRadians(190));
//        drive.followTrajectory(traj2);
//        drive.followTrajectory(traj3);
    /*drive.followTrajectory(myTrajectory);
    drive.turn(Math.toRadians(80));*/

        if (tagOfInterest == null || tagOfInterest.id == LEFT) {

//            drive.followTrajectory(middlePark);
//            drive.followTrajectory(frontPark);
            telemetry.addData("tag null or left", tagOfInterest.id);

        } else if (tagOfInterest.id == MIDDLE) {

//            drive.followTrajectory(middlePark);
            telemetry.addData("tag middle", tagOfInterest.id);

        } else {

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

//    public void linearsToTime (double timeout, double speed){
//        linearTime.reset();
//        while (linearTime.seconds() < timeout){
//            drive.linears.setPower(speed);
//        }
//        drive.linears.setPower(0);
//
//    }

}
