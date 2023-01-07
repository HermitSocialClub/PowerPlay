package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hermitsocialclub.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Meet3AutoButTheNewRobotThisTime")
public class Meet3AutoButTheNewRobotThisTime extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public bigWheelOdoMecanum drive;
    public Servo claw = null;
    public org.firstinspires.ftc.teamcode.util.Encoder leftEncoder;
    public org.firstinspires.ftc.teamcode.util.Encoder frontEncoder;

    private ElapsedTime linearTime;

    static final double FEET_PER_METER = 3.28084;

    int cpr = 28; //counts per rotation
    int gear_ratio = 40;
    double diameter = 6; //4.125;
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
    Pose2d toFirstCone = new Pose2d(-35,65,Math.toRadians(-90));
//    Pose2d toFirstCone2 = new Pose2d(-15,62,-90);

    Trajectory partOne;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new bigWheelOdoMecanum(hardwareMap);
        leftEncoder = new org.firstinspires.ftc.teamcode.util.Encoder(hardwareMap.get(DcMotorEx.class,"leftEncoder"));
        frontEncoder = new org.firstinspires.ftc.teamcode.util.Encoder(hardwareMap.get(DcMotorEx.class,"frontEncoder"));

        linearTime = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
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
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

// put the stuff that does the stuff before parking here
        drive.setPoseEstimate(toFirstCone);
        //drive.claw.setPosition(1);
        drive.linears.setTargetPosition(drive.linears.getCurrentPosition() - 20);
        drive.linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linears.setPower(0.5);
        drive.linears.setTargetPosition(drive.linears.getCurrentPosition() + 20);
        Trajectory arcToFirstCone = drive.trajectoryBuilder(toFirstCone)
                .splineToConstantHeading(new Vector2d(-12,65),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-11.50,50), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-12,20),Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(-44,10), Math.toRadians(180))
                .addDisplacementMarker(30, () -> {
                    drive.linears.setTargetPosition(drive.linears.getCurrentPosition()
                            + 4300);
                    drive.linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.linears.setPower(.55);

                })
                .addDisplacementMarker(() -> {
                    int x = drive.linears.getCurrentPosition();
                    drive.linears.setTargetPosition(drive.linears.getCurrentPosition()
                            - 500);
                })
                .addDisplacementMarker(() -> {
                    drive.claw.setPower(0.65);
                    //drive.linears.setPower(0);
                })
//                .strafeRight(10)
//                .forward(15)
                .build();

        Trajectory straightToFirstJunction = drive.trajectoryBuilder(arcToFirstCone.end())
                .splineToConstantHeading(new Vector2d(-13,20),Math.toRadians(-90))
                        .build();

        Trajectory arcAfterStraightToFirstJunction = drive.trajectoryBuilder(straightToFirstJunction.end())
                .splineToConstantHeading(new Vector2d(-25,10), Math.toRadians(180))
                .addDisplacementMarker(() -> {
                    int x = drive.linears.getCurrentPosition();
                    drive.linears.setTargetPosition(drive.linears.getCurrentPosition()
                            - 500);
                })
                .addDisplacementMarker(() -> {
                    drive.claw.setPower(-1);
                    //drive.linears.setPower(0);
                })
                .build();

        Trajectory toConeStackButStraight = drive.trajectoryBuilder(new Pose2d(arcAfterStraightToFirstJunction.end().vec(),Math.toRadians(180)))
                .lineToConstantHeading(new Vector2d(-60,12))
                .build();

        Trajectory reversedToConeStackButStraight = drive.trajectoryBuilder(toConeStackButStraight.start(),true)
                .lineToConstantHeading(new Vector2d(-60,12))
                        .build();

//        Trajectory toConeStack = drive.trajectoryBuilder(arcAfterStraightToFirstJunction.end())
//                .splineToSplineHeading(new Pose2d(-60,12,Math.toRadians(180)),Math.toRadians(195))
//                .build();
//
//        Trajectory backToPole = drive.trajectoryBuilder(toConeStack.end())
//                .splineToSplineHeading(new Pose2d(-25,10,Math.toRadians(-90)),Math.toRadians(200))
//                .build();
//
//        Trajectory backToCones = drive.trajectoryBuilder(backToPole.end())
//                .splineToSplineHeading(new Pose2d(-60,12,Math.toRadians(180)),Math.toRadians(195))
//                .build();
//
//        Trajectory backToPoleTwo = drive.trajectoryBuilder(backToCones.end())
//                .splineToSplineHeading(new Pose2d(-25,10,Math.toRadians(-90)),Math.toRadians(200))
//                .build();

        waitForStart();
        if(isStopRequested()) return;

        drive.claw.setPower(-1);
        drive.followTrajectory(arcToFirstCone);
//        drive.followTrajectory(straightToFirstJunction);
//        drive.followTrajectory(arcAfterStraightToFirstJunction); //1
        //drive.turn(Math.toRadians(-95));
        drive.followTrajectory(toConeStackButStraight); //2
        drive.claw.setPower(-1);
//        drive.followTrajectory(reversedToConeStackButStraight); //2
//        drive.turn(-90); //2
//        drive.followTrajectory(toConeStackButStraight); //3
//        drive.followTrajectory(reversedToConeStackButStraight); //3
//        drive.turn(-90); //3
//        drive.followTrajectory(toConeStackButStraight); //4
//        drive.followTrajectory(reversedToConeStackButStraight); //4
//        drive.turn(-90); //4
//        drive.followTrajectory(toConeStackButStraight); //5
//        drive.followTrajectory(reversedToConeStackButStraight); //5
//        drive.turn(-90); //5
//        drive.followTrajectory(toConeStackButStraight); //6
//        drive.followTrajectory(reversedToConeStackButStraight); //6
//        drive.turn(-90); //6

//        drive.followTrajectory(toConeStack); //2
//        drive.followTrajectory(backToPole); //2
//        drive.followTrajectory(backToCones); //3
//        drive.followTrajectory(backToPoleTwo); //3
//        drive.followTrajectory(backToCones); //4
//        drive.followTrajectory(backToPoleTwo);  //4
//        drive.followTrajectory(backToCones); //5
//        drive.followTrajectory(backToPoleTwo); //5
//        drive.followTrajectory(backToCones); //6
//        drive.followTrajectory(backToPoleTwo); //6


        /* Actually do something useful */
     /*   if(tagOfInterest == null || tagOfInterest.id == LEFT) {

          //  drive.followTrajectory(tmiddle);
//            drive.followTrajectory(tfront);

            telemetry.addData("tag null or left", tagOfInterest.id);
        }else if(tagOfInterest.id == MIDDLE){

            drive.followTrajectory(ToTwo);

            telemetry.addData("tag middle",tagOfInterest.id);
        }else{
            drive.followTrajectory(ToThree);
            //drive.followTrajectory(tmiddle);
//            drive.followTrajectory(tback);
            //drive.followTrajectory(tbackstrafe);
           // drive.turn(Math.toRadians(90));
           // drive.followTrajectory(tbacktwo);
*/

        telemetry.addData("tag of interest right",tagOfInterest.id);
        resetRuntime();
    }

    /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
    // while (opModeIsActive()) {sleep(20);}
    //}

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
    public void linearsToTime (double timeout, double speed){
        linearTime.reset();
        while (linearTime.seconds() < timeout){
            drive.linears.setPower(speed);
        }
        drive.linears.setPower(0);

    }
}
