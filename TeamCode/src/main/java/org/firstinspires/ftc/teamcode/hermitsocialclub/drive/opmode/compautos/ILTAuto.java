package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.compautos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hermitsocialclub.apriltags.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "ILTAuto")
public class ILTAuto extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public bigWheelOdoMecanum drive;

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


        Pose2d ToFirstCone = new Pose2d(-35, 65, Math.toRadians(90));
      //  drive.linears.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // int initposition = drive.linears.getCurrentPosition();
        drive.setPoseEstimate(ToFirstCone);

        Trajectory t1 = drive.trajectoryBuilder(ToFirstCone)
                .splineToConstantHeading(new Vector2d(-35,40),Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(-35,10,Math.toRadians(-50)),Math.toRadians(-90))
                .build();

        Trajectory t2 = drive.trajectoryBuilder(t1.end())
                .splineToSplineHeading(new Pose2d(-60,12,Math.toRadians(180)),Math.toRadians(180))
                .build();

        Trajectory t3 = drive.trajectoryBuilder(t2.end())
                .splineToSplineHeading(new Pose2d(-35,10,Math.toRadians(180)),Math.toRadians(-50))
                .build();

        Trajectory right = drive.trajectoryBuilder(new Pose2d(t1.end().vec(),Math.toRadians(-50)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-15,13,Math.toRadians(180)),Math.toRadians(180))
                .build();

        Trajectory left = drive.trajectoryBuilder(new Pose2d(t1.end().vec(),Math.toRadians(-50)),Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-62,13,Math.toRadians(180)),Math.toRadians(180))
                .build();

        waitForStart();
        if (isStopRequested())return;

        drive.followTrajectory(t1);
        //drive.followTrajectory(t2);
       // drive.followTrajectory(t3);

        /* Actually do something useful */
      /*  if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            drive.followTrajectory(left);
            telemetry.addData("tag null or left", tagOfInterest.id);
        }else if(tagOfInterest.id == MIDDLE){

            telemetry.addData("tag middle",tagOfInterest.id);
        }else{
            drive.followTrajectory(right);
            telemetry.addData("tag of interest right",tagOfInterest.id);
        } */

        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        // while (opModeIsActive()) {sleep(20);}
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
    void forwardToPosition(double inches, double speed){
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        drive.leftRear.setTargetPosition(drive.leftRear.getCurrentPosition() + move);
        drive.leftFront.setTargetPosition(drive.leftFront.getCurrentPosition() + move);
        drive.rightRear.setTargetPosition(drive.rightRear.getCurrentPosition() + move);
        drive.rightFront.setTargetPosition(drive.rightFront.getCurrentPosition() + move);
        //
        drive.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        drive.leftFront.setPower(speed);
        drive.leftRear.setPower(speed);
        drive.rightFront.setPower(speed);
        drive.rightRear.setPower(speed);
        //
        while (drive.leftFront.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && drive.rightRear.isBusy()) {
        }
        drive.rightFront.setPower(0);
        drive.leftFront.setPower(0);
        drive.rightRear.setPower(0);
        drive.leftRear.setPower(0);
        return;
    }

//    void strafeToPosition(double inches, double speed){
//        int move = (int) (Math.round(inches * cpi * meccyBias));
//        //
//        drive.leftRear.setTargetPosition(drive.leftRear.getCurrentPosition() - move);
//        drive.leftFront.setTargetPosition(drive.leftFront.getCurrentPosition() + move);
//        drive.rightRear.setTargetPosition(drive.rightRear.getCurrentPosition() + move);
//        drive.rightFront.setTargetPosition(drive.rightFront.getCurrentPosition() - move);
//        //
//        drive.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        //
//        drive.leftFront.setPower(speed);
//        drive.leftRear.setPower(speed);
//        drive.rightFront.setPower(speed);
//        drive.rightRear.setPower(speed);
//        //
//        while (drive.leftFront.isBusy() && drive.rightFront.isBusy() && drive.leftRear.isBusy() && drive.rightRear.isBusy()) {
//        }
//        drive.rightFront.setPower(0);
//        drive.leftFront.setPower(0);
//        drive.rightRear.setPower(0);
//        drive.leftRear.setPower(0);
//        return;
//    }
}