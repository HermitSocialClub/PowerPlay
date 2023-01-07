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
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous (name = "Meet3Auto")
public class Meet3Auto extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

//    public DcMotor right_drive = null;
//    public DcMotor left_drive = null;
//    public DcMotor right_drive_2 = null;
//    public DcMotor left_drive_2 = null;
    public SampleMecanumDrive drive;
//    public DcMotor linear = null;
    public Servo claw = null;
    public org.firstinspires.ftc.teamcode.util.Encoder leftEncoder;
    public org.firstinspires.ftc.teamcode.util.Encoder frontEncoder;

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
    Pose2d toFirstCone = new Pose2d(-37,65,Math.toRadians(0));
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

        drive = new SampleMecanumDrive(hardwareMap);
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
        claw.setPosition(1);
        drive.linears.setTargetPosition(drive.linears.getCurrentPosition() - 20);
        drive.linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.linears.setPower(0.5);
        drive.linears.setTargetPosition(drive.linears.getCurrentPosition() + 20);
        Trajectory myTrajectory = drive.trajectoryBuilder(toFirstCone)
                .splineTo(new Vector2d(-21, 7), Math.toRadians(240))
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
                    claw.setPosition(0);
                    //drive.linears.setPower(0);
                })
//                .strafeRight(10)
//                .forward(15)
                .build();

        Trajectory t2 = drive.trajectoryBuilder(new Pose2d(-15, 12, Math.toRadians(180)))
                .forward(25.25)
//                .addDisplacementMarker(() -> {
//                    drive.linears.setPower(-0.55);
//                    drive.linears.setTargetPosition(drive.linears.getCurrentPosition()
//                            -3500);
//                    drive.linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                })
                .addDisplacementMarker(()->{
                    linearsToTime(0.7,0.73);
                })
                .forward(25)
                .addDisplacementMarker(()-> {
                    claw.setPosition(1);
                })
                .build();

        Trajectory t3 = drive.trajectoryBuilder(new Pose2d(-21, 7, Math.toRadians(230)))
                .splineToLinearHeading(new Pose2d(-15, 12, Math.toRadians(230)), Math.toRadians(230))
                .addDisplacementMarker(() -> {
                    drive.linears.setPower(0);
                })
                        .build();

        Trajectory t4 = drive.trajectoryBuilder(t2.end())
                .addDisplacementMarker(()->{
                    drive.linears.setTargetPosition(drive.linears.getCurrentPosition()
                            + 4000);
                    drive.linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.linears.setPower(.55);
                })
                .back(53.5)
//                .addDisplacementMarker(() -> {
//                    drive.linears.setTargetPosition(drive.linears.getCurrentPosition()
//                            +1500);
//                    drive.linears.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    drive.linears.setPower(.55);
//                })
                .build();

        Trajectory t5 = drive.trajectoryBuilder(t4.end())
                .splineToLinearHeading(new Pose2d(-21, 7, Math.toRadians(230)),Math.toRadians(230))
                .addDisplacementMarker(()-> {
                    drive.linears.setTargetPosition(drive.linears.getCurrentPosition()
                    -500);
                        })
                .addDisplacementMarker(()->{
                    drive.linears.setPower(0);
                })
                .build();

        Trajectory tmiddle = drive.trajectoryBuilder(t2.end()).strafeRight(24).build();

        Trajectory tfront = drive.trajectoryBuilder(t2.end()).back(30).build();

        Trajectory tback = drive.trajectoryBuilder(t3.end()).forward(17).build();

        Trajectory tbackstrafe = drive.trajectoryBuilder(tback.end()).strafeRight(15).build();

        Trajectory ToTwo = drive.trajectoryBuilder(new Pose2d(t3.end().getX(), t3.end().getY()), Math.toRadians(0)).back(30).build();
        Trajectory ToThree = drive.trajectoryBuilder(new Pose2d(t3.end().getX(), t3.end().getY()), Math.toRadians(0)).back(55).build();



        waitForStart();

        if(isStopRequested()) return;

        getRuntime();

        drive.followTrajectory(myTrajectory);
        drive.followTrajectory(t3);
      //  drive.turn(Math.toRadians(50));
        //drive.linears.setTargetPosition(50);
      //  linearsToTime(0.6,0.3);
        //drive.linears.setPower(0.5);
        drive.followTrajectory(t2);
        //linearsToTime(0.1,0.1);
       // drive.claw.setPosition(1);
        //wait(500);
    //    wait(500);
        drive.followTrajectory(t4);
       // drive.turn(Math.toRadians(-50));
        drive.followTrajectory(t5);
       // drive.claw.setPosition(0);



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

