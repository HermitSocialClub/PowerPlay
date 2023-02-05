
/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.hermitsocialclub.apriltags;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.util.Encoder;


import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AprilTagAutonomousInitDetectionExample extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    public DcMotor right_drive = null;
    public DcMotor left_drive = null;
    public DcMotor right_drive_2 = null;
    public DcMotor left_drive_2 = null;

    public Encoder leftEncoder;
    public Encoder frontEncoder;

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

    @Override
    public void runOpMode()
    {
        right_drive = hardwareMap.get(DcMotor.class,"right_drive");
        right_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        right_drive_2 = hardwareMap.get(DcMotor.class,"right_drive_2");
        right_drive_2.setDirection(DcMotorSimple.Direction.REVERSE);
        left_drive = hardwareMap.get(DcMotor.class,"left_drive");
        left_drive_2 = hardwareMap.get(DcMotor.class,"left_drive_2");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,"leftEncoder"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class,"frontEncoder"));


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

        /* Actually do something useful */
        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
            forwardToPosition(-9,0.25);
            strafeToPosition(-16,0.25);
            telemetry.addData("tag null or left", tagOfInterest.id);
        }else if(tagOfInterest.id == MIDDLE){
            strafeToPosition(-14,0.25);
            telemetry.addData("tag middle",tagOfInterest.id);
        }else{
            forwardToPosition(8,0.25);
            strafeToPosition(-14,0.25);
            telemetry.addData("tag of interest right",tagOfInterest.id);
        }

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
        left_drive_2.setTargetPosition(left_drive_2.getCurrentPosition() + move);
        left_drive.setTargetPosition(left_drive.getCurrentPosition() + move);
        right_drive_2.setTargetPosition(right_drive_2.getCurrentPosition() + move);
        right_drive.setTargetPosition(right_drive.getCurrentPosition() + move);
        //
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        left_drive.setPower(speed);
        left_drive_2.setPower(speed);
        right_drive.setPower(speed);
        right_drive_2.setPower(speed);
        //
        while (left_drive.isBusy() && right_drive.isBusy() && left_drive_2.isBusy() && right_drive_2.isBusy()) {
        }
        right_drive.setPower(0);
        left_drive.setPower(0);
        right_drive_2.setPower(0);
        left_drive_2.setPower(0);
        return;
    }

    void strafeToPosition(double inches, double speed){
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        left_drive_2.setTargetPosition(left_drive_2.getCurrentPosition() - move);
        left_drive.setTargetPosition(left_drive.getCurrentPosition() + move);
        right_drive_2.setTargetPosition(right_drive_2.getCurrentPosition() + move);
        right_drive.setTargetPosition(right_drive.getCurrentPosition() - move);
        //
        left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        right_drive_2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        left_drive.setPower(speed);
        left_drive_2.setPower(speed);
        right_drive.setPower(speed);
        right_drive_2.setPower(speed);
        //
        while (left_drive.isBusy() && right_drive.isBusy() && left_drive_2.isBusy() && right_drive_2.isBusy()) {
        }
        right_drive.setPower(0);
        left_drive.setPower(0);
        right_drive_2.setPower(0);
        left_drive_2.setPower(0);
        return;
    }
}