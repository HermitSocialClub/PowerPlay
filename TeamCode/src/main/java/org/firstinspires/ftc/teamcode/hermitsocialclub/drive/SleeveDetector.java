package org.firstinspires.ftc.teamcode.hermitsocialclub.drive;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

/*
 * Image detector for Custom Sleeve
 */

public class SleeveDetector extends OpenCvPipeline {

        enum ParkingLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    private int width; //width of sleeve image
    ParkingLocation location;

    /*
    * width of the image - verify on camera
     */

    public SleeveDetector(int width) {
        this.width=width;
    }

@Override
public Mat processFrame(Mat input) {
    Mat result = input;
    boolean finished = false;
    // "Mat" stands for matrix, which is basically the image that the detector will process
    // the input matrix is the image coming from the camera
    // the function will return a matrix to be drawn on your phone's screen

    // The detector detects regular stones. The camera fits two stones.
    // If it finds one regular stone then the other must be the skystone.
    // If both are regular stones, it returns NONE to tell the robot to keep looking

    // Make a working copy of the input matrix in HSV
    Mat mat = new Mat();
    Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);

    //if there is an error, we assume CENTER
    if (mat.empty()) {
        location = ParkingLocation.CENTER;
        return input;

    }
    // We create a HSV range for each sleeve color to determine parking location
// NOTE: In OpenCV's implementation,
// Hue values are half the real value

    Scalar lowHSV = new Scalar(320, 50, 90); // lower bound HSV for pink
    Scalar highHSV = new Scalar(340, 70, 110); // higher bound HSV for pink
    Mat thresh = new Mat();

    // We'll get a black and white image. The white regions represent the regular stones.
    // inRange(): thresh[i][j] = {255,255,255} if mat[i][i] is within the range

    Core.inRange(mat, lowHSV, highHSV, thresh);

    // Use Canny Edge Detection to find edges
    // you might have to tune the thresholds for hysteresis
    Mat edges = new Mat();
    Imgproc.Canny(thresh, edges, 100, 300);

    // https://docs.opencv.org/3.4/da/d0c/tutorial_bounding_rects_circles.html
    // Oftentimes the edges are disconnected. findContours connects these edges.
    // We then find the bounding rectangles of those contours

    List<MatOfPoint> contours = new ArrayList<>();
    Mat hierarchy = new Mat();
    Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

    MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
    Rect[] boundRect = new Rect[contours.size()];
    for (int i = 0; i < contours.size(); i++) {
        contoursPoly[i] = new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
        boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
    }

    // Iterate and check whether the bounding boxes
    // cover left and/or right side of the image
    double left_x = 0.25 * width;
    double right_x = 0.75 * width;
    boolean left = false; // true if regular stone found on the left side
    boolean right = false; // "" "" on the right side
    for (int i = 0; i != boundRect.length; i++) {
        if (boundRect[i].x < left_x)
            left = true;
        if (boundRect[i].x + boundRect[i].width > right_x)
            right = true;

        // draw red bounding rectangles on mat
        // the mat has been converted to HSV so we need to use HSV as well
        Imgproc.rectangle(mat, boundRect[i], new Scalar(0.5, 76.9, 89.8));

        if (!left) location = ParkingLocation.LEFT;
        else if (!right) location = ParkingLocation.RIGHT;
            // if neither are true, then it is the Purple Center color.
        else location = ParkingLocation.CENTER;

    }
    return mat;  // return the mat with rectangles drawn

    }
    public ParkingLocation getLocation () {
        return this.location;
    }
}
