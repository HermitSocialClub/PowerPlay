package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ChampAutoConfig {
    public static double startingX = 32.5;
    public static double startingY = 63;
    public static double startingHead = -90;

    public static double toFirstPoleSpline1X = 36;
    public static double toFirstPoleSpline1Y = 28;
    public static double toFirstPoleSpline1Tan = -90;
    public static double toFirstPoleSpline2X = 32;
    public static double toFirstPoleSpline2Y = 6.5;
    public static double toFirstPoleSpline2Tan = -90;
    public static double toFirstPoleSpline2Head = -135;

    public static double toStackFromHighPoleX = 60;
    public static double toStackFromHighPoleY = 12;
    public static double toStackFromHighPoleTan = -5;
    public static double toStackFromHighPoleHead = 0;

    public static double backToJunction1X = 43;
    public static double backToJunction1Y = 14;
    public static double backToJunction1Tan = 0;
    public static double backToJunction1Head = -90;
    public static double backToJunction2X = 32;
    public static double backToJunction2Y = 6.5;
    public static double backToJunction2Tan = -160;
    public static double backToJunction2Head = -135;

    public static int toFirstPoleLinearHeight = 2170;
    public static int toStackFromHighPoleLinearHeight = 360;
    public static int afterToStackFromHighPoleClawCloseLinearHeight = 600;

    public static long sleepAfterToStack = 1000;
    public static long sleepAfterClawCloses = 1500;
    public static long sleepAfterLinearRaisesAfterStack = 1000;
}
