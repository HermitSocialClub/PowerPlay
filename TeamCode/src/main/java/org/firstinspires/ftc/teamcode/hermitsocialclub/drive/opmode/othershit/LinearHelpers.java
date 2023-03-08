package org.firstinspires.ftc.teamcode.hermitsocialclub.drive.opmode.othershit;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hermitsocialclub.drive.bigWheelOdoMecanum;

public class LinearHelpers {
    public DcMotorEx linears;
//    private bigWheelOdoMecanum drive;
    public int startingPosition;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public LinearHelpers(bigWheelOdoMecanum drv, HardwareMap hwmap, Telemetry tmtry) {
        telemetry = tmtry;
//        drive = drv;
        hardwareMap = hwmap;
        linears = hardwareMap.get(DcMotorEx.class,"linear");
        linears.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        linears.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        linears.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        startingPosition = linears.getCurrentPosition();
    }
    public void setLinearHeight(int heightInTicks) {
        linears.setTargetPosition(startingPosition
                + heightInTicks);
        linears.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        linears.setPower(.95);
    }
    public void waitForLinears() {
        while (linears.isBusy()){
            telemetry.addData("CurrentPos", linears.getCurrentPosition());
            telemetry.addData("targetPos", linears.getTargetPosition());
            telemetry.update();
            continue;
        }
    }
//    public void waitForClaw() {
//        while (drive.cla)
////    }
//    public void closeClaw() {
//        drive.claw.setPosition(1);
//    }
//    public void openClaw() {
//        drive.claw.setPosition(0);
//    }



//    public void linearsBusy() {
//        return linears.isBusy()
//    }

}
