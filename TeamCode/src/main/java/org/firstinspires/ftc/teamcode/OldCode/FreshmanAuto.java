package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.FreshmanBot;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BarcodeScanner;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
public class FreshmanAuto extends LinearOpMode {
    FreshmanBot robot;


    OpenCvInternalCamera phoneCam;
    BarcodeScanner pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new FreshmanBot(this, /*Mark_9.getSavedX()*/3.3528, /*Mark_9.getSavedY()*/2.1336, 0);

        robot.disableBrakes();
        telemetry.addData("Status", "Ready");
        telemetry.update();

        while (!isStarted()) {
            //Put Telemetry Here

            telemetry.update();
            if (isStopRequested()) {
                return;
            }

        }

        waitForStart();

        robot.SimpleForward(0.1);
        sleep(4000);


        robot.stopDrive();

    }
}