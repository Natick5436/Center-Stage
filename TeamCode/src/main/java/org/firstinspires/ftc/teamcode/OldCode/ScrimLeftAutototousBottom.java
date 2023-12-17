package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.ScrimRobot;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BarcodeScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//@Autonomous(name="Blue Auto Storage",group="Autonomous")
public class ScrimLeftAutototousBottom extends LinearOpMode {
    ScrimRobot robot;

    OpenCvInternalCamera phoneCam;
    BarcodeScanner pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ScrimRobot(this, /*Mark_9.getSavedX()*/3.3528, /*Mark_9.getSavedY()*/2.1336, 0);

        robot.disableBrakes();
        String visionPlaceholder = "low";
        telemetry.addData("Status", "Ready");
        telemetry.update();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BarcodeScanner();
        phoneCam.setPipeline(pipeline);

        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }
            @Override
            public void onError(int errorCode)
            {
            }
        });

        while (!isStarted()) {
            //telemetry.addData("Analysis", pipeline.getAnalysis());
            //telemetry.addData("Position", pipeline.position);
            //telemetry.addData("x", robot.getX());
            //telemetry.addData("y", robot.getY());
            telemetry.addData("Position",pipeline.getAnalysis());
            telemetry.update();
            if (isStopRequested()) {
                return;
            }

        }

        waitForStart();

        BarcodeScanner temp = pipeline;
        robot.getCameraOutput(pipeline);

        robot.arm.setPower(0.5);

        robot.strafe(0.2);
        //1500
        sleep(1600);
        robot.forward(-0.1);
        sleep(200);

        robot.stopDrive();

        robot.carousel.setPower(0.2);
        sleep(5000);
        robot.carousel.setPower(0);

        robot.turn(0.23,0.1,0.1);
        //robot.turn(0.4,Math.PI / 2 + 0.05,0.1);
        sleep(350);

        robot.forward(0.35);
        //1475
        sleep(1500);

        //robot.stopDrive();
        robot.turn(0.4,Math.PI / 2 + 0.05,0.1);
        robot.stopDrive();

        robot.forward(-0.2);
        //1475
        sleep(1000);

        robot.forward(0.2);
        if (pipeline.getAnalysis() == BarcodeScanner.BarcodePosition.RIGHT) {
            sleep(3000);
        } else {
            sleep(2500);
        }

        robot.stopDrive();

        //0.45
        robot.intake.setPower(-0.4);
        sleep(750);
        if (pipeline.getAnalysis() != BarcodeScanner.BarcodePosition.LEFT) {
            robot.outtake.setPosition(robot.OUTTAKEOPEN);
            sleep(1050);
            robot.outtake.setPosition(robot.OUTTAKECLOSED);
        }else{
            robot.outtake.setPosition(robot.OUTTAKEMIDDLE);
            sleep(1050);
            robot.outtake.setPosition(robot.OUTTAKECLOSED);
        }
        robot.intake.setPower(0);

        robot.forward(-0.25);
        sleep(1800);

        robot.forward(0);
        robot.arm.setPower(0.2);
        robot.arm.setTargetPosition(200);
        sleep(500);

        robot.strafe(-0.4);
        //610
        sleep(650);

        robot.stopDrive();

    }
}