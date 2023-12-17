package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.ScrimRobot;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BarcodeScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

//@Autonomous(name="Red Auto Storage",group="Autonomous")
public class ScrimRightAutototousBottom extends LinearOpMode {
    ScrimRobot robot;
    String visionPlaceholder = "low";

    OpenCvInternalCamera phoneCam;
    BarcodeScanner pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ScrimRobot(this, /*Mark_9.getSavedX()*/1.1, /*Mark_9.getSavedY()*/1.8, Math.PI);
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
            //.addData("x", robot.getX());
            //telemetry.addData("y", robot.getY());
            //telemetry.addData("Obstacles", robot);
            telemetry.addData("Position",pipeline.getAnalysis());
            telemetry.update();
            if (isStopRequested()) {
                return;
            }
        }

        waitForStart();

        robot.getCameraOutput(pipeline);
        robot.arm.setPower(0.5);

        robot.forward(0.2);
        sleep(800);
        robot.turn(0.3,Math.PI/2,0.1);
        sleep(350);

        robot.forward(-0.3);
        sleep(1800);
        robot.forward(0);

        robot.strafe(0.15);
        sleep(820);
        robot.strafe(0);

        robot.stopDrive();

        robot.carousel.setPower(-0.2);
        sleep(5000);

        robot.carousel.setPower(0);

        robot.forward(0.2);
        sleep(400);

        robot.strafe(-0.3);
        sleep(1650);

        robot.forward(0.2);
        sleep(1950);
        robot.stopDrive();

        //0.45

        robot.intake.setPower(-0.4);
        sleep(750);
        robot.outtake.setPosition(robot.OUTTAKEOPEN);
        sleep(750);
        robot.outtake.setPosition(robot.OUTTAKECLOSED);
        robot.intake.setPower(0);

        robot.forward(-0.2);
        sleep(2500);

        robot.forward(0);
        robot.arm.setPower(0.2);
        robot.arm.setTargetPosition(200);
        sleep(500);

        robot.strafe(0.25);
        sleep(1000);

        robot.stopDrive();

    }
}
