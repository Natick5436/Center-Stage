package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.ScrimRobot;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BarcodeScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Disabled
public class AutoTest extends LinearOpMode {
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
            telemetry.addData("Obstacles", robot);
            telemetry.addData("Position",pipeline.getAnalysis());
            telemetry.update();
            robot.outtake.setPosition(robot.OUTTAKECLOSED);
            if (isStopRequested()) {
                return;
            }
        }

        waitForStart();

        BarcodeScanner.BarcodePosition placeholder = BarcodeScanner.BarcodePosition.LEFT;
        robot.arm.setPower(0.5);

        if (placeholder == BarcodeScanner.BarcodePosition.RIGHT) {
            robot.arm.setTargetPosition(650 + robot.getArmEncoderDiff());
        } else if (placeholder == BarcodeScanner.BarcodePosition.CENTER) {
            robot.arm.setTargetPosition(450 + robot.getArmEncoderDiff());
        } else {
            robot.arm.setTargetPosition(230 + robot.getArmEncoderDiff());
        }

        sleep(1000);

        robot.intake.setPower(-0.4);

        robot.forward(0.1);
        sleep(5000);
        robot.stopDrive();

        robot.intake.setPower(0);


        sleep(4000);

        robot.stopDrive();

    }
}
