package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.Mark15;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.RedElementScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Disabled
public class RedFarAutoMove extends LinearOpMode {
    Mark15 robot;

    RedElementScanner pipeline;
    OpenCvWebcam phoneCam;
    RedElementScanner.ElementPosition elementPipeline;
    RedElementScanner.ElementPosition finalAnalysis;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark15(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new RedElementScanner();
        phoneCam.setPipeline(pipeline);


        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
                telemetry.addData("ERROR CODE:", errorCode);
                telemetry.update();
            }
        });

        while (opModeInInit())
        {
            elementPipeline = pipeline.getAnalysis();
            telemetry.addData("Analysis", elementPipeline);
            telemetry.update();

            finalAnalysis = elementPipeline;
        }


        waitForStart();



//        robot.autoTurns(0.3, 2000);
//        robot.autoForward(0.3, 2000);
//        robot.autoStrafe(0.3,2000);

        if(finalAnalysis == RedElementScanner.ElementPosition.LEFT){
            robot.autoTurns(0.3, 2000);
        } else if (finalAnalysis == RedElementScanner.ElementPosition.CENTER) {
            robot.autoForward(0.3, 2000);
        }else{
            robot.autoStrafe(0.3, 2000);
        }

        robot.stopDrive();

    }
}