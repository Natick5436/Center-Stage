package org.firstinspires.ftc.teamcode.Official;

import android.icu.util.ValueIterator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.Mark15;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BlueElementScanner;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.RedElementScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RedFarAuto",group="Autonomous")
public class RedFarAuto extends LinearOpMode {
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
//        robot.autoBackward(0.3, 2000);
//        robot.autoStrafe(0.3,2000);

        robot.leftSlide.setPower(0.9);
        robot.rightSlide.setPower(0.9);

        robot.pushDown.setPosition(.5);
        sleep(2000);

        for(int i=0; i<400000;i++) {
            elementPipeline = pipeline.getAnalysis();
            telemetry.addData("Analysis", elementPipeline);
            telemetry.addData("I", i);
            telemetry.update();

            finalAnalysis = elementPipeline;
        }

        if(finalAnalysis == RedElementScanner.ElementPosition.LEFT){

            // Moves to position and sets up pixel placement
            robot.autoForward(0.5, 1000);
            robot.leftSlide.setTargetPosition(500);
            robot.rightSlide.setTargetPosition(500);
            robot.autoTurns(0.5, 700);

            // Places the pixel 1
            sleep(500);
            robot.leftDoorServo.setPosition(.4);
            sleep(1500);
            robot.leftDoorServo.setPosition(.75);

            robot.autoForward(0.3, 600);


        } else if (finalAnalysis == RedElementScanner.ElementPosition.CENTER) {
            // Untested

            // Moves to position and sets up pixel placement
            robot.autoForward(0.3, 1700);
            robot.leftSlide.setTargetPosition(400);
            robot.rightSlide.setTargetPosition(400);
            sleep(500);
            robot.autoTurns(0.4, 1800);

            // Places the pixel 1
            sleep(500);
            robot.leftDoorServo.setPosition(.4);
            sleep(1500);
            robot.leftDoorServo.setPosition(.75);

            robot.autoForward(0.3, 600);

        }else{
            robot.autoForward(0.3, 1700);
            robot.leftSlide.setTargetPosition(400);
            robot.rightSlide.setTargetPosition(400);
            sleep(500);
            robot.autoTurns(-0.4, 1100);
            robot.autoForward(-0.2, 300);


            // Places the pixel 1
            sleep(500);
            robot.leftDoorServo.setPosition(.4);
            sleep(1500);
            robot.leftDoorServo.setPosition(.75);
            robot.autoForward(0.2, 500);

        }


        robot.stopDrive();

    }
}