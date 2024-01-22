package org.firstinspires.ftc.teamcode.Official;

import android.icu.util.ValueIterator;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.Mark15;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.RedElementScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RedCloseAuto",group="Autonomous")
public class RedCloseAuto extends LinearOpMode {
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



//        robot.autoTurns(0.3, 2000);
//        robot.autoForward(0.3, 2000);
//        robot.autoStrafe(0.3,2000);

        if(finalAnalysis == RedElementScanner.ElementPosition.LEFT){
            // Moves to position and sets up pixel placement
            robot.autoForward(0.5, 1000);
            robot.leftSlide.setTargetPosition(400);
            robot.rightSlide.setTargetPosition(400);
            robot.autoTurns(0.5, 700);


            // Places the pixel 1
            sleep(500);
            robot.leftDoorServo.setPosition(.4);
            sleep(1500);
            robot.leftDoorServo.setPosition(.75);

            // Moves to board
            robot.autoForward(.5, 80);
            robot.autoStrafe(-.7, 500);
            robot.autoForward(-.5, 800);
            robot.leftSlide.setTargetPosition(1400);
            robot.rightSlide.setTargetPosition(1400);
            robot.autoStrafe(.7, 300);
            robot.autoForward(-.3, 1000);

            // Places the pixel 2
            sleep(500);
            robot.rightDoorServo.setPosition(.75);
            robot.leftSlide.setTargetPosition(1500);
            robot.rightSlide.setTargetPosition(1500);
            sleep(1500);
            robot.autoForward(.3, 800);
            robot.rightDoorServo.setPosition(.4);

            // moves to park
            robot.leftSlide.setTargetPosition(0);
            robot.rightSlide.setTargetPosition(0);
            sleep(1000);
            robot.autoStrafe(-.7, 800);
            robot.autoForward(-.5, 1000);

        } else if (finalAnalysis == RedElementScanner.ElementPosition.CENTER) {

            // Moves to position and sets up pixel placement
            robot.autoForward(0.31, 15+00);
            robot.leftSlide.setTargetPosition(400);
            robot.rightSlide.setTargetPosition(400);
            sleep(500);
            robot.autoTurns(0.4, 2100);

            // Places the pixel 1
            sleep(500);
            robot.leftDoorServo.setPosition(.4);
            sleep(1500);
            robot.leftDoorServo.setPosition(.75);

            // Moves to board
            robot.autoForward(.3, 300);
            robot.autoTurns(0.4, 1100);
            robot.autoForward(-.3, 2800);
            robot.autoStrafe(-0.3,1000);
            robot.leftSlide.setTargetPosition(2000);
            robot.rightSlide.setTargetPosition(2000);
            robot.autoForward(-.2, 1100);
            sleep(1000);

            // Places the pixel 2
            sleep(500);
            robot.rightDoorServo.setPosition(.75);
            sleep(1500);
            robot.autoForward(.3, 800);
            robot.rightDoorServo.setPosition(.4);

            // moves to park
            robot.leftSlide.setTargetPosition(0);
            robot.rightSlide.setTargetPosition(0);
            sleep(1000);
            robot.autoStrafe(.7, 800);
            robot.autoForward(-.4, 850);

        }else{

            // Moves to position and sets up pixel placement
            robot.autoForward(0.5, 1000);
            robot.leftSlide.setTargetPosition(400);
            robot.rightSlide.setTargetPosition(400);
            robot.autoTurns(-0.5, 700);

            // Places the pixel 1
            sleep(500);
            robot.leftDoorServo.setPosition(.4);
            sleep(1500);
            robot.leftDoorServo.setPosition(.75);

            // Moves to board
            robot.autoForward(.5, 80);
            robot.autoTurns(-0.5, 1400);
            robot.autoForward(-.5, 800);
            robot.leftSlide.setTargetPosition(1400);
            robot.rightSlide.setTargetPosition(1400);
            robot.autoForward(-.3, 1000);

            // Places the pixel 2
            sleep(500);
            robot.rightDoorServo.setPosition(.75);
            robot.leftSlide.setTargetPosition(1500);
            robot.rightSlide.setTargetPosition(1500);
            sleep(1500);
            robot.autoForward(.3, 800);
            robot.rightDoorServo.setPosition(.4);

            // moves to park
            robot.leftSlide.setTargetPosition(0);
            robot.rightSlide.setTargetPosition(0);
            sleep(1000);
            robot.autoStrafe(-.7, 800);
            robot.autoForward(-.5, 1000);
        }

        robot.stopDrive();

    }
}