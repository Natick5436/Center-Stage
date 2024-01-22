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

@Autonomous(name="BlueCloseAuto",group="Autonomous")
public class BlueCloseAuto extends LinearOpMode {
    Mark15 robot;

    BlueElementScanner pipeline;
    OpenCvWebcam phoneCam;
    BlueElementScanner.ElementPosition elementPipeline;
    BlueElementScanner.ElementPosition finalAnalysis;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark15(this);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BlueElementScanner();
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

        if(finalAnalysis == BlueElementScanner.ElementPosition.LEFT){

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

            robot.autoStrafe(.25, 2000);
            robot.autoForward(-.5, 700);
            robot.leftSlide.setTargetPosition(1400);
            robot.rightSlide.setTargetPosition(1400);
            robot.autoStrafe(-.7, 300);
            robot.autoForward(-.3, 800);

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
            robot.autoStrafe(-.5, 1400);
            robot.autoForward(-.3, 1000);





        } else if (finalAnalysis == BlueElementScanner.ElementPosition.CENTER) {
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

            // Moves to board
            robot.autoForward(.3, 300);
            robot.autoTurns(-0.3, 1500);
            robot.autoForward(-.3, 1200);
            robot.autoStrafe(-0.3,1000);
            robot.leftSlide.setTargetPosition(1600);
            robot.rightSlide.setTargetPosition(1600);
            robot.autoForward(-.2, 1600);
            sleep(1000);

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
            robot.autoStrafe(-.5, 900);
            robot.autoForward(-.3, 1200);

        }else{
            robot.autoForward(0.3, 1700);
            robot.leftSlide.setTargetPosition(400);
            robot.rightSlide.setTargetPosition(400);
            sleep(500);
            robot.autoTurns(-0.4, 900);

            // Places the pixel 1
            sleep(500);
            robot.leftDoorServo.setPosition(.4);
            sleep(1500);
            robot.leftDoorServo.setPosition(.75);

            //to board
            robot.autoTurns(-0.4, 1700);
            robot.autoForward(-0.3, 2200);
            sleep(1500);

            robot.leftSlide.setTargetPosition(1500);
            robot.rightSlide.setTargetPosition(1500);
            sleep(1500);
            robot.rightDoorServo.setPosition(.75);
            sleep(1000);
            robot.rightDoorServo.setPosition(.4);

            robot.leftSlide.setTargetPosition(0);
            robot.rightSlide.setTargetPosition(0);
            sleep(1000);
            robot.autoStrafe(-.5, 1000);
            robot.autoForward(-.3, 1200);


        }


        robot.stopDrive();

    }
}