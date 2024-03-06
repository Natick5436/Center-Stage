package org.firstinspires.ftc.teamcode.Official;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.Mark15;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BlueElementScanner;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RRBlueCloseAuto",group="Autonomous")
public class RRBlueCloseAuto extends LinearOpMode {

    BlueElementScanner pipeline;
    OpenCvWebcam phoneCam;
    BlueElementScanner.ElementPosition elementPipeline;
    BlueElementScanner.ElementPosition finalAnalysis;

    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

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

        Trajectory toCenter = robot.trajectoryBuilder(new Pose2d())
                .forward(42)
                .build();
        robot.followTrajectory(toCenter);

        if(finalAnalysis == BlueElementScanner.ElementPosition.LEFT){
            robot.turn(Math.toRadians(-155));

            Trajectory back = robot.trajectoryBuilder(new Pose2d())
                    .back(10)
                    .build();
            robot.followTrajectory(back);

            robot.leftSlide.setTargetPosition(500);
            robot.rightSlide.setTargetPosition(500);
            sleep(1000);

            robot.leftDoorServo.setPosition(0.75);
            sleep(1000);
            robot.leftDoorServo.setPosition(0.40);

            sleep(1000);

            robot.leftSlide.setTargetPosition(10);
            robot.rightSlide.setTargetPosition(10);

        } else if (finalAnalysis == BlueElementScanner.ElementPosition.CENTER) {

        }else{

        }

    }
}
