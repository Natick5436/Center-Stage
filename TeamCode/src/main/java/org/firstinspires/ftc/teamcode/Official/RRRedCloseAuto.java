package org.firstinspires.ftc.teamcode.Official;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BlueElementScanner;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.RedElementScanner;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="RRRedCloseAuto",group="Autonomous")
public class RRRedCloseAuto extends LinearOpMode {

    RedElementScanner pipeline;
    OpenCvWebcam phoneCam;
    RedElementScanner.ElementPosition elementPipeline;
    RedElementScanner.ElementPosition finalAnalysis;

    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);

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

        for(int i=0; i<300000;i++) {
            elementPipeline = pipeline.getAnalysis();
            telemetry.addData("Analysis", elementPipeline);
            telemetry.addData("I", i);
            telemetry.update();

            finalAnalysis = elementPipeline;
        }

        Trajectory toCenter = robot.trajectoryBuilder(new Pose2d())
                .forward(45)
                .build();
        robot.followTrajectory(toCenter);

        if(finalAnalysis == RedElementScanner.ElementPosition.LEFT){
            robot.turn(Math.toRadians(180));

            Trajectory back = robot.trajectoryBuilder(new Pose2d())
                    .forward(12)
                    .build();
            robot.followTrajectory(back);

            robot.leftSlide.setTargetPosition(600);
            robot.rightSlide.setTargetPosition(600);
            sleep(1000);

            robot.leftDoorServo.setPosition(0.75);
            sleep(1000);
            robot.leftDoorServo.setPosition(0.40);

            sleep(1000);

            Trajectory end = robot.trajectoryBuilder(new Pose2d())
                    .forward(1)
                    .build();
            robot.followTrajectory(back);

            robot.leftSlide.setTargetPosition(0);
            robot.rightSlide.setTargetPosition(0);

            robot.followTrajectory(end);
        } else if (finalAnalysis == RedElementScanner.ElementPosition.CENTER) {
            robot.turn(Math.toRadians(290));

            Trajectory back = robot.trajectoryBuilder(new Pose2d())
                    .forward(12)
                    .build();
            robot.followTrajectory(back);

            robot.leftSlide.setTargetPosition(600);
            robot.rightSlide.setTargetPosition(600);
            sleep(1000);
            //robot.setMotorPowers(-0.2, -0.2, -0.2, -0.2);

            robot.leftDoorServo.setPosition(0.75);
            sleep(1000);
            robot.leftDoorServo.setPosition(0.40);
            //robot.setMotorPowers(0, 0, 0, 0);

            sleep(1000);

            Trajectory end = robot.trajectoryBuilder(new Pose2d())
                    .forward(1)
                    .build();
            robot.followTrajectory(back);

            robot.leftSlide.setTargetPosition(0);
            robot.rightSlide.setTargetPosition(0);

            robot.followTrajectory(end);
        }else{
            robot.turn(Math.toRadians(-155));

            Trajectory back = robot.trajectoryBuilder(new Pose2d())
                    .back(5)
                    .build();
            robot.followTrajectory(back);

            robot.leftSlide.setTargetPosition(600);
            robot.rightSlide.setTargetPosition(600);
            sleep(1000);

            robot.leftDoorServo.setPosition(0.75);
            sleep(1000);
            robot.leftDoorServo.setPosition(0.40);

            sleep(1000);

            robot.leftSlide.setTargetPosition(0);
            robot.rightSlide.setTargetPosition(0);

            robot.followTrajectory(back);
        }

    }
}
