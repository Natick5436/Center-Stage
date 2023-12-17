package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robots.Claw14;
import org.firstinspires.ftc.teamcode.Robots.ScrimRobot;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BarcodeScanner;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Scrim Auto",group="Autonomous")
public class ScrimAuto extends LinearOpMode {
    Claw14 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Claw14(this);

        while (!isStarted()) {

            telemetry.update();
            if (isStopRequested()) {
                return;
            }

        }
        robot.rF.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.simpleForward(0.3);
        sleep(4000);
        robot.simpleForward(0);

        robot.stopDrive();

    }
}