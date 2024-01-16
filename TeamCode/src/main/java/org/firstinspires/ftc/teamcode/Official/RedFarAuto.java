package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robots.Mark14;

@Disabled
public class RedFarAuto extends LinearOpMode {
    Mark14 robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark14(this);

        while (!isStarted()) {

            telemetry.update();
            if (isStopRequested()) {
                return;
            }

        }




        robot.stopDrive();

    }
}