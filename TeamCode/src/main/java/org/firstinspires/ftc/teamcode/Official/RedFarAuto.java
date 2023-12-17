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

        robot.rB.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lB.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.rF.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);

        robot.simpleStrafe(-0.3);
        sleep(4000);
        robot.simpleStrafe(0);

        robot.rB.setDirection(DcMotorSimple.Direction.FORWARD);
        robot.lB.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.rF.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);


        robot.stopDrive();

    }
}