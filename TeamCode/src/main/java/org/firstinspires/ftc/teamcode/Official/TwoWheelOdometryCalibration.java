/**
 * Calibration program for Odometry - Written by Nolan Palmer (2018 - 2020)
 * Edited by: [add your name and the current year here]
 *  - Maxwell Harriss (2022)
 *  - Adam Pochobut (2022)
 * ===================================
 * This code spins the robot to get values that odometry needs to work
 * will in telemetry display values
 */

package org.firstinspires.ftc.teamcode.Official;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.Robots.Mark13;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.TwoWheelOdometry;

import java.io.File;

@Autonomous(name = "2 Wheel Calibration", group = "Autonomous")
public class TwoWheelOdometryCalibration extends LinearOpMode {

    Mark13 robot;

    File horizontalRadiusFile = AppUtil.getInstance().getSettingsFile("horizontalRadius.txt");
    File verticalRadiusFile = AppUtil.getInstance().getSettingsFile("verticalRadius.txt");

    public void runOpMode(){
        robot = new Mark13(this, 0, 0, 0);
        TwoWheelOdometry odo = new TwoWheelOdometry(this, new DeadWheel(robot.lB, 0.0508, 8192, 1), new DeadWheel(robot.lF, 0.0508, 8192, 1), robot.getAngleTracker(), 1, 1, 1, 1);

        waitForStart();
        double lastH = odo.h.distanceTraveled();
        double lastV = odo.v.distanceTraveled();
        robot.inverseKinematics(new double[]{0, 0, 1});
        robot.turn(0.5);
        sleep(2500);
        robot.stopDrive();

        double centerToHorizontal = (odo.h.distanceTraveled()-lastH)/odo.getAngle();
        double centerToVertical = (odo.v.distanceTraveled()-lastV)/odo.getAngle();

        ReadWriteFile.writeFile(horizontalRadiusFile, String.valueOf(centerToHorizontal));
        ReadWriteFile.writeFile(verticalRadiusFile, String.valueOf(centerToVertical));

        while(opModeIsActive()){
            telemetry.addData("centerToHorizontal", centerToHorizontal);
            telemetry.addData("centerToVertical", centerToVertical);
            telemetry.addData("Angle", odo.getAngle());
            telemetry.update();
        }
    }
}
