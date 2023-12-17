package org.firstinspires.ftc.teamcode.Testing;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.ScrimRobot;

@Autonomous(name="Test Auto",group="Autonomous")
public class TestAuto extends LinearOpMode {
    ScrimRobot robot;

    String visionPlaceholder = "low";

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ScrimRobot(this, 0.1524, 0.9144, Math.PI/2);
        telemetry.addData("Status", "Ready");
        telemetry.update();

        while (!isStarted()) {
            telemetry.addData("x", robot.getX());
            telemetry.addData("y", robot.getY());
            telemetry.update();
            if (isStopRequested()) {
                return;
            }
        }
        waitForStart();

        if(visionPlaceholder == "low"){
            robot.arm.setTargetPosition(50);
        }else if(visionPlaceholder == "med"){
            robot.arm.setTargetPosition(185);
        }else{
            robot.arm.setTargetPosition(360);
        }

        robot.maneuverToPositionFinalAngle(1, 1, 0.3, Math.PI);//Placeholder Variable



    }
}
