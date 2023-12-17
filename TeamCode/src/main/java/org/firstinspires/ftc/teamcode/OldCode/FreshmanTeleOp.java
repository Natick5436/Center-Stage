package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.Mark13;

@TeleOp(name="TestTurn", group="TeleOp")
public class FreshmanTeleOp extends LinearOpMode {
    Mark13 robot;

    public void runOpMode() {
        robot = new Mark13(this, /*Mark_9.getSavedX()*/0, /*Mark_9.getSavedY()*/0, Math.PI / 2);

        double drivePower = 0.2;
        boolean bumperDown = false;
        boolean fastMode = false;
        boolean rightBumper = false;
        boolean backDown = false;

        waitForStart();


        while (opModeIsActive()) {


            if(gamepad1.a){
                robot.turn(1);
            }else if(gamepad1.b){
                robot.turn(0.02);
            }else if(gamepad1.x){
                robot.turn(0.1);
            }else{
                robot.turn(0);
            }


            //telemetry.addData("Raw IMU", ((REV_IMU)robot.getAngleTracker()).imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS));


            //telemetry.addData("Horizontal", robot.odo.h.distanceTraveled());
            //telemetry.addData("Vertical", robot.odo.v.distanceTraveled());
            //telemetry.addData("Angle", robot.getAngle());
            //telemetry.addData("Position", "X:"+robot.getX()+" Y:"+robot.getY());
            telemetry.addData("rF", robot.rF.getPower());
            telemetry.addData("rB", robot.rB.getPower());

            telemetry.addData("rF Direction", robot.rF.getDirection());
            telemetry.addData("rB Direction", robot.rB.getDirection());

            telemetry.update();
        }

    }
}
