package org.firstinspires.ftc.teamcode.OldCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Robots.ScrimRobot;

//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp",group="TeleOp")
public class TeleOp extends LinearOpMode {
    ScrimRobot robot;

    public void runOpMode() {
        robot = new ScrimRobot(this, /*Mark_9.getSavedX()*/0, /*Mark_9.getSavedY()*/0, Math.PI / 2);

        double drivePower = 0.2;
        boolean bumperDown = false;
        boolean fastMode = false;
        boolean rightBumper = false;
        boolean backDown = false;
        boolean bDown = false;
        boolean shooterOn = false;
        boolean aDown = false;
        boolean armOpen = false;

        boolean yDown = false;
        boolean outtakeOpen = false;

        boolean xDown = false;
        boolean outtakeMiddle = false;


        long armTime = System.currentTimeMillis();
        long armEncoderTime = System.currentTimeMillis();

        double carouselSpeed = 0;
        long carouselTime = System.currentTimeMillis();

        waitForStart();

        robot.arm.setPower(0.4);
        //50
        robot.arm.setTargetPosition(100 + robot.getArmEncoderDiff());
        robot.outtake.setPosition(robot.OUTTAKECLOSED);

        while (opModeIsActive()) {
            //Puts odo wheels up
//            robot.vertOdo.setPosition(-0.5);
//            robot.horizOdo.setPosition(-0.5);

            telemetry.addData("isBusy",robot.arm.isBusy());
            if (fastMode) {
                drivePower = 0.5;
            } else {
                drivePower = 0.3;
            }
            if (!gamepad1.right_bumper && rightBumper) fastMode = !fastMode;
            rightBumper = gamepad1.right_bumper;
            if (gamepad1.left_bumper) {
                if (Math.abs(gamepad1.right_stick_x) < 0.25) {
                    robot.angleStrafe(drivePower * Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x));
                } else {
                    robot.turningStrafe(drivePower * Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x), Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x), -gamepad1.right_stick_x);
                }
                bumperDown = true;
            } else if ((gamepad1.right_trigger - gamepad1.left_trigger) != 0) {
                robot.strafe(drivePower * (gamepad1.right_trigger - gamepad1.left_trigger));
            } else {
                if (!bumperDown) {
                        robot.lF.setPower(-drivePower * gamepad1.left_stick_y);
                        robot.lB.setPower(-drivePower * gamepad1.left_stick_y);
                        robot.rF.setPower(-drivePower * gamepad1.right_stick_y);
                        robot.rB.setPower(-drivePower * gamepad1.right_stick_y);
                        robot.setStatus(Mecanum_Drive.Status.DRIVING);
                } else {
                    if (Math.hypot(gamepad1.right_stick_y, gamepad1.right_stick_x) < 0.1 && Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x) < 0.1) {
                        bumperDown = false;
                    }
                }
            }
            if (gamepad1.back && !backDown) {
                // robot.getPositionTracker().resetPosition(0.2032, 3.40995);
                //robot.getAngleTracker().resetAngle(Math.PI/2);
                backDown = true;
            }
            backDown = gamepad1.back;

            if(gamepad2.left_trigger > 0){
                robot.carousel.setPower(0.2);
                /*
                robot.carousel.setPower(Range.clip(carouselSpeed,0,0.25));
                if (System.currentTimeMillis() - carouselTime > 100) {
                    carouselSpeed += 0.05;
                    carouselTime = System.currentTimeMillis();
                }
                 */
            }else if(gamepad2.right_trigger > 0){
                robot.carousel.setPower(-0.2);
                /*
                robot.carousel.setPower(Range.clip(carouselSpeed,-0.25,0));
                if (System.currentTimeMillis() - carouselTime > 100) {
                    carouselSpeed -= 0.05;
                    carouselTime = System.currentTimeMillis();
                }
                 */
            }else {
                robot.carousel.setPower(0);
                carouselSpeed = 0;
            }


            if (System.currentTimeMillis() - armTime > 700) {
                int armEncoderDiff = robot.getArmEncoderDiff();
                int armPosition = robot.arm.getTargetPosition();
                if(gamepad2.dpad_up && armPosition < 700 + armEncoderDiff){
                    robot.arm.setTargetPosition(armPosition + 200);
                    armTime = System.currentTimeMillis();
                    //sleep(500);
                }else if(gamepad2.dpad_down && armPosition > 1 + armEncoderDiff){
                    robot.arm.setTargetPosition(armPosition - 200);
                    armTime = System.currentTimeMillis();
                    //sleep(500);
                };
            }

            if (System.currentTimeMillis() - armEncoderTime > 250) {
                int armPosition = robot.arm.getTargetPosition();
                if (gamepad2.a) {
                    robot.setArmEncoderDiff(20);
                    robot.arm.setTargetPosition(armPosition + 20);
                    armEncoderTime = System.currentTimeMillis();
                } else if (gamepad2.b && !gamepad2.start) {
                    robot.setArmEncoderDiff(-20);
                    robot.arm.setTargetPosition(armPosition - 20);
                    armEncoderTime = System.currentTimeMillis();
                }
            }

            if (xDown && !gamepad2.x) {
                if (outtakeMiddle) {
                    robot.outtake.setPosition(robot.OUTTAKECLOSED);
                    outtakeOpen = false;
                } else {
                    robot.outtake.setPosition(robot.OUTTAKEMIDDLE);
                }
                outtakeMiddle = !outtakeMiddle;
            }

            if (yDown && !gamepad2.y) {
                if (outtakeOpen) {
                    robot.outtake.setPosition(robot.OUTTAKECLOSED);
                    outtakeMiddle = false;
                } else {
                    robot.outtake.setPosition(robot.OUTTAKEOPEN);
                }
                outtakeOpen = !outtakeOpen;
            }

            xDown = gamepad2.x;
            yDown = gamepad2.y;

            if(gamepad2.left_bumper){
                robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.intake.setPower(-0.5);
                //robot.carousel.getController().setServoPosition(robot.carousel.getPortNumber(),0);
            }else if(gamepad2.right_bumper){
                robot.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.intake.setPower(0.8);
            }else{
                robot.intake.setPower(0);
                //robot.carousel.getController().setServoPosition(robot.carousel.getPortNumber(),1);
            }




            //telemetry.addData("Raw IMU", ((REV_IMU)robot.getAngleTracker()).imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS));


            //telemetry.addData("Horizontal", robot.odo.h.distanceTraveled());
            //telemetry.addData("Vertical", robot.odo.v.distanceTraveled());
            //telemetry.addData("Angle", robot.getAngle());
            //telemetry.addData("Position", "X:"+robot.getX()+" Y:"+robot.getY());


            telemetry.addData("carouselSpeed", carouselSpeed);
            telemetry.addData("Arm position", robot.arm.getTargetPosition());
            telemetry.addData("down",gamepad1.dpad_down);
            telemetry.addData("up",gamepad1.dpad_up);
            telemetry.addData("fastMode",fastMode);
            telemetry.addData("Arm encoder diff", robot.getArmEncoderDiff());
            telemetry.addData("lb", robot.lB.getPower());
            telemetry.addData("rb", robot.rB.getPower());
            telemetry.addData("lF", robot.lF.getPower());
            telemetry.addData("rF", robot.rF.getPower());
            telemetry.addData("outtake Position", robot.outtake.getPosition());
            telemetry.addData("carousel", robot.carousel.getPower());
            //telemetry.addData("X",robot.getX());
            //telemetry.addData("Y",robot.getY());
            telemetry.addData("Angle",robot.getAngle());
            telemetry.update();
        }
        //Mark_9.savePosition(robot.getX(), robot.getY(), robot.getAngle());
    }
}
