package org.firstinspires.ftc.teamcode.Official;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Robots.Claw14;
import org.firstinspires.ftc.teamcode.Robots.Mark14;
import org.firstinspires.ftc.teamcode.Robots.Mark15;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp",group="TeleOp")
public class TeleOp extends LinearOpMode {
    Mark15 robot;
    PIDCoefficients pidc = new PIDCoefficients(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Mark15(this);
        double drivePower = 0.6;

        robot.disableBrakes();

        while(!opModeIsActive()){
            telemetry.addData("LowerArm", robot.leftSlide.getTargetPosition());
            telemetry.addData("upperArm", robot.rightSlide.getTargetPosition());
            telemetry.addData("outtakeServo", robot.outtakeServo.getPosition());
            telemetry.addData("droneLauncher", robot.droneLauncher.getPosition());
            telemetry.update();
        }


        waitForStart();

        robot.leftSlide.setPower(0.9);
        robot.rightSlide.setPower(0.9);


        while (opModeIsActive()) {

//            robot.intakeArm.setPower(-armController.update(robot.intakeArm.getCurrentPosition(), robot.intakeArm.getPower()));


            /**CONTROLLER 1**/
            /**GENERAL MOVEMENT**/

            if (gamepad1.right_trigger > 0 || gamepad1.left_trigger > 0) {
                //robot.SimpleStrafe(drivePower * (gamepad1.right_trigger - gamepad1.left_trigger));
                drivePower = 0.7;
                robot.rB.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.lB.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.rF.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);
                if (gamepad1.right_trigger > 0) {
                    robot.lF.setPower(-gamepad1.right_trigger * drivePower);
                    robot.rF.setPower(-gamepad1.right_trigger * drivePower);
                    robot.lB.setPower(-gamepad1.right_trigger * drivePower);
                    robot.rB.setPower(-gamepad1.right_trigger * drivePower);
                } else if (gamepad1.left_trigger > 0) {
                    robot.lF.setPower(gamepad1.left_trigger * drivePower);
                    robot.rF.setPower(gamepad1.left_trigger * drivePower);
                    robot.lB.setPower(gamepad1.left_trigger * drivePower);
                    robot.rB.setPower(gamepad1.left_trigger * drivePower);
                }
                robot.rB.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.lB.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.rF.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);
                drivePower = 0.5;
            } else {
                robot.rB.setDirection(DcMotorSimple.Direction.FORWARD);
                robot.lB.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.rF.setDirection(DcMotorSimple.Direction.REVERSE);
                robot.lF.setDirection(DcMotorSimple.Direction.FORWARD);

                robot.lF.setPower(-drivePower * gamepad1.left_stick_y);
                robot.lB.setPower(-drivePower * gamepad1.left_stick_y);
                robot.rF.setPower(-drivePower * gamepad1.right_stick_y);
                robot.rB.setPower(-drivePower * gamepad1.right_stick_y);
                robot.setStatus(Mecanum_Drive.Status.DRIVING);
            }

            /**CONTROLLER 2**/
            if(gamepad2.dpad_up){
                robot.leftSlide.setTargetPosition(2100);
                robot.rightSlide.setTargetPosition(2000);
            } else if (gamepad2.dpad_down) {
                robot.leftSlide.setTargetPosition(0);
                robot.rightSlide.setTargetPosition(0);

            }



            if(gamepad2.y){
                robot.intake.setPower(0.9);
            } else if (gamepad2.x) {
                robot.intake.setPower(-0.9);
            } else{
                robot.intake.setPower(0);
            }


            if(gamepad2.dpad_left){
                robot.outtakeServo.setPosition(0.88);
            }if(gamepad2.dpad_right){
                robot.outtakeServo.setPosition(0.52);
            }

            if(gamepad2.right_bumper){
                robot.doorServo.setPosition(0.85);
            }else if(gamepad2.left_bumper){
                robot.doorServo.setPosition(0.95);
            }

            if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0){
                robot.droneLauncher.setPosition(0.11);
            }


            if(gamepad1.b){
                robot.pushDown.setPosition(0);
            }




            telemetry.addData("leftSlide", robot.leftSlide.getTargetPosition());
            telemetry.addData("rightSlide", robot.rightSlide.getTargetPosition());

            telemetry.addData("outtakeServo", robot.outtakeServo.getPosition());
            telemetry.addData("droneLauncher", robot.droneLauncher.getPosition());

            telemetry.addData("DoorServo", robot.doorServo.getPosition());


            telemetry.update();


        }

    }
}
