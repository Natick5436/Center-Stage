package org.firstinspires.ftc.teamcode.Official;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
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
            telemetry.update();
        }


        waitForStart();

        robot.leftSlide.setPower(0.9);
        robot.rightSlide.setPower(0.9);

        robot.winchSetter.setPosition(.5);



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

                robot.lF.setPower(drivePower * gamepad1.left_stick_y);
                robot.lB.setPower(drivePower * gamepad1.left_stick_y);
                robot.rF.setPower(drivePower * gamepad1.right_stick_y);
                robot.rB.setPower(drivePower * gamepad1.right_stick_y);
//                robot.setStatus(Mecanum_Drive.Status.DRIVING);
            }

            /**CONTROLLER 2**/
            if(gamepad2.dpad_up){
                robot.leftSlide.setTargetPosition(2100);
                robot.rightSlide.setTargetPosition(2000);
            } else if (gamepad2.dpad_down) {
                robot.leftSlide.setTargetPosition(0);
                robot.rightSlide.setTargetPosition(0);

            }
//
//            if(gamepad2.dpad_up){
//                robot.leftSlide.setTargetPosition(robot.leftSlide.getTargetPosition() + 1);
//                robot.rightSlide.setTargetPosition(robot.rightSlide.getTargetPosition() + 1);
//            } else if (gamepad2.dpad_down) {
//                robot.leftSlide.setTargetPosition(robot.leftSlide.getTargetPosition() - 1);
//                robot.rightSlide.setTargetPosition(robot.rightSlide.getTargetPosition() - 1);
//
//            }







                if(gamepad2.y){
                robot.intake.setPower(0.9);
            } else if (gamepad2.x) {
                robot.intake.setPower(-0.9);
            } else{
                robot.intake.setPower(0);
            }


           /* if(gamepad2.left_bumper){
                robot.leftDoorServo.setPosition(0);
            }
            if(gamepad2.right_bumper){
                robot.leftDoorServo.setPosition(robot.leftDoorServo.getPosition() +0.01);
            }
            */
            if(gamepad2.left_bumper){
                robot.leftDoorServo.setPosition(.4);
            } else
            {
                robot.leftDoorServo.setPosition(.75);
            }
            if(gamepad2.right_bumper){
                robot.rightDoorServo.setPosition(.75);
            } else
            {
                robot.rightDoorServo.setPosition(.4);
            }



            if(gamepad1.right_bumper){
                robot.winch.setPower(1);
            } else if (gamepad1.left_bumper) {
                robot.winch.setPower(-1);

            }else{
                robot.winch.setPower(0);
            }


            if(gamepad2.left_trigger > 0 && gamepad2.right_trigger > 0){
                robot.droneLauncher.setPower(-0.2);
                sleep(1000);
                robot.droneLauncher.setPower(0);
            }


            if(gamepad1.b){
                robot.pushDown.setPosition(-.2);
            }
            if(gamepad1.x){
                robot.winchSetter.setPosition(robot.winchSetter.getPosition() +0.005);
            }
            if (gamepad1.y)
            {
                robot.winchSetter.setPosition(robot.winchSetter.getPosition() -0.005);
            }



            telemetry.addData("leftSlide", robot.leftSlide.getTargetPosition());
            telemetry.addData("rightSlide", robot.rightSlide.getTargetPosition());

            telemetry.addData("LeftServo", robot.leftDoorServo.getPosition());
            telemetry.addData("RightServo", robot.rightDoorServo.getPosition());
            telemetry.addData("winchsetter", robot.winchSetter.getPosition());


            telemetry.update();


        }

    }
}
