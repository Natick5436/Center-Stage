package org.firstinspires.ftc.teamcode.Official;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Robots.Claw14;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ClawTeleOp",group="TeleOp")
public class ClawTeleOp extends LinearOpMode {
    Claw14 robot;
    PIDCoefficients pidc = new PIDCoefficients(0, 0, 0);
    PIDFController armController = new PIDFController(pidc);
    long armTime = System.currentTimeMillis();
    long clawTime = System.currentTimeMillis();

    boolean firstTime = true;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Claw14(this);
        double drivePower = 0.5;

        robot.disableBrakes();

        while(!opModeIsActive()){
            telemetry.addData("LowerArm", robot.lowerArm.getTargetPosition());
            telemetry.addData("upperArm", robot.upperArm.getTargetPosition());
            telemetry.addData("outtakeServo", robot.outtakeServo.getPosition());
            telemetry.addData("droneLauncher", robot.droneLauncher.getPosition());
            telemetry.update();
        }


        waitForStart();

        robot.lowerArm.setPower(1);
        robot.upperArm.setPower(1);


        while (opModeIsActive()) {

//            robot.intakeArm.setPower(-armController.update(robot.intakeArm.getCurrentPosition(), robot.intakeArm.getPower()));
            robot.intakeArm.setPower(0.6);

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
                robot.lowerArm.setTargetPosition(robot.lowerArm.getTargetPosition() + 2);
                robot.upperArm.setTargetPosition(robot.upperArm.getTargetPosition() + 3);
            } else if (gamepad2.dpad_down) {
                robot.lowerArm.setTargetPosition(robot.lowerArm.getTargetPosition() - 2);
                robot.upperArm.setTargetPosition(robot.upperArm.getTargetPosition() - 3);
            }


//            if(gamepad2.left_trigger > 0.1){
//                robot.intakeArm.setTargetPosition(robot.intakeArm.getTargetPosition() + 3);
//            } else if (gamepad2.right_trigger > 0.1) {
//                robot.intakeArm.setTargetPosition(robot.intakeArm.getTargetPosition() - 3);
//            }


            if(!firstTime){
                if(System.currentTimeMillis() - armTime > 700) {
                    if (gamepad2.b && robot.intakeArm.getTargetPosition() == 0) {
                        robot.intakeArm.setTargetPosition(-495);
                        armTime = System.currentTimeMillis();
                    } else if (gamepad2.b && robot.intakeArm.getTargetPosition() == -495) {
                        robot.intakeArm.setTargetPosition(0);
                        armTime = System.currentTimeMillis();
                    }
                }

            }else{
                firstTime = false;
            }


            if(gamepad2.dpad_left){
                robot.outtakeServo.setPosition(0.55);
            }if(gamepad2.dpad_right){
                robot.outtakeServo.setPosition(0.28);
            }

            if(gamepad2.right_bumper){
                robot.doorServo.setPosition(0.28);
            }else if(gamepad2.left_bumper){
                robot.doorServo.setPosition(0.35);
            }

            if(gamepad2.left_trigger > 0){
                robot.droneLauncher.setPosition(robot.droneLauncher.getPosition() + 0.01);
            }
            if(gamepad2.right_trigger >0){
                robot.droneLauncher.setPosition(robot.droneLauncher.getPosition() - 0.01);
            }

            if(gamepad2.x){
                robot.clawRight.setPosition(1.0);
                robot.clawLeft.setPosition(0);
            }

            if(gamepad2.y){
                robot.clawRight.setPosition(0.81);
                robot.clawLeft.setPosition(0.19);
//                if(System.currentTimeMillis() - clawTime > 500) {
//                    robot.clawRight.setPosition(1.0);
//                    robot.clawLeft.setPosition(0);
//                    clawTime = System.currentTimeMillis();
//                }
            }

//            if(gamepad1.dpad_up){
//                pidc.kP += 0.01;
//            }
//            if(gamepad1.dpad_down){
//                pidc.kP -= 0.01;
//            }
//
//            if(gamepad1.dpad_left){
//                pidc.kI -= 0.01;
//            }
//            if(gamepad1.dpad_right){
//                pidc.kI += 0.01;
//            }
//
//            if(gamepad1.left_bumper){
//                pidc.kD -= 0.01;
//            }
//            if(gamepad1.right_bumper){
//                pidc.kD += 0.01;
//            }


            telemetry.addData("LowerArm", robot.lowerArm.getTargetPosition());
            telemetry.addData("upperArm", robot.upperArm.getTargetPosition());
            telemetry.addData("intakeArm", robot.intakeArm.getTargetPosition());

            telemetry.addData("intake Power", robot.intakeArm.getPower());

            telemetry.addData("outtakeServo", robot.outtakeServo.getPosition());
            telemetry.addData("droneLauncher", robot.droneLauncher.getPosition());

            telemetry.addData("leftClaw", robot.clawLeft.getPosition());
            telemetry.addData("rightClaw", robot.clawRight.getPosition());
            telemetry.addData("DoorServo", robot.doorServo.getPosition());

            telemetry.addData("intakeArm", robot.intakeArm.getTargetPosition());

            telemetry.update();


        }

    }
}
