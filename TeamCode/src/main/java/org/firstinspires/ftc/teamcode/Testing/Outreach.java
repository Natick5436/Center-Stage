package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Outreach Bot", group = "TeleOp")
public class Outreach extends LinearOpMode {
    DcMotor lF;
    DcMotor lB;
    DcMotor rF;
    DcMotor rB;

    @Override
    public void runOpMode() throws InterruptedException {
        lF = hardwareMap.dcMotor.get("lF");
        lB = hardwareMap.dcMotor.get("lB");
        rF = hardwareMap.dcMotor.get("rF");
        rB = hardwareMap.dcMotor.get("rB");

        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        rB.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            lF.setPower(gamepad1.left_stick_y);
            lB.setPower(gamepad1.left_stick_y);
            rF.setPower(gamepad1.right_stick_y);
            rB.setPower(gamepad1.right_stick_y);

            if (gamepad1.left_trigger > 0) {
                lF.setPower(-gamepad1.left_trigger);
                rF.setPower(-gamepad1.left_trigger);
                lB.setPower(gamepad1.left_trigger);
                rB.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0) {
                lF.setPower(gamepad1.left_trigger);
                rF.setPower(gamepad1.left_trigger);
                lB.setPower(-gamepad1.left_trigger);
                rB.setPower(-gamepad1.left_trigger);
            } else {
                lF.setPower(0);
                rF.setPower(0);
                lB.setPower(0);
                rB.setPower(0);
            }
        }
    }
}
