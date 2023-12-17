package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//@TeleOp(name="Bot",group="TeleOp")
public class First extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fl = hardwareMap.dcMotor.get("fL");
        DcMotor fr = hardwareMap.dcMotor.get("fR");
        DcMotor bl = hardwareMap.dcMotor.get("bL");
        DcMotor br = hardwareMap.dcMotor.get("bR");

        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            fl.setPower(gamepad1.left_stick_y);
            bl.setPower(gamepad1.left_stick_y);
            fr.setPower(gamepad1.right_stick_y);
            br.setPower(gamepad1.right_stick_y);

            if (gamepad1.left_trigger > 0) {
                fl.setPower(-gamepad1.left_trigger);
                fr.setPower(-gamepad1.left_trigger);
                bl.setPower(gamepad1.left_trigger);
                br.setPower(gamepad1.left_trigger);
            } else if (gamepad1.right_trigger > 0) {
                fl.setPower(gamepad1.left_trigger);
                fr.setPower(gamepad1.left_trigger);
                bl.setPower(-gamepad1.left_trigger);
                br.setPower(-gamepad1.left_trigger);
            } else {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }


        }
    }
}
