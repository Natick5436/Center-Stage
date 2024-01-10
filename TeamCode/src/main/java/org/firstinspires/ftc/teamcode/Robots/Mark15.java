package org.firstinspires.ftc.teamcode.Robots;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;

public class Mark15 extends Mecanum_Drive {

    public DcMotor leftSlide;
    public String leftSlideInit = "leftSlide";

    public DcMotor rightSlide;
    public String rightSlideInit = "rightSlide";

    public DcMotor intake;
    public String intakeInit = "intake";

    public Servo droneLauncher;
    public String droneLauncherInit = "droneLauncher";

    public Servo pushDown;
    public String pushDownInit = "pushDown";

    public Servo leftDoorServo;
    public String leftDoorServoInit = "leftDoorServo";

    public Servo rightDoorServo;
    public String rightDoorServoInit = "rightDoorServo";

    public DcMotor winch;
    public String winchInit = "winch";

    public Mark15(LinearOpMode ln) {
        super(ln.hardwareMap.dcMotor.get("lF")/*lF*/, ln.hardwareMap.dcMotor.get("lB")/*lB*/, ln.hardwareMap.dcMotor.get("rF")/*rF*/, ln.hardwareMap.dcMotor.get("rB")/*rB*/, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide = ln.hardwareMap.dcMotor.get(leftSlideInit);
        rightSlide = ln.hardwareMap.dcMotor.get(rightSlideInit);
        winch = ln.hardwareMap.dcMotor.get(winchInit);
        intake = ln.hardwareMap.dcMotor.get(intakeInit);

        droneLauncher = ln.hardwareMap.servo.get(droneLauncherInit);
        leftDoorServo = ln.hardwareMap.servo.get(leftDoorServoInit);
        rightDoorServo = ln.hardwareMap.servo.get(rightDoorServoInit);
        pushDown = ln.hardwareMap.servo.get(pushDownInit);

        leftSlide.setPower(0);
        rightSlide.setPower(0);
        intake.setPower(0);
        winch.setPower(0);

        leftSlide.setDirection(DcMotorSimple.Direction.FORWARD);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

//        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lowerArm.setTargetPosition(0);
//
//        lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setTargetPosition(0);
        leftSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setTargetPosition(0);
        rightSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rB.setDirection(DcMotorSimple.Direction.FORWARD);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);
        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);




        switchReverseSide();

    }

    public void autoTurns(double power, int time) throws InterruptedException {
        lF.setPower(power);
        rF.setPower(-power);
        lB.setPower(-power);
        rB.setPower(power);

        sleep(time);

        lF.setPower(0);
        rF.setPower(0);
        lB.setPower(0);
        rB.setPower(0);
    }

    public void autoStrafe(double power, int time) throws InterruptedException {
        rB.setDirection(DcMotorSimple.Direction.FORWARD);
        lB.setDirection(DcMotorSimple.Direction.FORWARD);
        rF.setDirection(DcMotorSimple.Direction.FORWARD);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);


        lF.setPower(power);
        rF.setPower(power);
        lB.setPower(power);
        rB.setPower(power);

        sleep(time);

        lF.setPower(0);
        rF.setPower(0);
        lB.setPower(0);
        rB.setPower(0);

        rB.setDirection(DcMotorSimple.Direction.FORWARD);
        lB.setDirection(DcMotorSimple.Direction.REVERSE);
        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void autoForward(double power, int time) throws InterruptedException {
        lF.setPower(power);
        rF.setPower(power);
        lB.setPower(-power);
        rB.setPower(-power);

        sleep(time);

        lF.setPower(0);
        rF.setPower(0);
        lB.setPower(0);
        rB.setPower(0);
    }

}
