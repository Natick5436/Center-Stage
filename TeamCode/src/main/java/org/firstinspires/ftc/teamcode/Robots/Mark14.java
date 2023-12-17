package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;

public class Mark14 extends Mecanum_Drive {

    public DcMotor lowerArm;
    public String lowerArmInit = "lowerArm";

    public DcMotor upperArm;
    public String upperArmInit = "upperArm";

    public DcMotor intake;
    public String intakeInit = "intake";

    public Servo outtakeServo;
    public String outtakeServoInit = "outtakeServo";

    public Servo droneLauncher;
    public String droneLauncherInit = "droneLauncher";

    public Servo pushDown;
    public String pushDownInit = "pushDown";

    public Servo doorServo;
    public String doorServoInit = "DoorServo";

    public Mark14(LinearOpMode ln) {
        super(ln.hardwareMap.dcMotor.get("lF")/*lF*/, ln.hardwareMap.dcMotor.get("lB")/*lB*/, ln.hardwareMap.dcMotor.get("rF")/*rF*/, ln.hardwareMap.dcMotor.get("rB")/*rB*/, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lowerArm = ln.hardwareMap.dcMotor.get(lowerArmInit);
        upperArm = ln.hardwareMap.dcMotor.get(upperArmInit);
        intake = ln.hardwareMap.dcMotor.get(intakeInit);

        outtakeServo = ln.hardwareMap.servo.get(outtakeServoInit);
        droneLauncher = ln.hardwareMap.servo.get(droneLauncherInit);
        doorServo = ln.hardwareMap.servo.get(doorServoInit);
        pushDown = ln.hardwareMap.servo.get(pushDownInit);

        lowerArm.setPower(0);
        upperArm.setPower(0);
        intake.setPower(0);

        lowerArm.setDirection(DcMotorSimple.Direction.REVERSE);
        upperArm.setDirection(DcMotorSimple.Direction.REVERSE);

//        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        lowerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        lowerArm.setTargetPosition(0);
//
//        lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lowerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lowerArm.setTargetPosition(0);
        lowerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        upperArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upperArm.setTargetPosition(0);
        upperArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        switchReverseSide();

    }

    public void simpleStrafe(double power){
        lF.setPower(power);
        rF.setPower(power);
        lB.setPower(power);
        rB.setPower(power);
    }

}
