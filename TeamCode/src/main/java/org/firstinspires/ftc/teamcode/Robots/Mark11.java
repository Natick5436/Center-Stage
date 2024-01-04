package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.TwoWheelOdometry;

public class Mark11 extends Mecanum_Drive{
    public static final double driveWheelRadius = 0.05;
    public static final double driveLengthX = 0.336;
    public static final double driveLengthY = 0.39625;
    public static final double driveMotorMaxRPM = 3697.75;

    //4

    public DcMotor liftMotor;
    String liftMotorInit = "liftMotor";

    public DcMotor angleMotor;
    String angleMotorInit = "angleMotor";

    public boolean[][] obstacles;
    public TwoWheelOdometry odo;

    public CRServo centerServo;
    String centerServoInit = "centerServo";

    public CRServo leftGrabber;
    String leftGrabberInit = "leftGrabber";

    public CRServo rightGrabber;
    String rightGrabberInit = "rightGrabber";


    public final static double ARM_HOME = 0.0;
    public final static double ARM_MIN_RANGE = 0.0;
    public final static double ARM_MAX_RANGE = 1.0;


    public DcMotor deadMotor;
    String motorInit = "deadMotor";

    public static int armEncoderDiff;
    public static int angleEncoderDiff;;

    public Mark11(LinearOpMode ln, double initialX, double initialY, double initialAngle) {
        super(ln.hardwareMap.dcMotor.get("lF")/*lF*/, ln.hardwareMap.dcMotor.get("lB")/*lB*/, ln.hardwareMap.dcMotor.get("rF")/*rF*/, ln.hardwareMap.dcMotor.get("rB")/*rB*/, DcMotor.RunMode.RUN_USING_ENCODER);
        setMeasurements(driveWheelRadius, driveLengthX, driveLengthY, driveMotorMaxRPM);
        attachLinearOpMode(ln);

        liftMotor = ln.hardwareMap.dcMotor.get(liftMotorInit);
        angleMotor = ln.hardwareMap.dcMotor.get(angleMotorInit);
        //leftServo = ln.hardwareMap.servo.get(leftServoInit);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        angleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        angleMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        angleMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angleMotor.setTargetPosition(0);
        angleMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        centerServo =ln.hardwareMap.crservo.get(centerServoInit);
        leftGrabber =ln.hardwareMap.crservo.get(leftGrabberInit);
        rightGrabber = ln.hardwareMap.crservo.get(rightGrabberInit);


        leftGrabber.setDirection(CRServo.Direction.REVERSE);
        rightGrabber.setDirection(CRServo.Direction.FORWARD);

        lB.setDirection(DcMotorSimple.Direction.REVERSE);



        angleMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armEncoderDiff = 0;
        angleEncoderDiff = 0;


    }

    public void SimpleForward(double power){
        lF.setPower(power);
        lB.setPower(power);
        rF.setPower(power);
        rB.setPower(power);
    }

    public void SimpleStrafe(double power){
        lF.setPower(Range.clip(-power, -1, 1));
        lB.setPower(Range.clip(power, -1, 1));
        rF.setPower(Range.clip(-power, -1, 1));
        rB.setPower(Range.clip(power, -1, 1));
    }



    public int getArmEncoderDiff() {
        return armEncoderDiff;
    }

    public void setArmEncoderDiff(int change) {
        armEncoderDiff += change;
        //ReadWriteFile.writeFile(armEncoderFile,Integer.toString(armEncoderDiff));
    }

    public int getAngleEncoderDiff() {
        return angleEncoderDiff;
    }

    public void setAngleEncoderDiff(int change) {
        angleEncoderDiff += change;
        //ReadWriteFile.writeFile(armEncoderFile,Integer.toString(armEncoderDiff));
    }

}
