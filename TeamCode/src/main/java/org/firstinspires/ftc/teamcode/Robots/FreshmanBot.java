package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.TwoWheelOdometry;

import java.io.File;

public class FreshmanBot extends Mecanum_Drive{
    public static final double driveWheelRadius = 0.05;
    public static final double driveLengthX = 0.336;
    public static final double driveLengthY = 0.39625;
    public static final double driveMotorMaxRPM = 369.75;

    //4
    public final double OUTTAKECLOSED = 0.40;
    public final double OUTTAKEMIDDLE = 0.30;
    public final double OUTTAKEOPEN = 0;


    public boolean[][] obstacles;
    public TwoWheelOdometry odo;

    public DcMotor deadMotor;
    String motorInit = "deadMotor";


    public static int armEncoderDiff;
    File armEncoderFile;


    public FreshmanBot(LinearOpMode ln, double initialX, double initialY, double initialAngle) {
        super(ln.hardwareMap.dcMotor.get("lF")/*lF*/, ln.hardwareMap.dcMotor.get("lB")/*lB*/, ln.hardwareMap.dcMotor.get("rF")/*rF*/, ln.hardwareMap.dcMotor.get("rB")/*rB*/, DcMotor.RunMode.RUN_USING_ENCODER);
        setMeasurements(driveWheelRadius, driveLengthX, driveLengthY, driveMotorMaxRPM);
        attachLinearOpMode(ln);



    }

    public void SimpleForward(double power){
        lF.setPower(power);
        lB.setPower(power);
        rF.setPower(power);
        rB.setPower(power);
    }

    public void SimpleStrafe(double power){
        lF.setPower(Range.clip(power, -1, 1));
        lB.setPower(Range.clip(-power, -1, 1));
        rF.setPower(Range.clip(-power, -1, 1));
        rB.setPower(Range.clip(power, -1, 1));
    }

}
