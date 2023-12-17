package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Hardware.REV_IMU;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BarcodeScanner;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.TwoWheelOdometry;

import java.io.File;

public class ScrimRobot extends Mecanum_Drive {
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

    public DcMotor arm;
    String armInit = "arm";

    public DcMotor deadMotor;
    String motorInit = "deadMotor";

    public DcMotor carousel;
    String carouselInit = "carousel";

    public Servo outtake;
    String outtakeInit = "outtake";

    //public Servo vertOdo;
    //String vertOdoInit = "vertOdo";

    //public Servo horizOdo;
    //String horizOdoInit = "horizOdo";

    //public Servo pusher;
    //String pusherInit = "pusher"

    public DcMotor intake;
    String intakeInit = "intake";

    public static int armEncoderDiff;
    File armEncoderFile;

    public ScrimRobot(LinearOpMode ln, double initialX, double initialY, double initialAngle) {
        //super(ln.hardwareMap.dcMotor.get("lF"),ln.hardwareMap.dcMotor.get("lB"),ln.hardwareMap.dcMotor.get("rF"),ln.hardwareMap.dcMotor.get("rB"),DcMotor.RunMode.RUN_USING_ENCODER,)
        super(ln.hardwareMap.dcMotor.get("lF")/*lF*/, ln.hardwareMap.dcMotor.get("lB")/*lB*/, ln.hardwareMap.dcMotor.get("rF")/*rF*/, ln.hardwareMap.dcMotor.get("rB")/*rB*/, DcMotor.RunMode.RUN_USING_ENCODER);
        setMeasurements(driveWheelRadius, driveLengthX, driveLengthY, driveMotorMaxRPM);
        attachLinearOpMode(ln);

        carousel = ln.hardwareMap.dcMotor.get(carouselInit);
        arm = ln.hardwareMap.dcMotor.get(armInit);
        intake = ln.hardwareMap.dcMotor.get(intakeInit);
        deadMotor = ln.hardwareMap.dcMotor.get(motorInit);
        outtake = ln.hardwareMap.servo.get(outtakeInit);

        outtake.setDirection(Servo.Direction.REVERSE);
        outtake.setPosition(this.OUTTAKECLOSED);


        //vertOdo = ln.hardwareMap.servo.get(vertOdoInit);
        //horizOdo = ln.hardwareMap.servo.get(horizOdoInit);

        //vertOdo.setDirection(Servo.Direction.REVERSE);
        //horizOdo.setDirection(Servo.Direction.REVERSE);

        /*
        deadMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        deadMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */

        REV_IMU imu = new REV_IMU(ln, "imu", 1, initialAngle);
        File horizontalRadiusFile = AppUtil.getInstance().getSettingsFile("horizontalRadius.txt");
        File verticalRadiusFile = AppUtil.getInstance().getSettingsFile("verticalRadius.txt");
        armEncoderFile = AppUtil.getInstance().getSettingsFile("armEncoder.txt");
        double horizontalRadius = Double.parseDouble(ReadWriteFile.readFile(horizontalRadiusFile).trim());
        double verticalRadius = Double.parseDouble(ReadWriteFile.readFile(verticalRadiusFile).trim());
        armEncoderDiff = 0; //Integer.parseInt(ReadWriteFile.readFile(armEncoderFile).trim());
        //odo = new TwoWheelOdometry(ln, new DeadWheel(intake, 0.0508, 8192, -1), new DeadWheel(deadMotor, 0.0508, 8192, -1), imu, horizontalRadius, verticalRadius, initialX, initialY);
        //odo.start();
        attachAngleTracker(imu);
        //attachPositionTracker(odo);
        enableBrakes();
        carousel.setDirection(DcMotorSimple.Direction.FORWARD);
        //vertOdo.setDirection(Servo.Direction.REVERSE);
        //horizOdo.setDirection(Servo.Direction.REVERSE);

        arm.setPower(0);
        deadMotor.setPower(1);
        /**This right here might be violent so be careful when testing**/
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /****/
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        /*
        lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);
        lB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lB.setDirection(DcMotorSimple.Direction.FORWARD);
        rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rF.setDirection(DcMotorSimple.Direction.FORWARD);
        rB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rB.setDirection(DcMotorSimple.Direction.FORWARD);
            */


        obstacles = new boolean[80][120];
        for (int x = 0; x < 80; x++) {
            for (int y = 0; y < 120; y++) {
                obstacles[x][y] = false;
            }
        }
        /*
        for (int x =46; x < 70; x++) {
            for (int y = 56; y < 82; y++) {
                obstacles[x][y] = true;
            }
        }
         */
        attachObstacles(obstacles, 2.3876, 3.6068);
        //this.attachObstacles(obstacles, 3.6576, 3.6576);
        this.createBuffers(0.30546);
        this.setPathFollowingParameters(2, Math.PI / 2, 0.7, 0);
    }

    public int getArmEncoderDiff() {
        return armEncoderDiff;
    }

    public void setArmEncoderDiff(int change) {
        armEncoderDiff += change;
        //ReadWriteFile.writeFile(armEncoderFile,Integer.toString(armEncoderDiff));
    }



    public void autoOutput(int repetitions, double power){
        long wait;
        boolean alternate = true;
        for(int i = 0; i < repetitions; i++) {
            wait = System.currentTimeMillis();
            if(System.currentTimeMillis() - wait > 500){
                if(alternate){
                    intake.setPower(power);
                    alternate = !alternate;
                }else{
                    intake.setPower(0);
                    alternate = !alternate;
                }
            }
        }
    }

    public void getCameraOutput(BarcodeScanner pipeline){
        if (pipeline.getAnalysis() == BarcodeScanner.BarcodePosition.RIGHT) {
            arm.setTargetPosition(650 + getArmEncoderDiff());
        } else if (pipeline.getAnalysis() == BarcodeScanner.BarcodePosition.CENTER) {
            arm.setTargetPosition(450 + getArmEncoderDiff());
        } else {
            arm.setTargetPosition(240 + getArmEncoderDiff());
        }
    }

    //moves robot a certain encoder ticks in the x direction
    public void moveX(double x) {
        double initX = this.getX();
        double angle = this.getAngle();
        double finX = initX + x;

        int dirX = x < 0 ? -1 : 1;
        int pow = 1;

        boolean stopX = false;

        while (true) {
            if (this.getX() != finX && !stopX) {
                this.angleStrafe(pow * dirX, angle);
            } else {
                stopX = true;
                this.stopDrive();
            }
        }
    }

    //moves robot a certain encoder ticks in the y direction
    public void moveY(double y) {
        double initY = this.getY();
        double angle = this.getAngle();
        double finY = initY + y;

        int dirY = y < 0 ? -1 : 1;
        int pow = 1;

        boolean stopY = false;

        while (true) {
            if (this.getY() != finY && !stopY) {
                this.angleStrafe(pow * dirY, Math.PI / 2);
            } else {
                stopY = true;
                this.stopDrive();
            }
        }
    }

    public void moveTest(double x, double y) throws InterruptedException{
        this.maneuverToPositionFinalAngle(x, y, 0.5, 0);
    }


}
