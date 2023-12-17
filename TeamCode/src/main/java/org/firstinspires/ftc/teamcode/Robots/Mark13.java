package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Hardware.REV_IMU;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.TwoWheelOdometry;

import java.io.File;

public class Mark13 extends Mecanum_Drive{
    public static final double driveWheelRadius = 0.048;
    public static final double driveLengthX = 0.333;
    public static final double driveLengthY = 0.36;
    public static final double driveMotorMaxRPM = 369.75;

    //4
    public final double OUTTAKECLOSED = 0.40;
    public final double OUTTAKEMIDDLE = 0.30;
    public final double OUTTAKEOPEN = 0;

    public boolean[][] obstacles;
    public TwoWheelOdometry odo;

    public DcMotor leftAxis;
    String leftAxisInit = "leftAxis";

    public DcMotor rightAxis;
    String rightAxisInit = "rightAxis";

    public DcMotor leftPulley;
    String leftPulleyInit = "leftPulley";

    public DcMotor rightPulley;
    String rightPulleyInit = "rightPulley";



    public Servo clawSpinner;
    String clawSpinnerInit = "clawSpinner";

    public Servo grabber;
    String grabberInit = "grabber";

    public Servo centerServo;
    String centerServoInit = "centerServo";

    public final static double ARM_HOME = 0.0;
    public final static double ARM_MIN_RANGE = 0.0;
    public final static double ARM_MAX_RANGE = 1.0;


    public static int armEncoderDiff;
    public static int angleEncoderDiff;;

    public Mark13(LinearOpMode ln, double initialX, double initialY, double initialAngle) {
        super(ln.hardwareMap.dcMotor.get("lF")/*lF*/, ln.hardwareMap.dcMotor.get("lB")/*lB*/, ln.hardwareMap.dcMotor.get("rF")/*rF*/, ln.hardwareMap.dcMotor.get("rB")/*rB*/, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setMeasurements(driveWheelRadius, driveLengthX, driveLengthY, driveMotorMaxRPM);
        switchReverseSide();
        attachLinearOpMode(ln);

        REV_IMU imu = new REV_IMU(ln, "imu", 1, initialAngle);

        File horizontalRadiusFile = AppUtil.getInstance().getSettingsFile("horizontalRadius.txt");
        File verticalRadiusFile = AppUtil.getInstance().getSettingsFile("verticalRadius.txt");
        double horizontalRadius = Double.parseDouble(ReadWriteFile.readFile(horizontalRadiusFile).trim());
        double verticalRadius = Double.parseDouble(ReadWriteFile.readFile(verticalRadiusFile).trim());


        //armEncoderDiff = 0; //Integer.parseInt(ReadWriteFile.readFile(armEncoderFile).trim());
        odo = new TwoWheelOdometry(ln, new DeadWheel(lB, 0.0508, 8192, -1), new DeadWheel(lF, 0.0508, 8192, -1), imu, horizontalRadius, verticalRadius, initialX, initialY);
        odo.start();
        attachAngleTracker(imu);
        attachPositionTracker(odo);
        enableBrakes();

        obstacles = new boolean[80][120];

        attachObstacles(obstacles, 2.3876, 3.6068);
        createBuffers(0.30546);
        setPathFollowingParameters(2, Math.PI / 2,1, 0);



        leftAxis = ln.hardwareMap.dcMotor.get(leftAxisInit);
        rightAxis = ln.hardwareMap.dcMotor.get(rightAxisInit);
        leftPulley = ln.hardwareMap.dcMotor.get(leftPulleyInit);
        rightPulley = ln.hardwareMap.dcMotor.get(rightPulleyInit);

        clawSpinner = ln.hardwareMap.servo.get(clawSpinnerInit);

        centerServo = ln.hardwareMap.servo.get(centerServoInit);

        grabber =ln.hardwareMap.servo.get(grabberInit);
        grabber.setDirection(Servo.Direction.REVERSE);

        /***Sets all motors to run on encoders***/
        leftAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftAxis.setTargetPosition(0);
        leftAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightAxis.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightAxis.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightAxis.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightAxis.setTargetPosition(0);
        rightAxis.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightAxis.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftPulley.setTargetPosition(0);
        leftPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftPulley.setDirection(DcMotorSimple.Direction.REVERSE);

        leftPulley.setDirection(DcMotorSimple.Direction.REVERSE);

        rightPulley.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPulley.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightPulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightPulley.setTargetPosition(0);
        rightPulley.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        leftAxis.setPower(0.4);
        rightAxis.setPower(0.4);
        leftPulley.setPower(0.3);
        rightPulley.setPower(0.3);


        armEncoderDiff = 0;
        angleEncoderDiff = 0;


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
    public void armLift(int position, double power){
        rightAxis.setPower(power);
        leftAxis.setPower(power);
        rightAxis.setTargetPosition(position);
        leftAxis.setTargetPosition(position);
    }
   /* public void placeCone(){
        clawSpinner.setPosition(0.5);//position of drop
        centerServo.setPosition(0);
        grabber.setPosition(0);
        clawSpinner.setPosition(0.5);
   }*/
    public void axisUpToPole(){
        rightAxis.setTargetPosition(0);
        leftAxis.setTargetPosition(0);
    }
    public void axisDownToPosition(int position){
        rightAxis.setTargetPosition(400);
        leftAxis.setTargetPosition(400);
        while(rightAxis.getCurrentPosition()<200){
            //just waiting to hit 200
        }
        while(rightAxis.getCurrentPosition()<395){
            rightAxis.setPower(0.25);
            leftAxis.setPower(0.25);
        }
        rightAxis.setPower(0.4);
        leftAxis.setPower(0.4);

        while(leftAxis.getCurrentPosition()!=position){
            leftAxis.setTargetPosition(position);//top cone in stack
            rightAxis.setTargetPosition(position);//top cone in stack
        }

        while(grabber.getPosition()!=0.125){
            grabber.setPosition(0.125);
        }

    }

    public void simpleStrafe(double power){
        rB.setDirection(DcMotorSimple.Direction.FORWARD);
        lB.setDirection(DcMotorSimple.Direction.FORWARD);
        rF.setDirection(DcMotorSimple.Direction.FORWARD);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);
        if(power>0){
            lF.setPower(power);
            rF.setPower(power);
            lB.setPower(-power);
            rB.setPower(-power);
        }else if(power<0){
            lF.setPower(-power);
            rF.setPower(-power);
            lB.setPower(power);
            rB.setPower(power);
        }
        rB.setDirection(DcMotorSimple.Direction.REVERSE);
        lB.setDirection(DcMotorSimple.Direction.FORWARD);
        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        lF.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void simpleForward(double power){
        lF.setPower(power);
        lB.setPower(power);
        rF.setPower(power);
        rB.setPower(power);
        //setStatus(Mecanum_Drive.Status.DRIVING);
    }

}
