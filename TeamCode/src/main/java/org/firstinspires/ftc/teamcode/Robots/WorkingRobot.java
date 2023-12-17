package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.Hardware.Mecanum_Drive;
import org.firstinspires.ftc.teamcode.Hardware.REV_IMU;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.TwoWheelOdometry;

import java.io.File;

public class WorkingRobot extends Mecanum_Drive{

    public static final double driveWheelRadius = 0.05;
    public static final double driveLengthX = 0.3333750;
    public static final double driveLengthY = 0.3683;
    public static final double driveMotorMaxRPM = 184.875;

    public TwoWheelOdometry odo;

    public boolean[][] obstacles;

    public DcMotor arm;
    String armInit = "arm";

    public CRServo carousel;
    String carouselInit = "carousel";

    public Servo vertOdo;
    String vertOdoInit = "vertOdo";

    public Servo horizOdo;
    String horizOdoInit = "horizOdo";

    //public Servo pusher;
    //String pusherInit = "pusher"

    public DcMotor intake;
    String intakeInit = "intake";

    public WorkingRobot(LinearOpMode ln, double initialX, double initialY, double initialAngle){
        super(ln.hardwareMap.dcMotor.get("lF")/*lF*/, ln.hardwareMap.dcMotor.get("lB")/*lB*/, ln.hardwareMap.dcMotor.get("rF")/*rF*/, ln.hardwareMap.dcMotor.get("rB")/*rB*/, DcMotor.RunMode.RUN_USING_ENCODER);
        setMeasurements(driveWheelRadius, driveLengthX, driveLengthY, driveMotorMaxRPM);
        attachLinearOpMode(ln);
        VoltageSensor voltage = ln.hardwareMap.voltageSensor.get("Control Hub");
        attachVoltageSensor(voltage);

        carousel = ln.hardwareMap.crservo.get(carouselInit);
        arm = ln.hardwareMap.dcMotor.get(armInit);
        intake = ln.hardwareMap.dcMotor.get(intakeInit);

        vertOdo = ln.hardwareMap.servo.get(vertOdoInit);
        horizOdo = ln.hardwareMap.servo.get(horizOdoInit);

        vertOdo.setDirection(Servo.Direction.REVERSE);
        horizOdo.setDirection(Servo.Direction.REVERSE);

        REV_IMU imu = new REV_IMU(ln, "imu", 1, initialAngle);
        File horizontalRadiusFile = AppUtil.getInstance().getSettingsFile("horizontalRadius.txt");
        File verticalRadiusFile = AppUtil.getInstance().getSettingsFile("verticalRadius.txt");
        double horizontalRadius = Double.parseDouble(ReadWriteFile.readFile(horizontalRadiusFile).trim());
        double verticalRadius = Double.parseDouble(ReadWriteFile.readFile(verticalRadiusFile).trim());

        odo = new TwoWheelOdometry(ln, new DeadWheel(intake, 0.0508, 8192, -1), new DeadWheel(lF, 0.0508, 8192, -1), imu, horizontalRadius, verticalRadius, initialX, initialY);

        odo.start();
        attachAngleTracker(imu);
        attachPositionTracker(odo);
        enableBrakes();

        //Check
        arm.setPower(0);
        /**This right here might be violent so be careful when testing**/
        arm.setTargetPosition(50);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //vertOdo.setPosition(0.05);

        horizOdo.setPosition(-0.05);

        obstacles = new boolean[80][120];
        /*for (int x = 32; x < 75; x++) {
            for (int y = 37; y < 40; y++) {
                obstacles[x][y] = true;
            }
        }*/
        attachObstacles(obstacles, 2.3876, 3.6068);
        createBuffers(0.30546);
        setPathFollowingParameters(2, Math.PI / 2, 0.3, 0);
    }
}
