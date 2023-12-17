package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;//package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Robots.Mark13;
//import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.TwoWheelOdometry;
//
//public class OdometryNavigation extends LinearOpMode {
//    // Declare robot object
//    private Mark13 robot;
//
//    // Declare odometry object
//    private TwoWheelOdometry odo;
//
//    // Declare elapsed time object
//    private ElapsedTime timer = new ElapsedTime();
//
//    // Constants for odometry wheel radii and track width
//    private static final double WHEEL_RADIUS = 0.048;
//    private static final double TRACK_WIDTH = 0.333;
//
//    // Target positions for the robot
//    private static final double TARGET_X1 = 0.5;
//    private static final double TARGET_Y1 = 0.5;
//    private static final double TARGET_X2 = 1.0;
//    private static final double TARGET_Y2 = 1.0;
//
//    // Maximum allowed error for target position
//    private static final double POSITION_ERROR = 0.05;
//
//    // Maximum allowed error for target angle
//    private static final double ANGLE_ERROR = 5.0;
//
//    // Constants for PID control
//    private static final double KP = 1.0;
//    private static final double KI = 0.1;
//    private static final double KD = 0.1;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        // Initialize robot and odometry objects
//        robot = new Mark13(this, 0, 0, 0);
//        odo = new TwoWheelOdometry(robot, WHEEL_RADIUS, TRACK_WIDTH);
//        robot.odo = odo;
//
//        // Wait for start button to be pressed
//        waitForStart();
//
//        // Start odometry thread
//        odo.start();
//
//        // Drive to first target position
//        driveToPosition(TARGET_X1, TARGET_Y1, 0);
//
//        // Wait for 1 second
//        timer.reset();
//        while (timer.milliseconds() < 1000) {
//            idle();
//        }
//
//        // Drive to second target position
//        driveToPosition(TARGET_X2, TARGET_Y2, 90);
//
//        // Stop odometry thread
//        odo.stop();
//    }
//
//    // Method to drive the robot to a target position using odometry information
//    // Method to drive the robot to a target position using odometry information
//    private void driveToPosition(double targetX, double targetY, double targetAngle) {
//        // Initialize PID control variables
//        double lastErrorX = 0;
//        double lastErrorY = 0;
//        double lastErrorA = 0;
//        double integralX = 0;
//        double integralY = 0;
//        double integralA = 0;
//
//        // Loop until target position is reached
//        while (opModeIsActive()) {
//            // Calculate error between current and target position
//            double errorX = targetX - odo.getX();
//            double errorY = targetY - odo.getY();
//            double errorA = AngleUnit.normalizeDegrees(targetAngle - odo.getAngle());
//
//            // Update integral term of PID control
//            integralX += errorX * timer.milliseconds();
//            integralY += errorY * timer.milliseconds();
//            integralA += errorA * timer.milliseconds();
//
//            // Calculate derivative term of PID control
//            double derivativeX = (errorX - lastErrorX) / timer.milliseconds();
//            double derivativeY = (errorY - lastErrorY) / timer.milliseconds();
//            double derivativeA = (errorA - lastErrorA) / timer.milliseconds();
//
//            // Calculate power for each drive motor
//            double powerX = KP * errorX + KI * integralX + KD * derivativeX;
//            double powerY = KP * errorY + KI * integralY + KD * derivativeY;
//            double powerA = KP * errorA + KI * integralA + KD * derivativeA;
//
//            // Set power for each drive motor
//            robot.lF.setPower(powerX + powerY + powerA);
//            robot.rF.setPower(-powerX + powerY - powerA);
//            robot.lB.setPower(-powerX + powerY + powerA);
//            robot.rB.setPower(powerX + powerY - powerA);
//
//            // Update last error for each axis
//            lastErrorX = errorX;
//            lastErrorY = errorY;
//            lastErrorA = errorA;
//
//            // Reset timer
//            timer.reset();
//
//            // Check if target position has been reached
//            if (Math.abs(errorX) < POSITION_ERROR && Math.abs(errorY) < POSITION_ERROR &&
//                    Math.abs(errorA) < ANGLE_ERROR) {
//                // Stop drive motors
//                robot.stopDrive();
//                break;
//            }
//        }
//    }
//}
//
//
//}