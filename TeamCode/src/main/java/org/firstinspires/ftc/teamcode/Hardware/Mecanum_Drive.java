/**
 * Sets up the Mecanum wheels that we use for our drive train - Written by ??? (If someone know please put it here (my guess is Nolan or adam))
 * Edited by: [add your name and the current year here]
 *  - Maxwell Harriss (2022)
 *  - Adam Pochobut (2022)
 *
 */
package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Math.ACMath;
import org.firstinspires.ftc.teamcode.Math.AStarSearch;
import org.firstinspires.ftc.teamcode.Math.PathInterface;
import org.firstinspires.ftc.teamcode.Math.PurePursuitPath;
import org.firstinspires.ftc.teamcode.Math.Waypoint;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.AngleTracker;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.PositionTracker;

import java.util.ArrayList;

    // Adds Identifiers to the robot for use in other parts of the program, and makes a value for that status to be identified with
public class Mecanum_Drive extends Robot{
    public enum Status {STRAFING, FORWARD, ARCHING, DRIVING, TURNING, ANGLE_STRAFING, INITIALIZING, INITIALIZED, PREINIT}
    private Status robotStatus;
    public void setStatus(Status s){
        robotStatus = s;
    }
    public Status getStatus(){
        return robotStatus;
    }

    // Adds values for the listed properties
    private double wheelRadius;
    private double LENGTH_Y;
    private double LENGTH_X;
    private double motorMaxRPM;
    private double rpmToRadPerSec = 0.10472;
    private double desiredVoltage = 12.0;
    private double forwardVPerStrafeV = 1.071;
    private double[][] inverseKinematicsMatrix;
    public DcMotor lF, lB, rF, rB;
    private double startAngle;
    final double maxCorrectionAngle = Math.PI/2;

    // Sensors
    private DcMotor.RunMode runMode;
    private VoltageSensor voltage;
    private AngleTracker angle;
    private PositionTracker pos;

    // Sets all the motors for the drive train as Dc motors and assigns values for the wheels properties *
    public DcMotor[] wheelArray;
    public Mecanum_Drive(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, DcMotor.RunMode runMode, double wheelRadius, double LENGTH_X, double LENGTH_Y, double motorMaxRPM){
        super();
        this.lF = lF;
        this.lB = lB;
        this.rF = rF;
        this.rB = rB;
        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        rB.setDirection(DcMotorSimple.Direction.REVERSE);
        lF.setMode(runMode);
        lB.setMode(runMode);
        rF.setMode(runMode);
        rB.setMode(runMode);
        this.runMode = runMode;
        this.wheelRadius = wheelRadius;
        this.LENGTH_X = LENGTH_X;
        this.LENGTH_Y = LENGTH_Y;
        this.motorMaxRPM = motorMaxRPM;
        this.wheelArray = new DcMotor[]{this.lF, this.rF, this.lB, this.rB};
        this.inverseKinematicsMatrix = new double[][]{{1, -1, (-(LENGTH_X + LENGTH_Y)/2)},
                                                        {1, 1, ((LENGTH_X + LENGTH_Y)/2)},
                                                        {1, 1, (-(LENGTH_X + LENGTH_Y)/2)},
                                                        {1, -1, ((LENGTH_X + LENGTH_Y)/2)}};
    }
    // Adds the Sets the the motors to run mode*
    public Mecanum_Drive(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, DcMotor.RunMode runMode){
        super();
        this.lF = lF;
        this.lB = lB;
        this.rF = rF;
        this.rB = rB;
        rF.setDirection(DcMotorSimple.Direction.REVERSE);
        rB.setDirection(DcMotorSimple.Direction.REVERSE);
        lF.setMode(runMode);
        lB.setMode(runMode);
        rF.setMode(runMode);
        rB.setMode(runMode);
        this.runMode = runMode;
        this.wheelArray = new DcMotor[]{this.lF, this.rF, this.lB, this.rB};
    }
    // Adds the measurements for the motor *
    public void setMeasurements(double wheelRadius, double LENGTH_X, double LENGTH_Y, double motorMaxRPM){
        this.wheelRadius = wheelRadius;
        this.LENGTH_X = LENGTH_X;
        this.LENGTH_Y = LENGTH_Y;
        this.motorMaxRPM = motorMaxRPM;
        this.inverseKinematicsMatrix = new double[][]{{1, -1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, 1, ((LENGTH_X + LENGTH_Y)/2)},
                {1, 1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, -1, ((LENGTH_X + LENGTH_Y)/2)}};
    }

    // Sets the program to execute in a sequential order
    public void attachLinearOpMode(LinearOpMode linearOpMode){
        super.ln = linearOpMode;
    }
    // This program changes the motors to resist movement when they are not being told to move
    public boolean enableBrakes(){
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if(runMode != DcMotor.RunMode.RUN_USING_ENCODER){
            return false;
        }else{
            return true;
        }
    }
    // This program changes the motors to not resist movement when they are not being told to move
    public void disableBrakes(){
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }




    /****Attaching and using Sensors****/

    public boolean isDriveEncoders(){
        return runMode == DcMotor.RunMode.RUN_USING_ENCODER;
    }

    //Voltage sensor
    public void attachVoltageSensor(VoltageSensor voltage){
        this.voltage = voltage;
    }
    public boolean isVoltage(){
        return voltage != null;
    }
    public VoltageSensor getVoltageSensor(){
        return voltage;
    }
    public double getVoltage(){
        return voltage.getVoltage();
    }

    //Angle Tracking
    public void attachAngleTracker(AngleTracker angle){
        this.angle = angle;
    }
    public boolean isAngleTracker(){
        return angle != null;
    }
    public AngleTracker getAngleTracker(){
        return angle;
    }
    public double getAngle(){
        return angle.getAngle();
    }

    //Position Tracking
    public void attachPositionTracker(PositionTracker pos){
        this.pos = pos;
    }
    public boolean isPosTracker(){
        return pos != null;
    }
    public PositionTracker getPositionTracker(){
        return pos;
    }
    public double getX(){
        return pos.getX();
    }
    public double getY(){
        return pos.getY();
    }




    /*****Basic Movement Methods (No position tracking or path following)*****/

    //All in one movement function
    // Parameters
    // index 0: X Velocity (m/s) aka velocity forward and backward (from robot's POV). Positive=Forward, Negative=Backwards
    // index 1: Y Velocity (m/s) aka velocity left and right (from robot's POV). Positive=Left, Negative=Right
    // index 2: Angular Velocity (rad/s) aka velocity counterclockwise and clockwise (from robot's POV). Positive=counterclockwise, Negative=clockwise
    public double[] inverseKinematics(double[] velocities){
        double[] wheelSpeeds = new double[4];
        velocities[1] = forwardVPerStrafeV*velocities[1];
        for(int i=0; i<inverseKinematicsMatrix.length; i++){
            double sum = 0;
            for(int j=0; j<inverseKinematicsMatrix[i].length; j++){
                sum+=inverseKinematicsMatrix[i][j]*velocities[j];
            }
            wheelSpeeds[i]=sum/wheelRadius;
        }
        for(int m=0; m<wheelArray.length; m++){
            wheelArray[m].setPower(wheelSpeeds[m]/(rpmToRadPerSec*motorMaxRPM));
        }
        return wheelSpeeds;
    }
    public void forward(double power){
        if(this.getStatus() != Status.FORWARD){
            startAngle = getAngle();
            this.setStatus(Status.FORWARD);
        }
        double angleError;
        if(ACMath.compassAngleShorter(getAngle(), startAngle)) {
            angleError = ACMath.toCompassAngle(getAngle()) - ACMath.toCompassAngle(startAngle);
        }else{
            angleError = ACMath.toStandardAngle(getAngle()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = angleError/maxCorrectionAngle;
        lF.setPower(power);
        lB.setPower(power);
        rF.setPower(power);
        rB.setPower(power);
        /*
        lF.setPower(Range.clip(power, -(1-turnOffset), (1-turnOffset))+turnOffset);
        lB.setPower(Range.clip(power, -(1-turnOffset), (1-turnOffset))+turnOffset);
        rF.setPower(Range.clip(power, -(1-turnOffset), (1-turnOffset))+turnOffset);
        rB.setPower(Range.clip(power, -(1-turnOffset), (1-turnOffset))+turnOffset);
         */
    }
    public void turn(double power){
        lF.setPower(-power);
        lB.setPower(-power);
        rF.setPower(power);
        rB.setPower(power);
    }
    public void turn(double power, double finalAngle, double angleFactor){
        double currentTime = System.currentTimeMillis();
        double lastTime = currentTime;
        double angleCorrection;
        if(ACMath.compassAngleShorter(finalAngle, getAngle())){
            angleCorrection = ACMath.toCompassAngle(finalAngle) - ACMath.toCompassAngle(getAngle());
        }else{
            angleCorrection =  ACMath.toStandardAngle(finalAngle) - ACMath.toStandardAngle(getAngle());
        }
        double lastAngleCorrection = angleCorrection;
        double angleI = 0;
        double angleD = 0;
        while(Math.abs(angleCorrection)>angleFactor && ln.opModeIsActive()){
            currentTime = System.currentTimeMillis();
            if(ACMath.compassAngleShorter(finalAngle, getAngle())){
                angleCorrection = ACMath.toCompassAngle(finalAngle) - ACMath.toCompassAngle(getAngle());
            }else{
                angleCorrection =  ACMath.toStandardAngle(finalAngle) - ACMath.toStandardAngle(getAngle());
            }
            angleI += angleCorrection*(currentTime-lastTime)/1000;
            angleD = 1000*(angleCorrection-lastAngleCorrection)/(currentTime-lastTime);
            double anglePower = 1*angleCorrection + 0.2*angleI + 0*angleD;/*D=0*/
            inverseKinematics(new double[]{0, 0, power*anglePower});
            lastTime = currentTime;
            ln.telemetry.addData("Turn: Angle correction", angleCorrection);
            ln.telemetry.update();
        }
        enableBrakes();
        stopDrive();
    }
    public void angleStrafe(double power, double angle){
        if(this.getStatus() != Status.ANGLE_STRAFING){
            startAngle = getAngle();
            this.setStatus(Status.ANGLE_STRAFING);
        }
        double angleError;
        if(ACMath.compassAngleShorter(getAngle(), startAngle)) {
            angleError = ACMath.toCompassAngle(getAngle()) - ACMath.toCompassAngle(startAngle);
        }else{
            angleError = ACMath.toStandardAngle(getAngle()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = angleError/maxCorrectionAngle;
        lF.setPower(Range.clip(power*Math.sin(angle + Math.PI/4), -(1-turnOffset), (1-turnOffset))+turnOffset);
        lB.setPower(Range.clip(power*Math.sin(angle - Math.PI/4), -(1-turnOffset), (1-turnOffset))+turnOffset);
        rB.setPower(Range.clip(power*Math.sin(angle + Math.PI/4), -(1-turnOffset), (1-turnOffset))-turnOffset);
        rF.setPower(Range.clip(power*Math.sin(angle - Math.PI/4), -(1-turnOffset), (1-turnOffset))-turnOffset);
    }
    public void turningStrafe(double strafePower, double angle, double turnPower){
        this.setStatus(Status.TURNING);
        double strafeProportion = Math.abs(strafePower/(Math.abs(strafePower)+Math.abs(turnPower)));
        double turnProportion = Math.abs(turnPower/(strafePower+turnPower));
        lF.setPower(strafeProportion*strafePower*Math.sin(angle + Math.PI/4)-turnProportion*turnPower);
        lB.setPower(strafeProportion*strafePower*Math.sin(angle - Math.PI/4)-turnProportion*turnPower);
        rB.setPower(strafeProportion*strafePower*Math.sin(angle + Math.PI/4)+turnProportion*turnPower);
        rF.setPower(strafeProportion*strafePower*Math.sin(angle - Math.PI/4)+turnProportion*turnPower);
    }
    public void strafe(double power) {
        if(this.getStatus() != Status.STRAFING){
            startAngle = getAngle();
            this.setStatus(Status.STRAFING);
        }
        double error;
        if(ACMath.compassAngleShorter(getAngle(), startAngle)) {
            error = ACMath.toCompassAngle(getAngle()) - ACMath.toCompassAngle(startAngle);
        }else{
            error = ACMath.toStandardAngle(getAngle()) - ACMath.toStandardAngle(startAngle);
        }
        double turnOffset = error/maxCorrectionAngle;
        lF.setPower(Range.clip(power+turnOffset, -1, 1));
        lB.setPower(Range.clip(-power+turnOffset, -1, 1));
        rF.setPower(Range.clip(-power-turnOffset, -1, 1));
        rB.setPower(Range.clip(power-turnOffset, -1, 1));
    }

    //Stops all 4 motors. Usually put after done with a movement so motors don't keep running
    public void stopDrive(){
        for(DcMotor wheel : wheelArray){
            wheel.setPower(0);
        }
    }

    //sets power to all the motors at once. Used for any purpose you can dream up.
    // Indexes of parameter
    // 0: leftBack
    // 1: leftFront
    // 2: rightBack
    // 3: rightFront
    public void setPowerAll(double[] powers){
        for(int i=0; i<wheelArray.length; i++){
            wheelArray[i].setPower(powers[i]);
        }
    }




    /*****Obstacle Avoidance, Path Following All that Jazz*****/

    private boolean[][] ob;
    private double pixelsPerMeter;
    public double fieldWidthX;
    public double fieldWidthY;
    private double maxRobotRadius;

    // Makes a function to set up obstacles so the auto can avoid field elements
    public void attachObstacles(boolean[][] ob, double fieldWidthX, double fieldWidthY){
        double tempPixelsPerMeter = Math.min(ob.length/fieldWidthX, ob[0].length/fieldWidthY);
        boolean[][] newOb = new boolean[(int)(fieldWidthX*tempPixelsPerMeter)][(int)(fieldWidthY*tempPixelsPerMeter)];
        for(int i=0; i<newOb.length; i++){
            for (int j=0; j<newOb[i].length; j++){
                if(i<ob.length && j<ob[i].length) {
                    newOb[i][j] = ob[i][j];
                }else{
                    newOb[i][j] = false;
                }
            }
        }
        this.ob = newOb;
        this.pixelsPerMeter = tempPixelsPerMeter;
        this.fieldWidthX = fieldWidthX;
        this.fieldWidthY = fieldWidthY;
    }

    public int toPixelSpace(double meterValue){
        return (int)(pixelsPerMeter*meterValue);
    }

    public void createBuffers(double maxRobotRadius){
        this.maxRobotRadius = maxRobotRadius;
        ob = PathInterface.addSpaceLimits(ob);
        ob = PathInterface.createObstacleBuffer(ob, toPixelSpace(maxRobotRadius));
    }

    public double m;
    public double s;
    private double positionError;
    private double linearCorrectSpeed;
    public void setPathFollowingParameters(double maxAngleCorrectVelocity /*Max Angular velocity to correct angle*/,
            double saturationAngle /*Max displacement angle before angle correction is saturated*/,
            double positionError/*How much the odometry can be off by before it stops the path and recalculates*/,
                                           double linearCorrectSpeed/*speed to correct smaller position errors*/){
        m = maxAngleCorrectVelocity;
        s = saturationAngle;
        this.positionError =  positionError;
        this.linearCorrectSpeed = linearCorrectSpeed;
    }


    /*
    *  THE COOL FUNCTION
    * If you set up everything correctly then this function will stop at nothing to make sure that you get
    * to the target location, this may very well turn your robot into a fighting machine so watch out
    *
    * Anyway this bad boy uses RECURSION and it melted my mind to create this
     */
    public boolean maneuverToPosition(double goalX, double goalY, double velocity, double maintainedAngle){
        disableBrakes();
        if(!(toPixelSpace(pos.getX())==toPixelSpace(goalX) && toPixelSpace(pos.getY())==toPixelSpace(goalY))) {
            boolean[][] instanceObstacles = PathInterface.drawSquare(ob, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(maxRobotRadius), getAngle(), false);
            AStarSearch aStar = new AStarSearch(ln, instanceObstacles, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(goalX), toPixelSpace(goalY), AStarSearch.AlgorthmType.DIJKSTRA);

            ArrayList<AStarSearch.Node> path;
            if (aStar.getFinalPath() != null) {
                path = aStar.getFinalPath();
            } else {
                return false;
            }

            path = PathInterface.condense(path);
            path = PathInterface.findShortCuts(path, instanceObstacles);
            Waypoint[] purePursuitInput = PathInterface.nodesToWaypoints(path, (1 / pixelsPerMeter));
            purePursuitInput = PathInterface.mergePoints(purePursuitInput, 0.10);
            PurePursuitPath follow = new PurePursuitPath(purePursuitInput);
            follow.mapTimes(velocity, this);

            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < follow.getTotalPathTime() * 1000.0) {
                double timeSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
                double angleCorrection;
                if (ACMath.compassAngleShorter(maintainedAngle + getAngle(), follow.angleByTime(timeSeconds))) {
                    angleCorrection = ACMath.toCompassAngle(follow.angleByTime(timeSeconds)) - ACMath.toCompassAngle(maintainedAngle + getAngle());
                } else {
                    angleCorrection = ACMath.toStandardAngle(follow.angleByTime(timeSeconds)) - ACMath.toStandardAngle(maintainedAngle + getAngle());
                }
                double xCorrection = follow.xOfTime(timeSeconds) - getX();
                double yCorrection = follow.yOfTime(timeSeconds) - getY();
                double[] speeds = inverseKinematics(new double[]{follow.linearVelocityOfTime(timeSeconds) * Math.cos(follow.angleByTime(timeSeconds) - getAngle()) + linearCorrectSpeed * (xCorrection / positionError) * Math.cos(getAngle()), follow.linearVelocityOfTime(timeSeconds) * Math.sin(follow.angleByTime(timeSeconds) - getAngle()) + linearCorrectSpeed * (yCorrection / positionError) * Math.sin(getAngle()), (follow.anglePrimeByTime(timeSeconds) + m * angleCorrection / s)});
                if (ln.isStopRequested()) {
                    enableBrakes();
                    stopDrive();
                    return false;
                }
                if (Math.hypot(follow.xOfTime(timeSeconds) - getX(), follow.yOfTime(timeSeconds) - getY()) > 1 ){//positionError) {
                    stopDrive();
                    ln.telemetry.addData("Off course:", "Recalculating");
                    ln.telemetry.addData("xoftime:", follow.xOfTime(timeSeconds));
                    ln.telemetry.addData("yoftime:", follow.yOfTime(timeSeconds));
                    ln.telemetry.update();
                    return maneuverToPosition(goalX, goalY, velocity, maintainedAngle);
                }
                pos.update();
                angle.update();
            }
        }
        enableBrakes();
        stopDrive();
        return true;
    }
    //Overloaded to be angle independent. Passes a DoubleFunction to find which angle it should be traveling at.
    public boolean maneuverToPositionFinalAngle(double goalX, double goalY, double velocity, double finalAngle){
        disableBrakes();
        if(!(toPixelSpace(pos.getX())==toPixelSpace(goalX) && toPixelSpace(pos.getY())==toPixelSpace(goalY))) {
            boolean[][] instanceObstacles = PathInterface.drawSquare(ob, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(maxRobotRadius), getAngle(), false);
            AStarSearch aStar = new AStarSearch(ln, instanceObstacles, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(goalX), toPixelSpace(goalY), AStarSearch.AlgorthmType.DIJKSTRA);

            ArrayList<AStarSearch.Node> path;
            if (aStar.getFinalPath() != null) {
                path = aStar.getFinalPath();
            } else {
                return false;
            }

            path = PathInterface.condense(path);
            path = PathInterface.findShortCuts(path, instanceObstacles);
            Waypoint[] purePursuitInput = PathInterface.nodesToWaypoints(path, (1 / pixelsPerMeter));
            //purePursuitInput = PathInterface.mergePoints(purePursuitInput, 0.3);
            PurePursuitPath follow = new PurePursuitPath(purePursuitInput);
            follow.mapTimes(velocity, this);

            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < follow.getTotalPathTime() * 1000.0) {
                double timeSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
                double angleCorrection;
                if (ACMath.compassAngleShorter(finalAngle, getAngle())) {
                    angleCorrection = ACMath.toCompassAngle(finalAngle) - ACMath.toCompassAngle(getAngle());
                } else {
                    angleCorrection = ACMath.toStandardAngle(finalAngle) - ACMath.toStandardAngle(getAngle());
                }
                double[] speeds = inverseKinematics(new double[]{follow.linearVelocityOfTime(timeSeconds) * Math.cos(follow.angleByTime(timeSeconds) - getAngle()), follow.linearVelocityOfTime(timeSeconds) * Math.sin(follow.angleByTime(timeSeconds) - getAngle()), m * angleCorrection / s});
                if (ln.isStopRequested()) {
                    enableBrakes();
                    stopDrive();
                    return false;
                }
                if (Math.hypot(follow.xOfTime(timeSeconds) - getX(), follow.yOfTime(timeSeconds) - getY()) > positionError) {
                    stopDrive();
                    ln.telemetry.addData("Off course:", "Recalculating");
                    ln.telemetry.addData("X: ", getX());
                    ln.telemetry.addData("Y: ", getY());
                    ln.telemetry.addData("y of time:", follow.yOfTime(timeSeconds));
                    ln.telemetry.addData("X of time:", follow.xOfTime(timeSeconds));
                    ln.telemetry.update();
                    return maneuverToPosition(goalX, goalY, velocity, finalAngle);
                }
            }
        }
        enableBrakes();
        stopDrive();
        return true;
    }
    public void switchReverseSide(){
        if(lF.getDirection() == DcMotorSimple.Direction.REVERSE){
            lF.setDirection(DcMotorSimple.Direction.FORWARD);
            lB.setDirection(DcMotorSimple.Direction.FORWARD);
            rF.setDirection(DcMotorSimple.Direction.REVERSE);
            rB.setDirection(DcMotorSimple.Direction.REVERSE);
        }else{
            lF.setDirection(DcMotorSimple.Direction.REVERSE);
            lB.setDirection(DcMotorSimple.Direction.REVERSE);
            rF.setDirection(DcMotorSimple.Direction.FORWARD);
            rB.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    public boolean maneuverToPositionFinalAngleAccurate(double goalX, double goalY, double velocity, double finalAngle, double accuracyFactor, double angleFactor){
        disableBrakes();
        if(!(toPixelSpace(pos.getX())==toPixelSpace(goalX) && toPixelSpace(pos.getY())==toPixelSpace(goalY))) {
            boolean[][] instanceObstacles = PathInterface.drawSquare(ob, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(maxRobotRadius), getAngle(), false);
            AStarSearch aStar = new AStarSearch(ln, instanceObstacles, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(goalX), toPixelSpace(goalY), AStarSearch.AlgorthmType.DIJKSTRA);
            ln.telemetry.addData("path null?", aStar.getFinalPath() == null);
            ln.telemetry.update();
            ArrayList<AStarSearch.Node> path;
            if (aStar.getFinalPath() != null) {
                path = aStar.getFinalPath();
            } else {
                return false;
            }
            path = PathInterface.condense(path);
            path = PathInterface.findShortCuts(path, instanceObstacles);
            Waypoint[] purePursuitInput = PathInterface.nodesToWaypoints(path, (1 / pixelsPerMeter));
            //purePursuitInput = PathInterface.mergePoints(purePursuitInput, 0.3);
            PurePursuitPath follow = new PurePursuitPath(purePursuitInput);
            follow.mapTimes(velocity, this);

            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < follow.getTotalPathTime() * 1000.0) {
                double timeSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
                double angleCorrection;
                if (ACMath.compassAngleShorter(finalAngle, getAngle())) {
                    angleCorrection = ACMath.toCompassAngle(finalAngle) - ACMath.toCompassAngle(getAngle());
                } else {
                    angleCorrection = ACMath.toStandardAngle(finalAngle) - ACMath.toStandardAngle(getAngle());
                }
                double[] speeds = inverseKinematics(new double[]{follow.linearVelocityOfTime(timeSeconds) * Math.cos(follow.angleByTime(timeSeconds) - getAngle()), follow.linearVelocityOfTime(timeSeconds) * Math.sin(follow.angleByTime(timeSeconds) - getAngle()), m * (angleCorrection + 0.05) / s});
                if (ln.isStopRequested()) {
                    enableBrakes();
                    stopDrive();
                    return false;
                }
                if (Math.hypot(follow.xOfTime(timeSeconds) - getX(), follow.yOfTime(timeSeconds) - getY()) > positionError) {
                    stopDrive();
                    ln.telemetry.addData("Off course:", "Recalculating");
                    ln.telemetry.update();
                    return maneuverToPositionFinalAngleAccurate(goalX, goalY, velocity, finalAngle, accuracyFactor, angleFactor);
                }
            }
        }
        double angleI = 0;
        double posXI = 0;
        double posYI = 0;
        double currentTime = System.currentTimeMillis();
        double lastTime = currentTime;
        double currentDistanceX = goalX-getX();
        double lastDistanceX = currentDistanceX;
        double currentDistanceY = goalY-getY();
        double lastDistanceY = currentDistanceY;
        double angleCorrection;
        if(ACMath.compassAngleShorter(finalAngle, getAngle())){
            angleCorrection = ACMath.toCompassAngle(finalAngle) - ACMath.toCompassAngle(getAngle());
        }else{
            angleCorrection =  ACMath.toStandardAngle(finalAngle) - ACMath.toStandardAngle(getAngle());
        }
        double lastAngleCorrection = angleCorrection;
        double angleD = 0;
        double posDX = 0;
        double posDY = 0;
        boolean stopPos = false;
        enableBrakes();
        while(ln.opModeIsActive()&&(Math.hypot(currentDistanceX, currentDistanceY)>accuracyFactor || Math.abs(angleCorrection)>angleFactor)){
            currentDistanceX = goalX-getX();
            currentDistanceY = goalY-getY();
            lastAngleCorrection = angleCorrection;
            if(ACMath.compassAngleShorter(finalAngle, getAngle())){
                angleCorrection = ACMath.toCompassAngle(finalAngle) - ACMath.toCompassAngle(getAngle());
            }else{
                angleCorrection =  ACMath.toStandardAngle(finalAngle) - ACMath.toStandardAngle(getAngle());
            }
            lastTime = currentTime;
            currentTime = System.currentTimeMillis();
            posXI += currentDistanceX*(currentTime-lastTime)/1000;
            posYI += currentDistanceY*(currentTime-lastTime)/1000;
            angleI += angleCorrection*(currentTime-lastTime)/1000;

            angleD = 1000*(angleCorrection-lastAngleCorrection)/(currentTime-lastTime);
            posDX = 1000*(currentDistanceX-lastDistanceX)/(currentTime-lastTime);
            posDX = 1000*(currentDistanceY-lastDistanceY)/(currentTime-lastTime);

            if(Math.hypot(goalX-getX(), goalY-getY()) < accuracyFactor){
                stopPos = true;
                enableBrakes();
                stopDrive();
            }
            double anglePower = 2*angleCorrection + 0.05*angleI + 0*angleD;
            double positionPowerX;
            double positionPowerY;
            if(!stopPos){
                positionPowerX = 1.5*velocity*currentDistanceX + 0.1*velocity*posXI;
                positionPowerY = 1.5*velocity*currentDistanceY + 0.1*velocity*posYI;
            }else{
                positionPowerX = 0;
                positionPowerY = 0;
            }
            double positionPower = Math.hypot(positionPowerX, positionPowerY);
            inverseKinematics(new double[]{positionPower*Math.cos(Math.atan2(goalY-getY(), goalX-getX())-getAngle()), positionPower*Math.sin(Math.atan2(goalY-getY(), goalX-getX())-getAngle()), velocity*anglePower});
            ln.telemetry.addData("Final correction", "final");
            ln.telemetry.addData("Movement: Angle correction", angleCorrection);
            ln.telemetry.update();
            ln.telemetry.update();
        }
        enableBrakes();
        stopDrive();
        return true;
    }
    //Overloaded to be angle independent. Passes a DoubleFunction to find which angle it should be traveling at.
    public boolean maneuverToPositionSpinny(double goalX, double goalY, double velocity, double spinSpeed){
        disableBrakes();
        if(!(toPixelSpace(pos.getX())==toPixelSpace(goalX) && toPixelSpace(pos.getY())==toPixelSpace(goalY))) {
            boolean[][] instanceObstacles = PathInterface.drawSquare(ob, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(maxRobotRadius), getAngle(), false);
            AStarSearch aStar = new AStarSearch(ln, instanceObstacles, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(goalX), toPixelSpace(goalY), AStarSearch.AlgorthmType.DIJKSTRA);

            ArrayList<AStarSearch.Node> path;
            if (aStar.getFinalPath() != null) {
                path = aStar.getFinalPath();
            } else {
                return false;
            }

            path = PathInterface.condense(path);
            path = PathInterface.findShortCuts(path, instanceObstacles);
            Waypoint[] purePursuitInput = PathInterface.nodesToWaypoints(path, (1 / pixelsPerMeter));
            //purePursuitInput = PathInterface.mergePoints(purePursuitInput, 0.3);
            PurePursuitPath follow = new PurePursuitPath(purePursuitInput);
            follow.mapTimes(velocity, this);

            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < follow.getTotalPathTime() * 1000.0) {
                double timeSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
                double[] speeds = inverseKinematics(new double[]{follow.linearVelocityOfTime(timeSeconds) * Math.cos(follow.angleByTime(timeSeconds) - getAngle()), follow.linearVelocityOfTime(timeSeconds) * Math.sin(follow.angleByTime(timeSeconds) - getAngle()), spinSpeed});
                if (ln.isStopRequested()) {
                    enableBrakes();
                    stopDrive();
                    return false;
                }
                if (Math.hypot(follow.xOfTime(timeSeconds) - getX(), follow.yOfTime(timeSeconds) - getY()) > positionError) {
                    stopDrive();
                    ln.telemetry.addData("Off course:", "Recalculating");
                    ln.telemetry.update();
                    return maneuverToPositionSpinny(goalX, goalY, velocity, spinSpeed);
                }
            }
        }
        enableBrakes();
        stopDrive();
        return true;
    }
    public boolean maneuverToPositionTransitionToFinalAngle(double goalX, double goalY, double velocity, double maintainedAngle, double finalAngle, double finalAngleDistanceOverride, double positionFactor, double angleFactor){
        disableBrakes();
        if(!(toPixelSpace(pos.getX())==toPixelSpace(goalX) && toPixelSpace(pos.getY())==toPixelSpace(goalY))) {
            boolean[][] instanceObstacles = PathInterface.drawSquare(ob, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(maxRobotRadius), getAngle(), false);
            AStarSearch aStar = new AStarSearch(ln, instanceObstacles, toPixelSpace(pos.getX()), toPixelSpace(pos.getY()), toPixelSpace(goalX), toPixelSpace(goalY), AStarSearch.AlgorthmType.DIJKSTRA);

            ArrayList<AStarSearch.Node> path;
            if (aStar.getFinalPath() != null) {
                path = aStar.getFinalPath();
            } else {
                return false;
            }

            path = PathInterface.condense(path);
            path = PathInterface.findShortCuts(path, instanceObstacles);
            Waypoint[] purePursuitInput = PathInterface.nodesToWaypoints(path, (1 / pixelsPerMeter));
            purePursuitInput = PathInterface.mergePoints(purePursuitInput, 0.10);
            PurePursuitPath follow = new PurePursuitPath(purePursuitInput);
            follow.mapTimes(velocity, this);

            double startTime = System.currentTimeMillis();
            boolean angleOverride = false;
            while (System.currentTimeMillis() - startTime < follow.getTotalPathTime() * 1000.0) {
                double timeSeconds = (System.currentTimeMillis() - startTime) / 1000.0;
                double angleCorrection;
                if (ACMath.compassAngleShorter(maintainedAngle + getAngle(), follow.angleByTime(timeSeconds))) {
                    angleCorrection = ACMath.toCompassAngle(follow.angleByTime(timeSeconds)) - ACMath.toCompassAngle(maintainedAngle + getAngle());
                } else {
                    angleCorrection = ACMath.toStandardAngle(follow.angleByTime(timeSeconds)) - ACMath.toStandardAngle(maintainedAngle + getAngle());
                }
                double xCorrection = follow.xOfTime(timeSeconds) - getX();
                double yCorrection = follow.yOfTime(timeSeconds) - getY();
                double[] speeds;
                if (Math.hypot(goalX - getX(), goalY - getY()) < finalAngleDistanceOverride) {
                    stopDrive();
                    ln.telemetry.addData("Status", "Turning");
                    ln.telemetry.update();
                    return maneuverToPositionFinalAngleAccurate(goalX, goalY, 3 * velocity / 4, finalAngle, positionFactor, angleFactor);
                } else {
                    speeds = inverseKinematics(new double[]{follow.linearVelocityOfTime(timeSeconds) * Math.cos(follow.angleByTime(timeSeconds) - getAngle()) + linearCorrectSpeed * (xCorrection / positionError) * Math.cos(getAngle()), follow.linearVelocityOfTime(timeSeconds) * Math.sin(follow.angleByTime(timeSeconds) - getAngle()) + linearCorrectSpeed * (yCorrection / positionError) * Math.sin(getAngle()), (follow.anglePrimeByTime(timeSeconds) + m * angleCorrection / s)});
                    ln.telemetry.addData("Status", "Normal");
                    ln.telemetry.update();
                }
                if (ln.isStopRequested()) {
                    enableBrakes();
                    stopDrive();
                    return false;
                }
                if (Math.hypot(xCorrection, yCorrection) > positionError) {
                    stopDrive();
                    ln.telemetry.addData("Off course:", "Recalculating");
                    ln.telemetry.update();
                    return maneuverToPositionTransitionToFinalAngle(goalX, goalY, velocity, maintainedAngle, finalAngle, finalAngleDistanceOverride, positionFactor, angleFactor);
                }
                pos.update();
                angle.update();
            }
        }
        enableBrakes();
        stopDrive();
        return true;
    }




    /*****Random Accessors for constants and other data. Used by other classes etc.*****/

    public double getLENGTH_Y() {
        return LENGTH_Y;
    }
    public void setLENGTH_Y(double LENGTH_Y) {
        this.LENGTH_Y = LENGTH_Y;
        this.inverseKinematicsMatrix = new double[][]{{1, -1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, 1, ((LENGTH_X + LENGTH_Y)/2)},
                {1, 1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, -1, ((LENGTH_X + LENGTH_Y)/2)}};
    }
    public double getLENGTH_X() {
        return LENGTH_X;
    }
    public void setLENGTH_X(double LENGTH_X) {
        this.LENGTH_X = LENGTH_X;
        this.inverseKinematicsMatrix = new double[][]{{1, -1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, 1, ((LENGTH_X + LENGTH_Y)/2)},
                {1, 1, (-(LENGTH_X + LENGTH_Y)/2)},
                {1, -1, ((LENGTH_X + LENGTH_Y)/2)}};
    }
    public double getWheelRadius(){
        return wheelRadius;
    }
    public double getMotorMaxAngularVelocity(){
        return motorMaxRPM*rpmToRadPerSec;
    }
}
