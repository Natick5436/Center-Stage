package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;

public class Odometry extends Thread implements PositionTracker, AngleTracker{
    //wheelDiameter in centimeters.
    double wheelDiameter;
    //wheel Circumference in meters for ease of odometry use.
    double wheelCircum;
    //ticks per full revolution of the motor axle
    double ticksPerMotorRev;
    double motorGearRatio;
    //ticks per full revolution of the wheel
    double ticksPer;
    //distance in meters between the two encoded wheels
    double LENGTH;
    //offset numbers
    double xOffset = 0.05;
    double yOffset = 0.075;
    double naturalMiddleMovementPerRadian = 0.082572;

    public double odometryX;
    public double odometryY;
    public double odometryAngle;
    double lastEncoderR, lastEncoderL, lastEncoderMiddle;
    double currentEncoderL, currentEncoderR, currentEncoderMiddle;
    double velocityL, velocityR, velocityMiddle;
    double lastTime, currentTime;

    public DeadWheel left;
    public DeadWheel right;
    public DeadWheel middle;

    LinearOpMode ln;
    public Odometry(LinearOpMode linear, DeadWheel l, DeadWheel r, DeadWheel m, double wheelDiameter, double ticksPerMotorRev, double motorGearRatio, double LENGTH, double naturalMiddleMovementPerRadian, double initialX, double initialY, double initialAngle, int lDirection, int rDirection, int mDirection){
        ln = linear;
        left = l;
        right = r;
        middle = m;
        this.wheelDiameter = wheelDiameter;
        this.wheelCircum = wheelDiameter*Math.PI/100.0;
        this.ticksPerMotorRev = ticksPerMotorRev;
        this.motorGearRatio = motorGearRatio;
        this.ticksPer = motorGearRatio*ticksPerMotorRev;
        this.LENGTH = LENGTH;
        this.naturalMiddleMovementPerRadian = naturalMiddleMovementPerRadian;
        odometryX = initialX;
        odometryY = initialY;
        odometryAngle = initialAngle;
        l.setDirection(lDirection);
        m.setDirection(mDirection);
        r.setDirection(rDirection);
        velocityL = 0;
        velocityR = 0;
        velocityMiddle = 0;
        currentTime = 0;
        currentEncoderL = 0;
        currentEncoderR = 0;
        currentEncoderMiddle = 0;
        lastEncoderL = left.distanceTraveled();
        lastEncoderR = right.distanceTraveled();
        lastEncoderMiddle = middle.distanceTraveled();
        lastTime = System.currentTimeMillis();
    }

    public double getX(){
        return odometryX;
    }
    public double getY(){
        return odometryY;
    }
    public double getAngle(){
        return odometryAngle;
    }
    public void setX(double x){
        odometryX = x;
    }
    public void setY(double y){
        odometryY = y;
    }
    public void resetAngle(double angle){
        odometryAngle = angle;
    }
    public double getVelocityL(){return velocityL;}
    public double getVelocityR(){return velocityR;}
    public double getVelocityMiddle(){return velocityMiddle;}
    public void run(){
        while(ln.opModeIsActive()||!ln.isStarted()){
            update();
        }
    }
    public void update(){
            currentEncoderL = left.distanceTraveled();
            currentEncoderR = right.distanceTraveled();
            currentEncoderMiddle = middle.distanceTraveled();
            currentTime = System.currentTimeMillis();
            double dL = currentEncoderL-lastEncoderL;
            double dR = currentEncoderR-lastEncoderR;
            double dM = (dL + dR) / 2;
            double dAngle = (dR - dL) / LENGTH;
            //Middle dead wheel is broken so we added a correction feature (scale factor) test
            double dMiddle = currentEncoderMiddle-lastEncoderMiddle;
            //measure of how much the middle wheel is different then the expected distance of a regular arch
            double dMidChange;
            if(dAngle != 0) {
                //constant is the middle to angle ratio found in the calibration program
                dMidChange = dMiddle - dAngle*naturalMiddleMovementPerRadian;
            }else{
                dMidChange = dMiddle;
            }
            velocityL = 1000*dL/(currentTime-lastTime);
            velocityR = 1000*dR/(currentTime-lastTime);
            velocityMiddle = 1000*dMiddle/(currentTime-lastTime);


            if(dAngle != 0) {
                odometryX = odometryX + (dM * Math.sin(dAngle) * Math.cos(odometryAngle + (dAngle / 2)) /
                        (dAngle * Math.cos(dAngle / 2))) + (dMidChange*Math.cos((odometryAngle + (dAngle/2))-Math.PI/2));
                odometryY = odometryY + (dM * Math.sin(dAngle) * Math.sin(odometryAngle + (dAngle / 2)) /
                        (dAngle * Math.cos(dAngle / 2))) + (dMidChange*Math.sin((odometryAngle + (dAngle/2))-Math.PI/2));
                odometryAngle = odometryAngle + dAngle;
            }else{
                odometryX = odometryX + (dM * Math.cos(odometryAngle)) + (dMidChange*Math.cos((odometryAngle)-Math.PI/2));
                odometryY = odometryY + (dM * Math.sin(odometryAngle)) + (dMidChange*Math.sin((odometryAngle)-Math.PI/2));
                odometryAngle = odometryAngle + dAngle;
            }
            lastEncoderL = currentEncoderL;
            lastEncoderR = currentEncoderR;
            lastEncoderMiddle = currentEncoderMiddle;
            lastTime = currentTime;
    }
    public void resetPosition(double x, double y){
        odometryX = x;
        odometryY = y;
    }
}
