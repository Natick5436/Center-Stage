package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Hardware.DeadWheel;
import org.firstinspires.ftc.teamcode.Math.ACMath;


public class TwoWheelOdometry extends Thread implements PositionTracker, AngleTracker{

    LinearOpMode ln;
    public DeadWheel h, v;
    public AngleTracker a;
    double odometryX, odometryY;
    double centerToHorizontal, centerToVertical;

    double currentH, currentV, currentAngle, currentTime;
    double lastH, lastV, lastAngle, lastTime;

    public TwoWheelOdometry(LinearOpMode linear, DeadWheel horizontal, DeadWheel vertical, AngleTracker angle, double centerToHorizontal, double centerToVertical, double initialX, double initialY){
        ln = linear;
        h = horizontal;
        v = vertical;
        a = angle;
        this.centerToHorizontal = centerToHorizontal;
        this.centerToVertical = centerToVertical;
        odometryX = initialX;
        odometryY = initialY;
    }
    public double getX(){
        return odometryX;
    }
    public double getY(){
        return odometryY;
    }
    public double getAngle(){
        return a.getAngle();
    }

    double dH, dV, dA, dt;
    double averageAngle;
    public void update(){
        currentH = h.distanceTraveled();
        currentV = v.distanceTraveled();
        currentAngle = a.getAngle();
        currentTime = System.currentTimeMillis();
        dH = currentH-lastH;
        dV = currentV-lastV;
        dA = currentAngle-lastAngle;
        dt = currentTime-lastTime;

        if(ACMath.compassAngleShorter(currentAngle, lastAngle)) {
            dA = ACMath.toCompassAngle(currentAngle)-ACMath.toCompassAngle(lastAngle);
            averageAngle = (ACMath.toCompassAngle(currentAngle) + ACMath.toCompassAngle(lastAngle)) / 2;
        }else{
            dA = ACMath.toStandardAngle(currentAngle)-ACMath.toStandardAngle(lastAngle);
            averageAngle = (ACMath.toStandardAngle(currentAngle) + ACMath.toStandardAngle(lastAngle)) / 2;
        }

        odometryX = odometryX + (dH-centerToHorizontal*dA)*Math.cos(averageAngle) + (dV-centerToVertical*dA)*Math.cos(averageAngle+Math.PI/2);
        odometryY = odometryY + (dH-centerToHorizontal*dA)*Math.sin(averageAngle) + (dV-centerToVertical*dA)*Math.sin(averageAngle+Math.PI/2);

        lastH = currentH;
        lastV = currentV;
        lastAngle = currentAngle;
        lastTime = currentTime;
    }
    public void run(){
        lastH = h.distanceTraveled();
        lastV = v.distanceTraveled();
        lastAngle = a.getAngle();
        lastTime = System.currentTimeMillis();
        while(ln.opModeIsActive()||!ln.isStarted()){
            update();
        }
    }
    public void resetPosition(double x, double y){
        odometryX = x;
        odometryY = y;
    }
    public void resetAngle(double angle){
        a.resetAngle(angle);
    }
}
