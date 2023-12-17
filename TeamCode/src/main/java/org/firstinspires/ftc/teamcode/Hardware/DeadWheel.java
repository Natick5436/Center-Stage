/**
 * ODOMITRY WHEEL CODE - Written by Nolan Palmer (2018 - 2020)
 * Edited by: [add your name and the current year here]
 * - Maxwell Harriss (2022)
 * - Adam Pochobut (2022)
 * - Even Hyman
 * ==============================
 * This code will track the position via Odometry,
 * Uses data from motion sesnses to estimate the change in position over time,
 * Used to estimate position relative to a starting location
 */

package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;


//AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA
public class DeadWheel {

    private double ticksPer;    // How many "time pieces" per rotation
    private double wheelCirc;   // Wheel circumference
    private DcMotor device;     // DCMotor object named "device"
    private int direction;      // What direction it's facing
    private DcMotor.Direction motorDirection;   // Direction of the motor
    private double deadWheelCorrection;         // The amount the Odo wheels correct by

    // Defines a DeadWheel using the robot's motors, regular configmotor is also reclassified as DcMotor
    // Each one has different requirements on what they are being used for (Determined by the values shown after configMotor)
    public DeadWheel(DcMotor configMotor, int direction){
        this.motorDirection = configMotor.getDirection();
        if(motorDirection == DcMotor.Direction.REVERSE){
            this.direction = -direction;
        }else {
            this.direction = direction;
        }
        device = configMotor;
        ticksPer = 1;
        wheelCirc = 0;
        deadWheelCorrection = 1;
    }

    public DeadWheel(DcMotor configMotor, double wheelDiameter, double ticksPer, int direction){
        this.motorDirection = configMotor.getDirection();
        if(motorDirection == DcMotor.Direction.REVERSE){
            this.direction = -direction;
        }else {
            this.direction = direction;
        }
        device = configMotor;
        this.wheelCirc = wheelDiameter*Math.PI;
        this.ticksPer = ticksPer;
        deadWheelCorrection = 1;
    }

    public DeadWheel(DcMotor configMotor, double wheelDiameter, double ticksPer, int direction, double deadWheelCorrection){
        this.motorDirection = configMotor.getDirection();
        if(motorDirection == DcMotor.Direction.REVERSE){
            this.direction = -direction;
        }else {
            this.direction = direction;
        }
        device = configMotor;
        this.wheelCirc = wheelDiameter*Math.PI;
        this.ticksPer = ticksPer;
        this.deadWheelCorrection = deadWheelCorrection;
    }
    // DEADWHEELS set up end here ----------------------------------


    // Makes sure that the direction of the Odo wheel is not backwards
    public void updateParameters(){
        if(motorDirection != device.getDirection()){
            direction = -direction;
            motorDirection = device.getDirection();
        }
    }
    // Sets the direction to a value
    public void setDirection(int d) {
        if(motorDirection == DcMotor.Direction.REVERSE) {
            direction = -d;
        }else{
            direction = d;
        }
    }
    // This program is setting the current position to the value given by the Odo wheel and setting it to positive or negative
    public int getCurrentPosition(){
        updateParameters();
        return direction*device.getCurrentPosition();
    }
    // This part of the program gets how far the robot has gone from the origen
    public double distanceTraveled(){
        return deadWheelCorrection*this.wheelCirc*getCurrentPosition()/this.ticksPer;
    }

    public double getWheelCirc() {
        return wheelCirc;
    }

    public void setWheelCirc(double wheelCirc) {
        this.wheelCirc = wheelCirc;
    }

    public double getTicksPerRev() {
        return ticksPer;
    }

    public void setTicksPerRev(double ticksPer) {
        this.ticksPer = ticksPer;
    }
}
