package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class NoLoadMotorInterface extends Thread {

    LinearOpMode ln;
    VoltageSensor voltage;
    DcMotor motor;
    public double s;
    public double power;

    public NoLoadMotorInterface(LinearOpMode ln, VoltageSensor voltage, DcMotor motor){
        this.ln = ln;
        this.voltage = voltage;
        this.motor = motor;
        s = 0;
        power = 0;
    }
    public void setSpeed(double s){
        this.s = s;
    }

    public void run(){
        while(ln.opModeIsActive()||!ln.isStarted()){
            setFlyWheelVoltageRegulated(s);
        }
    }

    public boolean setFlyWheelVoltageRegulated(double speed/*in rpm*/) {
        if (speed != 0) {
            double requiredVoltage = (speed + 600) / 500;
            power = requiredVoltage / voltage.getVoltage();
            if (power > 1) {
                motor.setPower(1);
                return false;
            } else {
                motor.setPower(power);
                return true;
            }
        } else {
            motor.setPower(0);
            return false;
        }
    }
}