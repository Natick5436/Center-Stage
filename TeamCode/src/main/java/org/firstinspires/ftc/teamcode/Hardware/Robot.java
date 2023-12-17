package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class Robot {
    public LinearOpMode ln;
    public Robot(){
        ln = null;
    }
    public Robot(LinearOpMode ln){
        this.ln = ln;
    }

    public abstract void forward(double power);
    public abstract void turn(double power);
    public abstract void stopDrive();
}
