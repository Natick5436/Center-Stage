/**
 * odometry WHEEL CODE - Written by Maxwell Harriss (2023 - xxxx)
 * Edited by: [add your name and the current year here]
 * -
 * ==============================
 * This code will track the position via Odometry,
 * Used to estimate position relative to a starting location
 */

package org.firstinspires.ftc.teamcode.Hardware;

// init values

public class odometry
{
    private double WheelCircumference; // Wheel circumference

    private double DistenceY; // how far left or right from the starting position

    private double DistenceX; // how far forward or backward from starting position

    private int DriveZoneX; // holds the X value for the area that the robot can drive based on the odo wheels

    private int DriveZoneY; // holds the Y value for the area that the robot can drive based on the odo wheels


    /* the constructor for the odo wheels which takes the values in inches
     Wheel is for the Circumference of the wheels
     y & x are for the starting y and x positions based on the corner being (0,0)
     Size is for the
     */
    public odometry (double wheel, double y, double x, int Width, int Lenght)
    {

        WheelCircumference = wheel;

        DistenceY = y;

        DistenceX = x;

        DriveZoneX = Lenght;

        DriveZoneY = Width;

    }
    public odometry (double wheel, double y, double x)
    {

        WheelCircumference = wheel;

        DistenceY = y;

        DistenceX = x;

        DriveZoneX = 18;

        DriveZoneY = 18;

    }


    // getter methods to return the x and y value stored by the odometry
    public double getX()
    {
        return DistenceX;
    }

    public double getY()
    {
        return DistenceY;
    }



    // Checks if the robot is still moving by taking a time and waiting a few seconds and then taking another and finding the diffrence.
    public boolean checkSpeed()
    {
        double tempX = DistenceX;
        double tempY = DistenceY;



        if (tempX - DistenceX < .5 && tempY - DistenceY < .5)
        {
            return true;
        } else
        {
            return false;
        }
    }


    // returns how far away from the walls the robot is on the x axis
    public double DistenceToWallX()
    {

        if (DistenceX > DriveZoneX/2)
        {
            return (DriveZoneX - DistenceX);
        } else
        {
            return DistenceX;
        }
    }
    // returns how far away from the walls the robot is on the y axis
    public double DistenceToWallY()
    {
        if (DistenceY > DriveZoneY/2)
        {
            return (DriveZoneY - DistenceY);
        } else
        {
            return DistenceY;
        }
    }
}

