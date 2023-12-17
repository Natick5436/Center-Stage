package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

public interface PositionTracker {
    double getX();
    double getY();
    void update();
    void resetPosition(double x, double y);
}
