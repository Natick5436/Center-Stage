package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class NewBeaconDetector {

    public enum BeaconColor {
        ORANGE,
        MAGENTA,
        GREEN
    }

    public Scalar getLowerMagenta() {
        return lowerMagenta;
    }

    public void setLowerMagenta(Scalar lowerMagenta) {
        this.lowerMagenta = lowerMagenta;
    }

    public Scalar getUpperMagenta() {
        return upperMagenta;
    }

    public void setUpperMagenta(Scalar upperMagenta) {
        this.upperMagenta = upperMagenta;
    }

    public Scalar lowerMagenta = new Scalar(80, 0, 50);
    public Scalar upperMagenta = new Scalar(275, 100, 170);
    public Scalar lowerGreen = new Scalar(30, 40, 70);//30, 40, 70
    public Scalar upperGreen = new Scalar(110, 255, 255);
    public Scalar lowerOrange = new Scalar(70, 30, 0);//70, 30, 0
    public Scalar upperOrange = new Scalar(255, 210, 100);

    public int magentaPix;
    public int orangePix;
    public int greenPix;

    Mat maskYellow = new Mat();
    Mat maskOrange = new Mat();
    Mat maskGreen = new Mat();



    public Mat getMaskGreen() {
        return maskGreen;
    }



    public Mat getMaskOrange() {
        return maskOrange;
    }



    public Mat getMaskYellow() {
        return maskYellow;
    }

    public BeaconColor detect(Mat frame) {


            // Convert the frame to the HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(frame, hsv, Imgproc.COLOR_RGB2HSV);

            // Create masks for each color
            maskYellow = new Mat();
            Core.inRange(hsv, lowerMagenta, upperMagenta, maskYellow);
            maskGreen = new Mat();
            Core.inRange(hsv, lowerGreen, upperGreen, maskGreen);
            maskOrange = new Mat();
            Core.inRange(hsv, lowerOrange, upperOrange, maskOrange);

            // Combine the masks
//            Mat mask = new Mat();
//            Core.add(maskYellow, maskGreen, mask);
//            Core.add(mask, maskOrange, mask);

            // Find contours in the mask
//            List<MatOfPoint> contours = new ArrayList<>();
//            Mat hierarchy = new Mat();
//            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            // If no contours are found, return NONE
//            if (contours.size() == 0) {
//                return BeaconColor.NONE;
//            }

            // Otherwise, determine which color the beacon is based on the contours
/*
            yellowPix = Core.countNonZero(maskYellow);
            orangePix = Core.countNonZero(maskOrange);
            greenPix = Core.countNonZero(maskGreen);

            if (Core.countNonZero(maskYellow) > Core.countNonZero(maskGreen) && Core.countNonZero(maskYellow) > Core.countNonZero(maskOrange)) {
                return BeaconColor.YELLOW;
            } else if (Core.countNonZero(maskGreen) > Core.countNonZero(maskYellow) && Core.countNonZero(maskGreen) > Core.countNonZero(maskOrange)) {
                return BeaconColor.GREEN;
            } else {
                return BeaconColor.ORANGE;
            }*/

        magentaPix = Core.countNonZero(maskYellow);
        greenPix = Core.countNonZero(maskGreen);
        orangePix = Core.countNonZero(maskOrange);

// Determine which color is the most intense
        if (magentaPix > greenPix && magentaPix > orangePix) {
            return BeaconColor.MAGENTA;
        } else if (greenPix > magentaPix && greenPix > orangePix) {
            return BeaconColor.GREEN;
        } else {
            return BeaconColor.ORANGE;
        }

    }

    public int getMagentaPix(){
        return magentaPix;
    }

    public int getOrangePix(){
        return orangePix;
    }

    public int getGreenPix(){
        return greenPix;
    }


}
