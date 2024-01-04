package org.firstinspires.ftc.teamcode.ThreadsandInterfaces;

import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvPipeline;

public class BeaconPipeline extends OpenCvPipeline {
    private NewBeaconDetector beaconDetector = new NewBeaconDetector();

    public int magentaPix;
    public int orangePix;
    public int greenPix;

    NewBeaconDetector.BeaconColor beaconColor;

    @Override
    public Mat processFrame(Mat frame) {
        beaconColor = beaconDetector.detect(frame);
//        frame.setTo(new Scalar(255, 255, 255), beaconDetector.getMaskYellow());
        magentaPix = beaconDetector.getMagentaPix();
        orangePix = beaconDetector.getOrangePix();
        greenPix = beaconDetector.getGreenPix();

        return frame;
    }

    public NewBeaconDetector.BeaconColor getBeaconColor(){
        return beaconColor;
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

    public Scalar getLowerMagenta() {
        return beaconDetector.getLowerMagenta();
    }

    public Scalar getUpperMagenta() {
        return beaconDetector.getUpperMagenta();
    }


    public void setLowerMagenta(Scalar scalar){
        beaconDetector.setLowerMagenta(scalar);
    }

    public void setUpperMagenta(Scalar scalar){
        beaconDetector.setUpperMagenta(scalar);
    }

}