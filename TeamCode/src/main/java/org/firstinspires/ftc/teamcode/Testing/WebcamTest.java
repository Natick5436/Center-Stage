/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.BeaconPipeline;
import org.firstinspires.ftc.teamcode.ThreadsandInterfaces.NewBeaconDetector;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */
@Autonomous(name="WebcamConfig", group = "TeleOp")
public class WebcamTest extends LinearOpMode
{
    OpenCvInternalCamera phoneCam;
    BeaconPipeline pipeline;

    @Override
    public void runOpMode()
    {
        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        pipeline = new BeaconPipeline();
        phoneCam.setPipeline(pipeline);

        // We set the viewport policy to optimized view so the preview doesn't appear 90 deg
        // out when the RC activity is in portrait. We do our actual image processing assuming
        // landscape orientation, though.
        phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        Scalar lowerMagenta = new Scalar(80, 0, 130);
        Scalar upperMagenta = new Scalar(255, 50, 255);

        int lr = 80;
        int lg= 0;
        int lb = 130;

        int ur = 255;
        int ug= 50;
        int ub = 255;



        while (opModeInInit())
        {
            NewBeaconDetector.BeaconColor beaconPipeline = pipeline.getBeaconColor();
            telemetry.addData("Analysis", beaconPipeline);
            telemetry.addData("Magenta", pipeline.getMagentaPix());
            telemetry.addData("Orange", pipeline.getOrangePix());
            telemetry.addData("Green TEST", pipeline.getGreenPix());
            telemetry.addData("Lower range", lowerMagenta);
            telemetry.addData("upper range", upperMagenta);


            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive()){
            //LOWER LIMIT
            if(gamepad1.y){
                lr++;
                pipeline.setLowerMagenta(new Scalar(lr, lg, lb));
            }else if(gamepad1.a){
                lr--;
                pipeline.setLowerMagenta(new Scalar(lr, lg, lb));
            }

            if(gamepad1.x){
                lg++;
                pipeline.setLowerMagenta(new Scalar(lr, lg, lb));
            }else if(gamepad1.b){
                lg--;
                pipeline.setLowerMagenta(new Scalar(lr, lg, lb));
            }

            if(gamepad1.dpad_up){
                lb++;
                pipeline.setLowerMagenta(new Scalar(lr, lg, lb));
            }else if(gamepad1.dpad_down){
                lb--;
                pipeline.setLowerMagenta(new Scalar(lr, lg, lb));
            }

            //UPPER LIMIT
            if(gamepad1.dpad_right){
                ur++;
                pipeline.setUpperMagenta(new Scalar(ur, ug, ub));
            }else if(gamepad1.dpad_left){
                ur--;
                pipeline.setUpperMagenta(new Scalar(ur, ug, ub));
            }

            if(gamepad1.right_bumper){
                ug++;
                pipeline.setUpperMagenta(new Scalar(ur, ug, ub));
            }else if(gamepad1.left_bumper){
                ug--;
                pipeline.setUpperMagenta(new Scalar(ur, ug, ub));
            }

            if(gamepad1.right_trigger > 0){
                ub++;
                pipeline.setUpperMagenta(new Scalar(ur, ug, ub));
            }else if(gamepad1.left_trigger > 0){
                ub--;
                pipeline.setUpperMagenta(new Scalar(ur, ug, ub));
            }


            NewBeaconDetector.BeaconColor beaconPipeline = pipeline.getBeaconColor();
            telemetry.addData("Analysis", beaconPipeline);
            telemetry.addData("Magenta", pipeline.getMagentaPix());
            telemetry.addData("Orange", pipeline.getOrangePix());
            telemetry.addData("Green TEST", pipeline.getGreenPix());


            telemetry.addData("Lower", pipeline.getLowerMagenta());
            telemetry.addData("Upper", pipeline.getUpperMagenta());

            telemetry.update();
            //phoneCam.stopStreaming();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
    }

}
