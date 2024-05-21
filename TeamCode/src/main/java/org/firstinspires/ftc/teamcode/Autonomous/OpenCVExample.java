package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous
@Disabled
public class OpenCVExample extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {

        WebcamName webcamName = hardwareMap.get(WebcamName.class,"webcam1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId","id",hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            public void onOpened() {
                webcam1.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int errorCode) {
            }
        });
    }

    @Override
    public void loop() {

    }

    class examplePipeline extends OpenCvPipeline {
        Mat hsv = new Mat();
        Mat redHigh = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        double leftavgfin;
        double rightavgfin;
        Mat outPut = new Mat();

        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input,hsv,Imgproc.COLOR_RGB2HSV);
            telemetry.addLine("pipeline running");

            //green 80,0,0 to 150,255,255
            //yellow 30,0,0 to 90,255,255
            Scalar lowHSV = new Scalar(80,0,0);
            Scalar highHSV = new Scalar(150,255,255);

            Core.inRange(hsv,lowHSV,highHSV,outPut);

            Rect leftRect = new Rect(1,1,319,359);
            Rect rightRect = new Rect(320,1,319,359);

            leftCrop = outPut.submat(leftRect);
            rightCrop = outPut.submat(rightRect);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);

            leftavgfin = leftavg.val[0];
            rightavgfin = rightavg.val[0];

            telemetry.addLine("Left: " + Double.toString(leftavgfin));
            telemetry.addLine("Right: " + Double.toString(rightavgfin));

            if (leftavgfin > rightavgfin) {
                telemetry.addLine("Left");
            }
            else {
                telemetry.addLine("Right");
            }
            return(outPut);

        }

    }
}
