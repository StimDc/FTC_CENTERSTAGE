package org.firstinspires.ftc.teamcode.Implementations.Camera;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class

RedPropThreshold_Backstage implements VisionProcessor {


    private double left,center;

    Mat testMat= new Mat();
    Mat highMat=new Mat();
    Mat lowMat=new Mat();
    Mat finalMat=new Mat();

    public static double RedPixels;
    double redThreshold=0.1d;//0.049d

    String outStr="center";


    static final Rect LEFT_RECTANGLE=new Rect (
            new Point(0,250),
            new Point(270,480)
    );
    static final Rect CENTER_RECTANGLE=new Rect(
            new Point(271,250),
            new Point(640,480)
    );

    /*
    static final Rect RIGHTT_RECTANGLE=new Rect(
            new Point(428,0),
            new Point(640,480)
    );

     */

    /**
     * @param width
     * @param height
     * @param calibration
     */
    @Override
    public void init(int width, int height, CameraCalibration calibration) {

    }

    /**
     * @param frame
     * @param captureTimeNanos
     * @return
     */
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame,testMat,Imgproc.COLOR_RGB2HSV);

        Scalar lowHSVRedLower=new Scalar(0,100,20);
        Scalar lowHSVRedUpper=new Scalar(15,255,255);

        Scalar highHSVRedLower=new Scalar(172,100,20);
        Scalar highHSVRedUpper=new Scalar(180,255,255);

        Core.inRange(testMat,lowHSVRedLower,lowHSVRedUpper,lowMat);
        Core.inRange(testMat,highHSVRedLower,highHSVRedUpper,highMat);

        testMat.release();

        Core.bitwise_or(lowMat,highMat,finalMat);

        lowMat.release();
        highMat.release();

        ///METODA 1: adunam toti pixelii din fiecare regiune, suma o impartim la numarul de pixeli total si apoi comparam rezultatul final cu redThreshold

        double leftBox=Core.sumElems(finalMat.submat(LEFT_RECTANGLE)).val[0];
        double centerBox=Core.sumElems(finalMat.submat(CENTER_RECTANGLE)).val[0];
        // double rightBox=Core.sumElems(finalMat.submat(RIGHTT_RECTANGLE)).val[0];

        double averageLeftBox=leftBox/LEFT_RECTANGLE.area()/255;
        double averageCenterBox=centerBox/CENTER_RECTANGLE.area()/255;

        left=averageLeftBox;
        center=averageCenterBox;

        RedPixels = centerBox;

        // double averageRightBox=rightBox/RIGHTT_RECTANGLE.area()/255;

        // double maxim=Math.max(Math.max(averageLeftBox,averageRightBox),averageCenterBox);

        double maxim=Math.max(averageLeftBox,averageCenterBox);


        if(averageLeftBox>redThreshold && averageLeftBox>averageCenterBox){
            outStr="left";
        }else if(averageCenterBox>redThreshold && averageCenterBox>averageLeftBox){
            outStr="center";
        }else{

            outStr="right";

        }
        /*else if(averageRightBox==maxim){
            outStr="right";
        }



         */

        return null;
    }

    /**
     * @param canvas
     * @param onscreenWidth
     * @param onscreenHeight
     * @param scaleBmpPxToCanvasPx
     * @param scaleCanvasDensity
     * @param userContext
     */
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public String getPropPosition(){
        return outStr;
    }

    public double PrintLeft(){

        return left;

    }

    public  double PrintCenter(){

        return  center;

    }

}

