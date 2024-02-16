package org.firstinspires.ftc.teamcode.Implementations.Robot;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.CachingGainControl;
import org.firstinspires.ftc.robotcore.internal.collections.MutableReference;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold_Frontstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Frontstage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class Camera {
    public AprilTagProcessor atag;

    private ExposureControl exposure;
    private GainControl gain;


    public BluePropThreshold_Backstage bluePropBackstage;
    public BluePropThreshold_Frontstage bluePropFrontstage;
    public RedPropThreshold_Backstage redPropThresholdBackstage;
    public RedPropThreshold_Frontstage redPropThresholdFrontstage;

    public VisionProcessor processor;
    public WebcamName backCam, frontCam;
    public VisionPortal vision;

    public Robot robot;

    public Camera(HardwareMap hardwareMap,Robot robot){

        if(robot.parameter.get("alliance").equals("RED") &&
                robot.parameter.get("position").equals("BACK_STAGE")){
            this.redPropThresholdBackstage=new RedPropThreshold_Backstage();

        }
        else if(robot.parameter.get("alliance").equals("RED") &&
                robot.parameter.get("position").equals("FRONT_STAGE")){
            this.redPropThresholdFrontstage= new RedPropThreshold_Frontstage();
        }

        else if(robot.parameter.get("alliance").equals("BLUE") &&
                robot.parameter.get("position").equals("FRONT_STAGE")){
            this.bluePropFrontstage=new BluePropThreshold_Frontstage();
        }
        else{
            this.bluePropBackstage= new BluePropThreshold_Backstage();

        }






        this.atag = new AprilTagProcessor.Builder().build();
        this.robot = robot;

        backCam = hardwareMap.get(WebcamName.class, "Camera1");
        frontCam = hardwareMap.get(WebcamName.class, "Camera2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(backCam,frontCam);


        this.getProcessor();
        vision = new VisionPortal.Builder()
                .setCamera(switchableCamera)
               //.addProcessors(atag,redPropThresholdBackstage)
                .addProcessors(atag,processor)
                .build();

        while(vision.getCameraState() != VisionPortal.CameraState.STREAMING){

        }
        //vision.setProcessorEnabled(redPropThresholdBackstage,false);
       vision.setProcessorEnabled(processor,false);

      //  exposure=vision.getCameraControl(ExposureControl.class);
      //  exposure.setMode(ExposureControl.Mode.Manual);
     //   exposure.setExposure(15, TimeUnit.MILLISECONDS);

      //  gain=vision.getCameraControl(GainControl.class);
       // gain.setGain(28);

    }

    public void openBackCam(){
        if(vision.getCameraState() == VisionPortal.CameraState.STREAMING){
            vision.setActiveCamera(backCam);
        }

        vision.setProcessorEnabled(atag,true);
        //vision.setProcessorEnabled(redPropThresholdBackstage,false);
        vision.setProcessorEnabled(this.processor,false);


        while(vision.getCameraState()!=VisionPortal.CameraState.STREAMING){

        }


    }

    public int getMaxGain(){

        gain=vision.getCameraControl(GainControl.class);

        return  gain.getMaxGain();

    }

    public int getMinGain(){

        gain=vision.getCameraControl(GainControl.class);

        return  gain.getMinGain();

    }

    public boolean isExposure(){

        exposure=vision.getCameraControl(ExposureControl.class);

        return exposure.isExposureSupported();

    }

    public void openFrontCam(){
        if(vision.getCameraState()== VisionPortal.CameraState.STREAMING){
            vision.setActiveCamera(frontCam);
        }

        vision.setProcessorEnabled(atag,false);
       //vision.setProcessorEnabled(redPropThresholdBackstage,true);
        vision.setProcessorEnabled(this.processor,true);


        while(vision.getCameraState()!=VisionPortal.CameraState.STREAMING){

        }

    }

    public void FrontCamAtag(){

        if(vision.getCameraState()== VisionPortal.CameraState.STREAMING){
            vision.setActiveCamera(frontCam);
        }

        vision.setProcessorEnabled(atag,true);
        //vision.setProcessorEnabled(redPropThresholdBackstage,true);
        vision.setProcessorEnabled(this.processor,false);


        while(vision.getCameraState()!=VisionPortal.CameraState.STREAMING){

        }

    }

    public void getProcessor(){
        if(robot.parameter.get("alliance").equals("RED") &&
                robot.parameter.get("position").equals("BACK_STAGE")){
            this.processor = this.redPropThresholdBackstage;

        }
        else if(robot.parameter.get("alliance").equals("RED") &&
                robot.parameter.get("position").equals("FRONT_STAGE")){
            this.processor = this.redPropThresholdFrontstage;
        }

        else if(robot.parameter.get("alliance").equals("BLUE") &&
                robot.parameter.get("position").equals("FRONT_STAGE")){
            this.processor = this.bluePropFrontstage;
        }
        else{
            this.processor = this.bluePropBackstage;

        }
    }

    public String GetPropPosition(){
        if(robot.parameter.get("alliance").equals("RED") &&
                robot.parameter.get("position").equals("BACK_STAGE")){
           return this.redPropThresholdBackstage.getPropPosition();

        }
        else if(robot.parameter.get("alliance").equals("RED") &&
                robot.parameter.get("position").equals("FRONT_STAGE")){
            return this.redPropThresholdFrontstage.getPropPosition();
        }

        else if(robot.parameter.get("alliance").equals("BLUE") &&
                robot.parameter.get("position").equals("FRONT_STAGE")){
            return this.bluePropFrontstage.getPropPosition();
        }
        else{
            return this.bluePropBackstage.getPropPosition();

        }
    }



}


