package org.firstinspires.ftc.teamcode.Implementations.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold_Frontstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.Choising_Color;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Frontstage;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Dictionary;

public class Camera {
    public AprilTagProcessor atag;
 /*   public BluePropThreshold_Backstage bluePropBackstage;
    public BluePropThreshold_Frontstage bluePropFrontstage;
    public RedPropThreshold_Backstage redPropThresholdBackstage;
    public RedPropThreshold_Frontstage redPropThresholdFrontstage;


  */
public Choising_Color color;

    public BluePropThreshold_Backstage bluePropBackstage;
    public BluePropThreshold_Frontstage bluePropFrontstage;
    public RedPropThreshold_Backstage redPropThresholdBackstage;
    public RedPropThreshold_Frontstage redPropThresholdFrontstage;

    public VisionProcessor processor;
    public String positionProp;



    public WebcamName backCam, frontCam;
    public VisionPortal vision;

    int STARTPOSITION;

    public VisionProcessor colorProcessor;
    public Robot robot;


    /*
    public void getColor() {


        if (STARTPOSITION == -2) {

            bluePropBackstage = new BluePropThreshold_Backstage();

        } else if (STARTPOSITION == -1) {

            redPropThresholdBackstage = new RedPropThreshold_Backstage();


        } else if (STARTPOSITION == 1) {

            redPropThresholdFrontstage = new RedPropThreshold_Frontstage();

        } else if (STARTPOSITION == 2) {

            bluePropFrontstage = new BluePropThreshold_Frontstage();
        }else{

        }
    }

    */



    public Camera(HardwareMap hardwareMap, int startPosition,Robot robot){ //1 redFrontstage,-1 redbackstage, 2 bluefrontstage, -2 bluebackstage
        //TODO: make the difference between bluePropBackstage and redprop

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
       // bluePropBackstage = new BluePropThreshold_Backstage();

       //  color=new Choising_Color();

      //  color.getStartPosition(startPosition);

        //STARTPOSITION= startPosition;

        //getColor();


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
/*
    public String getPositionProp(){



    }
    
 */



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

    public String GetPropPositionr(){
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


