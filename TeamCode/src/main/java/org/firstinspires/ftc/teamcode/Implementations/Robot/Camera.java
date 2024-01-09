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

public class Camera {
    public AprilTagProcessor atag;
 /*   public BluePropThreshold_Backstage bluePropBackstage;
    public BluePropThreshold_Frontstage bluePropFrontstage;
    public RedPropThreshold_Backstage redPropThresholdBackstage;
    public RedPropThreshold_Frontstage redPropThresholdFrontstage;


  */
public Choising_Color color;

    public WebcamName backCam, frontCam;
    public VisionPortal vision;

    public VisionProcessor colorProcessor;

    public Camera(HardwareMap hardwareMap, int startPosition){ //1 redFrontstage,-1 redbackstage, 2 bluefrontstage, -2 bluebackstage
        //TODO: make the difference between bluePropBackstage and redprop
        atag = new AprilTagProcessor.Builder().build();
       // bluePropBackstage = new BluePropThreshold_Backstage();

        color=new Choising_Color();

        color.getStartPosition(startPosition);


        backCam = hardwareMap.get(WebcamName.class, "Camera1");
        frontCam = hardwareMap.get(WebcamName.class, "Camera2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(backCam,frontCam);

        vision = new VisionPortal.Builder()
                .setCamera(switchableCamera)
               // .addProcessors(atag,bluePropBackstage)
                .addProcessors(atag,color.getColor())
                .build();

        while(vision.getCameraState() != VisionPortal.CameraState.STREAMING){

        }
        //vision.setProcessorEnabled(bluePropBackstage,false);
        vision.setProcessorEnabled(color.getColor(),false);

    }

    public void openBackCam(){
        if(vision.getCameraState() == VisionPortal.CameraState.STREAMING){
            vision.setActiveCamera(backCam);
        }

        vision.setProcessorEnabled(atag,true);
        //vision.setProcessorEnabled(bluePropBackstage,false);
        vision.setProcessorEnabled(color.getColor(),false);


        while(vision.getCameraState()!=VisionPortal.CameraState.STREAMING){

        }
    }
    public void openFrontCam(){
        if(vision.getCameraState()== VisionPortal.CameraState.STREAMING){
            vision.setActiveCamera(frontCam);
        }

        vision.setProcessorEnabled(atag,false);
       // vision.setProcessorEnabled(bluePropBackstage,true);
        vision.setProcessorEnabled(color.getColor(),true);


        while(vision.getCameraState()!=VisionPortal.CameraState.STREAMING){

        }
    }

    public String getPositionProp(){

        return color.getProp();

    }

}
