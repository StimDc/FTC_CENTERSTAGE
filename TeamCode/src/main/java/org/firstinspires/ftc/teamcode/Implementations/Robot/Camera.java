package org.firstinspires.ftc.teamcode.Implementations.Robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Camera {
    public AprilTagProcessor atag;
    public BluePropThreshold blueProp;

    public WebcamName backCam, frontCam;
    public VisionPortal vision;
    public Camera(HardwareMap hardwareMap){ //TODO: make the difference between blueprop and redprop
        atag = new AprilTagProcessor.Builder().build();
        blueProp = new BluePropThreshold();

        backCam = hardwareMap.get(WebcamName.class, "Camera1");
        frontCam = hardwareMap.get(WebcamName.class, "Camera2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(backCam,frontCam);

        vision = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessors(atag,blueProp)
                .build();

        while(vision.getCameraState() != VisionPortal.CameraState.STREAMING){

        }
        vision.setProcessorEnabled(blueProp,false);
    }

    public void openBackCam(){
        if(vision.getCameraState() == VisionPortal.CameraState.STREAMING){
            vision.setActiveCamera(backCam);
        }

        vision.setProcessorEnabled(atag,true);
        vision.setProcessorEnabled(blueProp,false);

        while(vision.getCameraState()!=VisionPortal.CameraState.STREAMING){

        }
    }
    public void openFrontCam(){
        if(vision.getCameraState()== VisionPortal.CameraState.STREAMING){
            vision.setActiveCamera(frontCam);
        }

        vision.setProcessorEnabled(atag,false);
        vision.setProcessorEnabled(blueProp,true);
        while(vision.getCameraState()!=VisionPortal.CameraState.STREAMING){

        }
    }
}
