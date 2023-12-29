package org.firstinspires.ftc.teamcode.Implementations.Robot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class CameraFunc {

    public static void openCamera(WebcamName cam,String position, VisionPortal vision, AprilTagProcessor atag, BluePropThreshold blueProp){

        if(vision.getCameraState() == VisionPortal.CameraState.STREAMING){
            vision.setActiveCamera(cam);
        }
        if(position.equals("front")) {
            vision.setProcessorEnabled(atag, false);
            vision.setProcessorEnabled(blueProp, true);
        }
        else{
            vision.setProcessorEnabled(atag,true);
            vision.setProcessorEnabled(blueProp,false);
        }

        while(vision.getCameraState() != VisionPortal.CameraState.STREAMING){

        }
    }
}
