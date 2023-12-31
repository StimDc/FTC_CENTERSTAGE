package org.firstinspires.ftc.teamcode.ControlPart;

import static java.lang.Math.abs;
import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.DebugTools.CatchingBugs;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Camera View")

public class CameraView extends OpMode {

    private RedPropThreshold redProp;
    private VisionPortal visionPortal1,visionPortal2;
    private AprilTagProcessor apriltagProcesor;

    private boolean PressX=false, PressO=false;

    @Override
    public void init() {



        redProp=new RedPropThreshold();

        visionPortal1 = new VisionPortal.Builder()
                .addProcessor(apriltagProcesor)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera1"))
                .setCameraResolution(new Size(640,480))
                .build();

        visionPortal2=new VisionPortal.Builder()
                .addProcessor(redProp)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera2"))
                .setCameraResolution(new Size(640,480))
                .build();

        visionPortal2.stopStreaming();

        while(visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING){

        }

    }


    @Experimental()
    @Override
    public void loop() {

        if(gamepad1.a){
            if(!PressX){
                Cam_Front();
            }
            PressX=true;

        }else {
            PressX=false;
        }

        if(gamepad1.b){

            if(!PressO){
                Cam_Back();
            }
            PressO=true;

        }else{
            PressO=false;
        }

        telemetry.addLine("Front Camera: X");
        telemetry.addLine("Back Camera: O");
        telemetry.update();

    }

    public void Cam_Front(){

        visionPortal1.stopStreaming();
        while(visionPortal1.getCameraState() != VisionPortal.CameraState.STOPPING_STREAM){

        }
        sleep(500);
        visionPortal2.resumeStreaming();
        while(visionPortal2.getCameraState() != VisionPortal.CameraState.STREAMING){

        }

    }

    public void Cam_Back(){

        visionPortal2.stopStreaming();
        while(visionPortal2.getCameraState() != VisionPortal.CameraState.STOPPING_STREAM){

        }
        sleep(500);
        visionPortal1.resumeStreaming();
        while(visionPortal1.getCameraState() != VisionPortal.CameraState.STREAMING){

        }

    }

}
