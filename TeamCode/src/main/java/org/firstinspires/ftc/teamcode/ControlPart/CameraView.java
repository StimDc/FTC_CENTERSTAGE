package org.firstinspires.ftc.teamcode.ControlPart;

import static java.lang.Math.abs;
import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.io.IOException;

@TeleOp(name = "Camera View"
)

public class CameraView extends OpMode {

    private Robot robot;

    private boolean PressX=false, PressO=false,once=true;

    private boolean baclCam=false;

    @Override
    public void init() {

        try {
            robot = new Robot(hardwareMap,telemetry,-1);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }


    @Experimental()
    @Override
    public void loop() {

        if(gamepad1.a){
            if(!PressX){
                robot.camera.openFrontCam();
                baclCam=false;
            }
            PressX=true;

        }else {
            PressX=false;
        }

        if(gamepad1.b){

            if(!PressO){
                robot.camera.openBackCam();
                baclCam=true;
            }
            PressO=true;

        }else{
            PressO=false;
        }

        telemetry.addLine("Front Camera: A");
        telemetry.addLine("Back Camera: B");
        telemetry.addLine("---------------");
        if(baclCam==true){
        //    telemetry.addLine("Prop position:"+robot.camera.getPositionProp());

        }
        telemetry.update();

    }

}
