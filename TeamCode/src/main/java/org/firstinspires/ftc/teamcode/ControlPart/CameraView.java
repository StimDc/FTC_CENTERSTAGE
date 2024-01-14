package org.firstinspires.ftc.teamcode.ControlPart;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;


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
            robot = new Robot(hardwareMap,telemetry);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

    }

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
        if(baclCam){
        //    telemetry.addLine("Prop position:"+robot.camera.getPositionProp());

        }
        telemetry.update();

    }

}
