package org.firstinspires.ftc.teamcode.ControlPart;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;


import java.io.IOException;
import java.util.concurrent.TimeUnit;

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


        robot.camera.openBackCam();
        setManualExposure(1,1);

    }

    @Override
    public void loop() {

        setManualExposure(1,1);
        telemetry.addLine("Exposure: "+robot.camera.isExposure());
      //  telemetry.addLine("Max Gain: "+robot.camera.getMaxGain());
      //  telemetry.addLine("Min Gain: "+robot.camera.getMinGain());

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


    private boolean   setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (robot.camera.vision == null) {
            return false;
        }

        // Wait for the camera to be open
        if (robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while ((robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!false)
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = robot.camera.vision.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(7, TimeUnit.MILLISECONDS);
            sleep(20);

            telemetry.addLine("Exposure Ceva: "+exposureControl);

            // Set Gain.
            GainControl gainControl = robot.camera.vision.getCameraControl(GainControl.class);

            telemetry.addLine("Gain Ceva: "+gainControl);

            if(gainControl!=null){
                boolean haide= gainControl.setGain(255);

            }
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }




}
