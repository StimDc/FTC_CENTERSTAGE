package org.firstinspires.ftc.teamcode.Implementations.Camera;

import org.firstinspires.ftc.vision.VisionProcessor;

public class Choising_Color{


    public BluePropThreshold_Backstage bluePropBackstage;
    public BluePropThreshold_Frontstage bluePropFrontstage;
    public RedPropThreshold_Backstage redPropThresholdBackstage;
    public RedPropThreshold_Frontstage redPropThresholdFrontstage;

    private int startPosition;

    public void getStartPosition(int startPosition){

        this.startPosition=startPosition;


    }

    public VisionProcessor getColor() {


        if (startPosition == -2) {

            bluePropBackstage = new BluePropThreshold_Backstage();
            return bluePropBackstage;

        } else if (startPosition == -1) {

            redPropThresholdBackstage = new RedPropThreshold_Backstage();
            return redPropThresholdBackstage;


        } else if (startPosition == 1) {

            redPropThresholdFrontstage = new RedPropThreshold_Frontstage();
            return redPropThresholdFrontstage;

        } else if (startPosition == 2) {

            bluePropFrontstage = new BluePropThreshold_Frontstage();
            return bluePropFrontstage;
        }else{
            return null;
        }
    }

    public String getProp(){

        if (startPosition == -2) {

            return bluePropBackstage.getPropPosition();

        } else if (startPosition == -1) {

            return redPropThresholdBackstage.getPropPosition();


        } else if (startPosition == 1) {

            return redPropThresholdFrontstage.getPropPosition();

        } else if (startPosition == 2) {

            return bluePropFrontstage.getPropPosition();
        }else{
            return "nope";
        }

    }

}
