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


        if (this.startPosition == -2) {

            this.bluePropBackstage = new BluePropThreshold_Backstage();
            return this.bluePropBackstage;

        } else if (this.startPosition == -1) {

            this.redPropThresholdBackstage = new RedPropThreshold_Backstage();
            return this.redPropThresholdBackstage;


        } else if (this.startPosition == 1) {

            this.redPropThresholdFrontstage = new RedPropThreshold_Frontstage();
            return this.redPropThresholdFrontstage;

        } else if (this.startPosition == 2) {

            this.bluePropFrontstage = new BluePropThreshold_Frontstage();
            return this.bluePropFrontstage;
        }else{
            return null;
        }
    }

    public String getProp(){

        if (this.startPosition == -2) {

            return this.bluePropBackstage.getPropPosition();

        } else if (this.startPosition == -1) {

            return this.redPropThresholdBackstage.getPropPosition();


        } else if (this.startPosition == 1) {

            return this.redPropThresholdFrontstage.getPropPosition();

        } else if (this.startPosition == 2) {

            return this.bluePropFrontstage.getPropPosition();
        }else{
            return "nope";
        }

    }

}
