/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Primitive_RoutesRed {
    private RedPropThreshold redProp;
    private BluePropThreshold blueProp;

    private AprilTagProcessor apriltagProcesor;
    private int apriltagid,idTarget;

    FtcDashboard dashboard;
    public static int target=3;

    public double val=0;

    private Robot robot;
    private HardwareMap hardwareMap;
    public void runOpMode() throws InterruptedException {

        FUCKINGCAMERAFRONT_Red();


        RED_BACKSTAGE("LEFT",0);

    }

   public void RED_BACKSTAGE(String parking, int waittimer) throws InterruptedException {




       if (redProp.getPropPosition().equals("left")) {

           idTarget = 4;
           //  Backstage_LeftProp_Red(parking,waittimer,idTarget);
       }else if(redProp.getPropPosition().equals("center")) {

           idTarget = 5;
           Backstage_CenterProp_Red(parking, waittimer, idTarget);


       }else if(redProp.getPropPosition().equals("right")){
           idTarget = 6;
           //  Backstage_RightProp_Red(parking,waittimer,idTarget);


       }

        while(redProp.getPropPosition().equals("nope")) {
            if (Red_Prop_Pos().equals("left")) {

                idTarget = 4;
                //  Backstage_LeftProp_Red(parking,waittimer,idTarget);
            }else if(redProp.getPropPosition().equals("center")) {

                idTarget = 5;
                Backstage_CenterProp_Red(parking, waittimer, idTarget);


            }else if(redProp.getPropPosition().equals("right")){
                    idTarget = 6;
                  //  Backstage_RightProp_Red(parking,waittimer,idTarget);


            }
        }

    }


/*
    public void RED_FRONTSTAGE(String parking, int waittimer){

        InitCamera();
        CamFront_Open_Red();

        while(Red_Prop_Pos()=="nope") {
            switch (Red_Prop_Pos()) {

                case "left":
                    idTarget = 4;
                    Frontstage_LeftProp_Red(parking,waittimer,idTarget);
                    break;

                case "center":
                    idTarget = 5;
                    Frontstage_CenterProp_Red(parking,waittimer,idTarget);
                    break;

                case "right":
                    idTarget = 6;
                    Frontstage_RightProp_Red(parking,waittimer,idTarget);
                    break;

            }
        }

    }

 */

    public void Backstage_CenterProp_Red(String parking, int waittimer,int atag) throws InterruptedException {

        dashboard=FtcDashboard.getInstance();



        robot.claw.setPosition(Claw.CLOSED);
        robot.joint.setPosition(Joint.UP);

        robot.joint.setPosition(Joint.DOWN);

        sleep(250);
        robot.move.forward(1,0.7,69);
        sleep(500);


        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);

        robot.move.forward(-1,15,0);
        sleep(500);

        robot.claw.setPosition(Claw.CLOSED);
        sleep(500);

        robot.joint.setPosition(Joint.UP);
        sleep(500);
/*
        move.Rotate(-1,0,90);
        sleep(500);

        move.Move_To_April(atag);

        move.moveArm(200);

        claw.setPosition(clawPos.OPEN);

        move.moveArm(0);



        if(parking=="LEFT"){

            move.lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            move.lateral(1,0,0);

        }

       */

    }
/*
    public void Backstage_LeftProp_Red(String parking, int waittimer,int atag){
        move.InitMotors();
        move.InitIMU();
        move.initArm();

        jointPos=new Joint();
        joint=hardwareMap.get(Servo.class,"joint");

        clawPos=new Claw();
        claw= hardwareMap.get(Servo.class,"claw");

        dashboard=FtcDashboard.getInstance();

        move.CamBack_Open();

        waitForStart();

        joint.setPosition(Joint.UP);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(500);

        move.Forward(1,0,0);
        sleep(500);

        move.Rotate(-1,0,90);
        sleep(500);

        joint.setPosition(Joint.DOWN);
        sleep(250);

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(250);

        move.Forward(-1,0,0);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(500);

        joint.setPosition(Joint.UP);
        sleep(500);

        move.Move_To_April(atag);

        move.moveArm(200);

        claw.setPosition(Claw.OPEN);

        move.moveArm(0);

        if(parking=="LEFT"){

            move.lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            move.lateral(1,0,0);

        }

    }

    public void Backstage_RightProp_Red(String parking, int waittimer,int atag){
        move.InitMotors();
        move.InitIMU();
        move.initArm();

        jointPos=new Joint();
        joint=hardwareMap.get(Servo.class,"joint");

        clawPos=new Claw();
        claw= hardwareMap.get(Servo.class,"claw");

        dashboard=FtcDashboard.getInstance();

        move.CamBack_Open();

        waitForStart();

        joint.setPosition(Joint.UP);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(500);

        move.Forward(1,0,0);
        sleep(500);

        move.Rotate(-1,0,90);
        sleep(500);

        move.Forward(-1,0,0);
        sleep(250);

        joint.setPosition(Joint.DOWN);
        sleep(250);

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(250);

        move.Forward(-1,0,0);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(500);

        joint.setPosition(Joint.UP);
        sleep(500);

        move.Move_To_April(atag);

        move.moveArm(200);

        claw.setPosition(Claw.OPEN);

        move.moveArm(0);

        if(parking=="LEFT"){

            move.lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            move.lateral(1,0,0);

        }


    }

    public void Frontstage_CenterProp_Red(String parking, int waittimer,int atag){
        move.InitMotors();
        move.InitIMU();
        joint=hardwareMap.get(Servo.class,"joint");
        claw = hardwareMap.get(Servo.class,"claw");
        dashboard = FtcDashboard.getInstance();
        move.CamBack_Open();

        waitForStart();

        joint.setPosition(Joint.UP);
        claw.setPosition(Claw.CLOSED);
        move.Forward(1,0,0);
        joint.setPosition(Joint.DOWN);
        claw.setPosition(Claw.OPEN);
        move.Forward(-1,0,0);
        claw.setPosition(Claw.CLOSED);
        joint.setPosition(Joint.UP);
        move.Rotate(-1,0,90);
        move.lateral(-1,0,0);
        move.Forward(-1,0,0);

        //Camera tag idk ce a vrut sa scrie domnul Rares doamne dami rabdare ca daca imi dai putere ma arunc de pe geam
    }



    public void Frontstage_LeftProp_Red(String parking, int waittimer,int atag){
        move.InitMotors();
        move.InitIMU();
        joint = hardwareMap.get(Servo.class,"joint");
        claw=hardwareMap.get(Servo.class,"claw");
        dashboard = FtcDashboard.getInstance();
        move.CamBack_Open();

        waitForStart();

        joint.setPosition(Joint.UP);
        claw.setPosition(Claw.CLOSED);
        move.Forward(1,0,0);
        move.Rotate(-1,0,90);
        move.Forward(-1,0,0);
        joint.setPosition(Joint.DOWN);
        claw.setPosition(Claw.OPEN);
        move.Forward(-1,0,0);
        claw.setPosition(Claw.CLOSED);
        joint.setPosition(Joint.UP);
        move.Forward(1,0,0);
        move.lateral(-1,0,0);
        move.Forward(-1,0,0);
        move.moveRobot(0,0,move.Correcting_Yaw());
    }

    public void Frontstage_RightProp_Red(String parking, int waittimer,int atag){

        move.InitMotors();
        move.InitIMU();
        joint=hardwareMap.get(Servo.class,"joint");
        claw = hardwareMap.get(Servo.class,"claw");
        dashboard = FtcDashboard.getInstance();
        move.CamBack_Open();

        waitForStart();

        joint.setPosition(Joint.UP);
        claw.setPosition(Claw.CLOSED);
        move.Forward(1,0,0);
        move.Rotate(1,0,90);
        move.Forward(-1,0,0);
        joint.setPosition(Joint.DOWN);
        claw.setPosition(Claw.OPEN);
        move.Forward(-1,0,0);
        claw.setPosition(Claw.CLOSED);
        joint.setPosition(Joint.UP);
        move.Rotate(1,0,180);
        move.lateral(-1,0,0);
        move.Forward(-1,0,0);
        move.moveRobot(0,0,move.Correcting_Yaw());
    }


 */

    public void FUCKINGCAMERAFRONT_Red(){

        redProp=new RedPropThreshold();

       /* vision = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"Camera2"))
                .addProcessor(apriltagProcesor)
                .addProcessor(redProp)
                //.addProcessor(blueProp)
                .setCameraResolution(new Size(640,480))
                //.addProcessor(blueProp)
                .build();

        //while(vision.getCameraState() != VisionPortal.CameraState.STREAMING){

        //}
*/
    }



    public String Red_Prop_Pos(){

        return redProp.getPropPosition();

    }
    public double Correcting_Yaw(){

        AprilTagDetection desiredTag = null;
        boolean targetFound = false;;

        while(!targetFound){

            List<AprilTagDetection> currentDetections = apriltagProcesor.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if ((detection.metadata != null && (detection.id==5 || detection.id==2))){
                    targetFound = true;
                    desiredTag = detection;
                    break;  // don't look any further.
                }
            }

        }

        return Range.clip(desiredTag.ftcPose.bearing* 1, -0.7, 0.7) ;
    }

}
//1283 linii 71 warnings 473 linii cu 19 warning