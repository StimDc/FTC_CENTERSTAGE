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

import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;

import java.io.IOException;
import java.util.Dictionary;

@Autonomous(name="AUTONOMIE", group = "Robot")
public class Autonomie_Primitive extends LinearOpMode {
    Dictionary<String,String> paramaters;
    private int tagID;

    @Override
    public void runOpMode() {

        Robot robot;
        try {
            paramaters = Choice_Menu.readFromFile();
            robot = new Robot(hardwareMap,telemetry);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot.camera.openFrontCam();
        double target = robot.arm.ZERO_OFFSET;
        //String propPosition= robot.camera.GetPropPosition();
        String propPosition="nope";
        boolean once = true;


        // RED=1 && BLUE=2
        String alliance = paramaters.get("alliance"); //RED or BLUE
        // FRONTSTAGE=1 && BACKSTAGE=2
        String startpoint = paramaters.get("position");//FRONT_STAGE or BACK_STAGE
        // LEFT=1 && RIGHT=2
        String parking = paramaters.get("parking");//LEFT or RIGHT

        String timer=paramaters.get("timer");// HOW TO CONVERT FROM STRING TO INT???
        try{
            //how much to wait in seconds
            int wait = Integer.parseInt(timer);
        }
        catch (NumberFormatException ex){
            ex.printStackTrace();
        }

        robot.wheels.setDirection();
        int[] tags = (alliance.equals("RED")) ? new int[]{4, 5, 6} : new int[]{1, 2, 3};

        RedBackStageAprilFast redBack = null;
        BlueBackStageAprilFast blueOpMode = null;
        RedFrontStage redFront = null;
        if(alliance.equals("RED") && startpoint.equals("BACK_STAGE")){
            redBack = new RedBackStageAprilFast(robot,telemetry,tagID);
            redBack.init();

        }
        else if(alliance.equals("BLUE") && startpoint.equals("BACK_STAGE")){
            blueOpMode = new BlueBackStageAprilFast(robot,telemetry,tagID);
            blueOpMode.init();
        }
        else if(alliance.equals("RED") && startpoint.equals("FRONT_STAGE")){
            redFront = new RedFrontStage(robot,telemetry,tagID);
            redFront.init();
        }



        waitForStart();
        while (propPosition.equals("nope") && opModeIsActive() && !isStopRequested() ){

            telemetry.addLine("Nope :( "+propPosition);
            propPosition=robot.camera.GetPropPosition();
            telemetry.addLine(propPosition);
            telemetry.update();
            switch (propPosition) {
                case "left":
                    if(alliance.equals("RED"))
                        tagID = 4;
                    else
                        tagID = 1;
                    once = false;

                    //Backstage_LeftProp_Red(PARKING, 0);

                    break;
                case "center":
                    if(alliance.equals("RED"))
                        tagID = 5;
                    else
                        tagID = 2;
                    once = false;

                    //Backstage_CenterProp_Red(PARKING, 0);

                    break;
                case "right":
                    if(alliance.equals("RED"))
                        tagID = 6;
                    else
                        tagID = 3;
                    once = false;
                    //Backstage_RightProp_Red(PARKING, 0);
                    break;
            }
            telemetry.addLine(alliance);

            telemetry.addLine(propPosition);
            telemetry.addData("tag", tagID);
            telemetry.addData("pix", RedPropThreshold_Backstage.RedPixels);
            telemetry.update();

            //robot.clearBulkCache();

        }



        if(startpoint.equals("BACK_STAGE")) {
            if (alliance.equals("RED") && propPosition.equals("center")) {
                redBack.passTag(tagID);
                redBack.backStageCenterProp(-1, 0);
            } else if (alliance.equals("RED") && propPosition.equals("left")) {
                redBack.passTag(tagID);
                redBack.backStageLeftProp(-1, 0);
            } else if (alliance.equals("RED") && propPosition.equals("right")) {
                redBack.passTag(tagID);
                redBack.backStageRightProp(-1, 0);
            } else if (alliance.equals("BLUE") && propPosition.equals("left")) {
                blueOpMode.passTag(tagID);
                blueOpMode.backStageLeftProp(-1, 0);
            } else if (alliance.equals("BLUE") && propPosition.equals("center")) {
                blueOpMode.passTag(tagID);
                blueOpMode.backStageCenterProp(-1, 0);
            } else if (alliance.equals("BLUE") && propPosition.equals("right")) {
                blueOpMode.passTag(tagID);
                blueOpMode.backStageRightProp(-1, 0);
            }
        }
        else if(startpoint.equals("FRONT_STAGE")){
            if(alliance.equals("RED") && propPosition.equals("left")){
                redFront.frontStageLeftProp(-1,0);
            }
        }
        else if(startpoint.equals("FRONT_STAGE"));

    }
}
