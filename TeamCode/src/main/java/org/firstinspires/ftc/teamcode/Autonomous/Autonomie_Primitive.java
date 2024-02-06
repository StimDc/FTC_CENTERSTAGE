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

import org.firstinspires.ftc.teamcode.Implementations.ChoiceMenu.ChoiceMenu;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;

import java.io.IOException;
import java.util.Dictionary;

@Photon
@Autonomous(name="AUTONOMIE PEIMITIVE", group = "Robot")
public class Autonomie_Primitive extends LinearOpMode {
    Dictionary<String,String> paramaters;


    @Override
    public void runOpMode() {

        Robot robot;
        try {
            paramaters = ChoiceMenu.readFromFile();
            robot = new Robot(hardwareMap,telemetry);

        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot.camera.openFrontCam();
        double target = robot.arm.ZERO_OFFSET;
        String propPosition= robot.camera.GetPropPosition();

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


        telemetry.addLine(alliance +" "+ startpoint +" "+ parking);
        telemetry.update();

        waitForStart();

        int[] tags = (alliance.equals("RED")) ? new int[]{4, 5, 6} : new int[]{1, 2, 3};

        //TODO: de facut apelurile functiilor corespunzator pentru programe
        while((propPosition.equals("nope") || once)&& opModeIsActive() && !isStopRequested()){
            telemetry.addLine("Nope :( " + propPosition);
            propPosition = robot.camera.GetPropPosition();

            switch(propPosition){
                case "left":
                    telemetry.addLine(propPosition);
                    int tagID = tags[0];
                    once = false;

                    if(alliance.equals("RED") && startpoint.equals("FRONT_STAGE")){

                    }
                    else if(alliance.equals("RED") && startpoint.equals("BACK_STAGE")){

                    }
                    else if(alliance.equals("BLUE") && startpoint.equals("FRONT_STAGE")){

                    }
                    else if(alliance.equals("BLUE") && startpoint.equals("BACK_STAGE")){

                    }
                    else{

                    }
                    break;

                case "center":
                    telemetry.addLine(propPosition);
                    tagID = tags[1];
                    once = false;
                    if(alliance.equals("RED") && startpoint.equals("FRONT_STAGE")){

                    }
                    else if(alliance.equals("RED") && startpoint.equals("BACK_STAGE")){

                    }
                    else if(alliance.equals("BLUE") && startpoint.equals("FRONT_STAGE")){

                    }
                    else if(alliance.equals("BLUE") && startpoint.equals("BACK_STAGE")){

                    }
                    else{

                    }
                    break;

                case "right":
                    telemetry.addLine(propPosition);
                    tagID = tags[2];
                    once = false;
                    if(alliance.equals("RED") && startpoint.equals("FRONT_STAGE")){

                    }
                    else if(alliance.equals("RED") && startpoint.equals("BACK_STAGE")){

                    }
                    else if(alliance.equals("BLUE") && startpoint.equals("FRONT_STAGE")){

                    }
                    else if(alliance.equals("BLUE") && startpoint.equals("BACK_STAGE")){

                    }
                    else{

                    }
                    break;
            }
            telemetry.update();
            robot.controlHub.clearBulkCache();
            robot.expansionHub.clearBulkCache();
        }

    }
}
