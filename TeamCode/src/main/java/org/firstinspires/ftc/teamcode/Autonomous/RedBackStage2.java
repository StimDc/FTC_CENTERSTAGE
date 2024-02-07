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

import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.BACKWARDS;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.RIGHT;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;

import java.io.IOException;

@Autonomous(name="RED BACKSTAGE DOI", group = "Red Routes")

public class RedBackStage2 extends  LinearOpMode{

    private int PARKING=1; //-1 for left parking and 1 for right

    public static double target;
    private  Robot robot;
    private int tagID;
    private   final double ZERO_OFFSET = 70.0-3.85;
    private   double TargetPosInDegrees=70.0-3.85;
    @Override
    public void runOpMode () {
        try {
            robot = new Robot(hardwareMap,telemetry);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        robot.camera.openFrontCam();
        target=robot.arm.ZERO_OFFSET;

        String propPosition="nope";


        boolean once=true;

        waitForStart();

        while ((propPosition.equals("nope") || once) && opModeIsActive() && !isStopRequested()){
            telemetry.addLine("Nope :( "+propPosition);
            switch (propPosition) {
                case "left":
                    tagID = 4;
                    once = false;
                    Backstage_LeftProp_Red(PARKING, 0);
                    break;
                case "center":
                    tagID = 5;
                    once = false;
                    Backstage_CenterProp_Red(PARKING, 0);
                    break;
                case "right":
                    tagID = 6;
                    once = false;
                    Backstage_RightProp_Red(PARKING, 0);
                    break;
            }
            telemetry.update();
        }
    }

    public void Backstage_LeftProp_Red(int parking,int timer){

        robot.camera.openBackCam();


        robot.camera.openBackCam();


        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);
        robot.joint.setPosition(Joint.DOWN);
        sleep(1000);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(1000);
        robot.joint.setPosition(Joint.UP);

        robot.move.forward(FORWARD,0.6,50);
        sleep(250);

        robot.move.rotate(-1,0.6,90);

        robot.joint.setPosition(Joint.DOWN);
        sleep(1000);
        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);
        robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(250);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(1000);
        robot.joint.setPosition(Joint.UP);
        sleep(1000);

        robot.move.Move_to_AprilAllAxes(tagID,robot,robot.camera.atag);

        int stateArm=0;

        boolean armtarget=false,OKtarget=false;

        while(!armtarget){

            switch (stateArm){

                case 0:

                    robot.arm.setPosition(ZERO_OFFSET,1);
                    stateArm=1;
                    break;

                case 1:
                    robot.arm.setPosition(381,1);
                    stateArm=2;
                    break;

                case 2:
                    if(robot.arm.isOnTarget(1)) {
                        stateArm=3;
                    }
                    break;

                case 3:

                    robot.claw.setPosition(Claw.OPEN);
                    stateArm=4;
                    break;

                case 4:


                    if(Math.abs(robot.claw.getPosition()-Claw.OPEN) <0.03){

                        stateArm=5;
                    }
                    break;


                case 5:
                    robot.arm.setPosition(ZERO_OFFSET,1);
                    stateArm=6;
                    break;

                case 6:
                    if(robot.arm.isOnTarget(1)) {
                        if(Math.abs(robot.arm.targetPosInDegrees-ZERO_OFFSET)<3){

                            OKtarget=true;

                        }
                        stateArm=7;
                    }
                    break;

                case 7:

                    armtarget=true;
                    telemetry.addLine("DONE :D");
            }

            if(Math.abs(robot.arm.targetPosInDegrees-ZERO_OFFSET)<3 && OKtarget){

                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(0);

            }

            if(Math.abs(robot.arm.targetPosInDegrees-robot.arm.ZERO_OFFSET)>=3){
                OKtarget=false;

            }

            if(!OKtarget){
                robot.arm.armTask();

            }
            // telemetry.addLine("Ceva: "+ceva);
            telemetry.addLine("Pos: "+robot.arm.elevator1.getCurrentPosition());
            telemetry.addLine("Target: "+target);
            telemetry.addLine("State: "+stateArm);
            telemetry.update();


        }
        sleep(1000);

        robot.move.lateral(RIGHT,0.6,27);



    }

    public void Backstage_CenterProp_Red(int parking,int timer){

        robot.camera.openBackCam();

        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(750);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);



        robot.move.lateral(RIGHT,0.4,8);
        sleep(175);

        robot.move.forward(FORWARD,0.6,57);
        sleep(175);
        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);
        robot.move.forward(BACKWARDS,0.5,5.5);
        sleep(175);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        robot.joint.setPosition(Joint.UP);

        robot.move.rotate(-1,0.6,90);
        sleep(250);


        int stateArm=0;

        boolean armtarget=false,OKtarget=false;

        while(!armtarget && opModeIsActive() && !isStopRequested()){

            switch (stateArm){

                case 0:

                    robot.arm.setPosition(ZERO_OFFSET,1);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=1;
                    break;

                case 1:
                    robot.arm.setPosition(240,6);
                    TargetPosInDegrees=230;
                    stateArm=2;
                    break;

                case 2:
                    if(robot.arm.isOnTarget(6) && robot.move.getArmState()) {
                        stateArm=4;
                    }
                    break;

                case 3:

                    if(robot.move.getArmState()){

                        stateArm=4;
                    }

                    break;

                case 4:

                    robot.claw.setPosition(Claw.OPEN);
                    stateArm=4;
                    break;

                case 5:


                    if(Math.abs(robot.claw.getPosition()-Claw.OPEN) <0.03){

                        stateArm=5;
                    }
                    break;


                case 6:
                    robot.arm.setPosition(ZERO_OFFSET,1);
                    TargetPosInDegrees=ZERO_OFFSET;
                    stateArm=6;
                    break;

                case 7:
                    if(robot.arm.isOnTarget(5)) {
                        if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)<5){

                            OKtarget=true;

                        }
                        stateArm=7;
                    }
                    break;

                case 8:

                    armtarget=true;
                    telemetry.addLine("DONE :D");
                    // telemetry.update();

            }

            robot.move.Move_to_AprilAllAxes_StateMachine(tagID,robot,robot.camera.atag);


            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)<5 && OKtarget==true){

                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                robot.arm.setPower(0);

            }

            if(Math.abs(TargetPosInDegrees-ZERO_OFFSET)>=3){
                OKtarget=false;

            }

            if(!OKtarget){
                robot.arm.armTask();

            }
            telemetry.addLine("Pos: "+robot.arm.getPosition());
            telemetry.addLine("Target: "+TargetPosInDegrees);
            telemetry.addLine("State: "+stateArm);
            telemetry.update();


        }

        robot.move.lateral(RIGHT,0.6,58);

        sleep(250);

        robot.move.forward(BACKWARDS,0.6,25);

    }
    public void Backstage_RightProp_Red(int parking, double timer){

        robot.camera.openBackCam();


        robot.joint.setPosition(Joint.DOWN);
        sleep(500);

        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);

        robot.move.forward(1,0.7,61);
        sleep(500);

        robot.move.rotate(-1,0.5,90);
        sleep(500);

        robot.joint.setPosition(Joint.DOWN);

        robot.move.forward(-1,0.6,25);
        sleep(500);

        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);

        robot.move.forward(-1,0.4,3.5);
        sleep(500);

        robot.claw.setPosition(Claw.CLOSED);
        sleep(100);

        robot.joint.setPosition(Joint.UP);

        /*
        Move_to_April(tagID);

        moveArm(200);
        sleep(500);

        claw.setPosition(Claw.OPEN);
        sleep(500);

        moveArm(0);
        sleep(500);

        if(parking=="LEFT"){

            lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            lateral(1,0,0);

        }

         */

        robot.move.Move_to_AprilAllAxes(tagID,robot,robot.camera.atag);


        int stateArm=0;

        boolean armtarget=false,OKtarget=false;

        while(!armtarget){

            switch (stateArm){

                case 0:

                    robot.arm.setPosition(robot.arm.ZERO_OFFSET,1);
                    stateArm=1;
                    break;

                case 1:
                    robot.arm.setPosition(381,1);
                    stateArm=2;
                    break;

                case 2:
                    if(robot.arm.isOnTarget(1)) {
                        stateArm=3;
                    }
                    break;

                case 3:

                    robot.claw.setPosition(Claw.OPEN);
                    stateArm=4;
                    break;

                case 4:


                    if(Math.abs(robot.claw.getPosition()-Claw.OPEN) <0.03){

                        stateArm=5;
                    }
                    break;


                case 5:
                    robot.arm.setPosition(robot.arm.ZERO_OFFSET,1);
                    stateArm=6;
                    break;

                case 6:
                    if(robot.arm.isOnTarget(1)) {
                        if(Math.abs(robot.arm.targetPosInDegrees-robot.arm.ZERO_OFFSET)<3){

                            OKtarget=true;

                        }
                        stateArm=7;
                    }
                    break;

                case 7:

                    armtarget=true;
                    telemetry.addLine("DONE :D");
                    // telemetry.update();

            }

            if(Math.abs(robot.arm.targetPosInDegrees-robot.arm.ZERO_OFFSET)<3 && OKtarget){
                robot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.arm.setPower(0);

            }

            if(Math.abs(robot.arm.targetPosInDegrees-robot.arm.ZERO_OFFSET)>=3){
                OKtarget=false;

            }

            if(!OKtarget){
                robot.arm.armTask();

            }
            // telemetry.addLine("Ceva: "+ceva);
            telemetry.addLine("Pos: "+robot.arm.elevator1.getCurrentPosition());
            telemetry.addLine("Target: "+target);
            telemetry.addLine("State: "+stateArm);
            telemetry.update();


        }
        sleep(1000);

        robot.move.lateral(RIGHT,0.6,30);
    }
}
