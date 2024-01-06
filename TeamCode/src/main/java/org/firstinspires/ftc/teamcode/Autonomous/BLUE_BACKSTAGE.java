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

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Constants.PIDConstantsArm;
import org.firstinspires.ftc.teamcode.Implementations.Robot.CameraFunc;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="BLUE BACKSTAGE", group = "Red Routes")

public class BLUE_BACKSTAGE extends LinearOpMode {

    private int PARKING = -1; //-1 for left parking and 1 for right
    private WebcamName backCam, frontCam;
    private BluePropThreshold blueProp;
    private AprilTagProcessor atag;
    private VisionPortal vision;

    private PIDController controller;


    public static int target = 3;
    private final double ticks_in_degrees = 288 / (360.0 * 0.36); /// gear ratio: 45/125=0.36

    private DcMotorEx elevator1, elevator2;
    Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap,telemetry);


        //BasicMovements.init(hardwareMap);
        //BasicMovements.wheels.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        CameraFunc.openCamera(frontCam,"front",vision,atag,blueProp);

        String propPosition = blueProp.getPropPosition();

        boolean once = true;


        waitForStart();

        robot.claw.setPosition(Claw.INTERMEDIARY);

        sleep(1500);

        robot.joint.setPosition(Joint.DOWN);

        sleep(1500);

        robot.claw.setPosition(Claw.CLOSED);

        sleep(1500);

        //BasicMovements.lateral(-1, 0.6, 95);

        /*
        while ((propPosition.equals("nope") || once) && opModeIsActive() && !isStopRequested()){

            telemetry.addLine("Nope :( "+propPosition);
            propPosition=blueProp.getPropPosition();

            if(propPosition.equals("left")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=4;

                Backstage_LeftProp_Blue(PARKING,0);

            }else if(propPosition.equals("center")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=5;

                Backstage_CenterProp_Blue(PARKING,0);

            }else if(propPosition.equals("right")){

                telemetry.addLine(propPosition);
                telemetry.update();
                tagID=6;

                Backstage_RightProp_Blue(PARKING,0);

            }

            telemetry.update();

        }

         */
    }

    public void Backstage_LeftProp_Blue(int parking, int timer) {

        CameraFunc.openCamera(backCam,"back",vision,atag,blueProp);

        waitForStart();

        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        robot.joint.setPosition(Joint.DOWN);
        sleep(1000);

        robot.claw.setPosition(Claw.CLOSED);
        sleep(1000);

        robot.move.forward(1,0.7,64);
        sleep(1000);


        robot.move.rotate(-1, 0.5, 90);
        sleep(1000);

        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        robot.joint.setPosition(Joint.UP);
        sleep(1000);

        //BasicMovements.lateral(-1, 0.7, 61);
        sleep(1000);

        //BasicMovements.forward(1, 0.7, 45);


        /*
        Forward(1,0.5,7);
        sleep(500);

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);

        Forward(-1,0.4,3);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(100);

        joint.setPosition(Joint.UP);

        //ove_to_April(tagID);

        moveArm(200);
        sleep(500);

        claw.setPosition(Claw.OPEN);
        sleep(500);

        moveArm(0);

        if(parking=="LEFT"){

            lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            lateral(1,0,0);

        }


         */

        // lateral(parking,0.6,20);//DOAR PARCARE


    }

    public void Backstage_CenterProp_Blue(int parking, int timer) {

        CameraFunc.openCamera(backCam,"back",vision,atag,blueProp);


        waitForStart();

        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);
        robot.joint.setPosition(Joint.DOWN);
        sleep(1000);

        robot.claw.setPosition(Claw.CLOSED);
        sleep(1000);


        sleep(1000);
        //BasicMovements.forward(1, 0.7, 66);
        sleep(1000);


        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        robot.joint.setPosition(Joint.UP);

        // Forward(-1,0.5,4);
        //sleep(1000);

        // claw.setPosition(clawPos.CLOSED);
        // sleep(1000);

        //BasicMovements.forward(-1, 0.7, 65);
        sleep(1000);

        //BasicMovements.lateral(-1, 0.5, 45);
        sleep(1000);

        // claw.setPosition(Claw.INTERMEDIARY);
        // sleep(1000);

        //joint.setPosition(Joint.UP);

/*
        Rotate(1,0,90);
        sleep(500);

        Move_To_April(tagID);
        sleep(500);

        moveArm(200);

        sleep(500);

        setPosition(clawPos.OPEN);

        moveArm(0);

        sleep(500);


        if(parking=="LEFT"){

            lateral(-1,0,0);

        }else if(parking=="RIGHT"){

            lateral(1,0,0);

        }

       */

        //BasicMovements.lateral(parking, 0.5, 20);//PARCAM SI ATAT


    }

    public void Backstage_RightProp_Blue(int parking, double timer) {

        CameraFunc.openCamera(backCam,"back",vision,atag,blueProp);

        waitForStart();


        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        robot.joint.setPosition(Joint.DOWN);
        sleep(1000);

        robot.claw.setPosition(Claw.CLOSED);
        sleep(1000);

        //BasicMovements.forward(1, 0.7, 64);
        sleep(1000);


        robot.move.rotate(1, 0.5, 90);
        sleep(1000);

        robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(1000);

        robot.joint.setPosition(Joint.UP);
        sleep(1000);

        //BasicMovements.lateral(1, 0.7, 61);
        sleep(1000);

        //BasicMovements.forward(-1, 0.7, 45);

        /*
        Open_BackCam();

        waitForStart();

        joint.setPosition(Joint.UP);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(500);

        Forward(1,0.7,61);
        sleep(500);

        Rotate(1,0.5,90);
        sleep(500);

        joint.setPosition(Joint.DOWN);

        Forward(-1,0.6,25);
        sleep(500);

        claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);

        Forward(-1,0.4,3.5);
        sleep(500);

        claw.setPosition(Claw.CLOSED);
        sleep(100);

        joint.setPosition(Joint.UP);

         */

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

        //BasicMovements.lateral(parking, 0.6, 30);


    }


    public void moveArm(int desiredTarget) {//TODO: RESCRIS

        if (elevator1.getCurrentPosition() < desiredTarget) {

            for (target = elevator1.getCurrentPosition(); target <= desiredTarget; target++) {
                controller.setPID(PIDConstantsArm.p, PIDConstantsArm.i, PIDConstantsArm.d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(250);
            }

        } else if (elevator1.getCurrentPosition() > desiredTarget) {

            for (target = elevator1.getCurrentPosition(); target >= 0; target--) {
                controller.setPID(PIDConstantsArm.p, PIDConstantsArm.i, PIDConstantsArm.d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / ticks_in_degrees));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(250);
            }
        }
    }

}