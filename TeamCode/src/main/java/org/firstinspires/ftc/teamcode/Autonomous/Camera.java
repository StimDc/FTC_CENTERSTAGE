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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Camera", group = "Robot")

public class Camera extends LinearOpMode {
    private AprilTagProcessor apriltagProcesor;

    private VisionPortal myVisionPortal;

    private int apriltagid;

    @Override
    public void runOpMode() {

        apriltagProcesor = AprilTagProcessor.easyCreateWithDefaults();
        myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Camera1"), apriltagProcesor);


        waitForStart();

        while (opModeIsActive()) {

            double aprilX1, aprilY1, aprilZ1;
            double aprilX2, aprilY2, aprilZ2;
            double aprilX3, aprilY3, aprilZ3;
            Vector3 april1 = new Vector3();
            Vector3 april2 = new Vector3();
            Vector3 april3 = new Vector3();

            List<AprilTagDetection> AprilDetections;

            AprilDetections = apriltagProcesor.getDetections();

            for(AprilTagDetection detection : AprilDetections)
            telemetry.addData("April Tag-uri: ",AprilDetections.size());


            for (AprilTagDetection currentDetection : AprilDetections) {

                if (currentDetection.metadata != null) {
                    apriltagid = currentDetection.id;

                    switch(apriltagid){
                        case 1:
                            april1.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);

                            break;
                        case 2:
                            april2.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);

                            break;
                        case 3:
                            april3.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);
                                                        break;
                    }
                    telemetry.addLine("AprilTag1: X " + String.valueOf(april1.x) + " Y " + String.valueOf(april1.y) + " Z " + String.valueOf(april1.z));
                    telemetry.addLine("AprilTag2: X " + String.valueOf(april2.x) + " Y " + String.valueOf(april2.y) + " Z " + String.valueOf(april2.z));
                    telemetry.addLine("AprilTag3: X " + String.valueOf(april3.x) + " Y " + String.valueOf(april3.y) + " Z " + String.valueOf(april3.z));

                    //telemetry.addLine(String.format("XYZ1: %6.1f %6.1f %6.1f",currentDetection.ftcPose.x, currentDetection.ftcPose.y,currentDetection.ftcPose.z));
                                                                                /*

                    if(apriltagid==1){
                        april1.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);

                        telemetry.addLine(String.format("XYZ1: %6.1f %6.1f %6.1f",currentDetection.ftcPose.x, currentDetection.ftcPose.y,currentDetection.ftcPose.z));
                    }
                    else if(apriltagid==2){
                        april2.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);


                        telemetry.addLine(String.format("XYZ2: %6.1f %6.1f %6.1f",currentDetection.ftcPose.x,currentDetection.ftcPose.y,currentDetection.ftcPose.z));

                    }
                    else if(apriltagid==3){
                        april3.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);



                        telemetry.addLine(String.format("XYZ3: %6.1f %6.1f %6.1f",currentDetection.ftcPose.x,currentDetection.ftcPose.y,currentDetection.ftcPose.z));

                    }

                     */

                    telemetry.update();
                    sleep(20);

                }
            }


        }

    }

    public double Inch_to_Cm (int inch){

        double cm=2.54*inch;

        return cm;
    }

}
