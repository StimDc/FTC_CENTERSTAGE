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

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Math.Vector3;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name="AprilTag Move", group = "Robot")

public class AprilTag_Move extends LinearOpMode {

    private AprilTagProcessor apriltagProcesor;
    private VisionPortal myVisionPortal;
    private int apriltagid;
    private double distx=999, disty=999;
    private final double POWER=0.3;

    @Override
    public void runOpMode() {

        //BasicMovements.init(hardwareMap);

        /// apriltagProcesor = AprilTagProcessor.easyCreateWithDefaults();
        /// myVisionPortal = VisionPortal.easyCreateWithDefaults(hardwareMap.get(WebcamName.class, "Camera1"), apriltagProcesor);


        apriltagProcesor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
               // .setLensIntrinsics()
                .build();

        myVisionPortal = new VisionPortal.Builder()
                .addProcessor(apriltagProcesor)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera1"))
                .setCameraResolution(new Size(640,480))
                ///.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();


        while(myVisionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){

        }


        /* CA SA REDUCEM BLURAREA APRIL TAG-ULUI, ATUNCI CAND SE MISCA ROBOTUL

        ExposureControl exposure =myVisionPortal.getCameraControl(ExposureControl.class);
        exposure.setMode(ExposureControl.Mode.Manual);
        exposure.setExposure(15, TimeUnit.MILLISECONDS);
        /// telemetry.addData("Exposure: ",exposure.isExposureSupported());


        GainControl gain=myVisionPortal.getCameraControl(GainControl.class);
        gain.setGain(255);
        ///telemetry.addData("Min gain: ", gain.getMinGain());
        ///telemetry.addData("Max gain: ",gain.getMaxGain());


         */
        waitForStart();

        while(distx == 999 && disty==999) {
            AprilTagg_Detection();
        }

        while (!isStopRequested() && opModeIsActive()) {


            //BasicMovements.lateral(POWER, MathFunc.inchToTicks(distx *(1/ Math.sin(45))));
            distx=0;

            sleep(2500);


            disty=9.5;

            sleep(2500);

        }

    }

    public void AprilTagg_Detection(){

        Vector3 april1 = new Vector3();
        Vector3 april2 = new Vector3();
        Vector3 april3 = new Vector3();


        List<AprilTagDetection> AprilDetections;

        AprilDetections = apriltagProcesor.getDetections();

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
                        distx=april2.x;
                        disty=april2.y;

                        break;
                    case 3:
                        april3.setVector(currentDetection.ftcPose.x, currentDetection.ftcPose.y, currentDetection.ftcPose.z);
                        break;
                }
                telemetry.addLine("AprilTag1: X " + String.valueOf(april1.x) + " Y " + String.valueOf(april1.y) + " Z " + String.valueOf(april1.z));
                telemetry.addLine("AprilTag2: X " + String.valueOf(april2.x) + " Y " + String.valueOf(april2.y) + " Z " + String.valueOf(april2.z));
                telemetry.addLine("AprilTag3: X " + String.valueOf(april3.x) + " Y " + String.valueOf(april3.y) + " Z " + String.valueOf(april3.z));
                telemetry.update();
                sleep(20);
            }
        }
    }

}
