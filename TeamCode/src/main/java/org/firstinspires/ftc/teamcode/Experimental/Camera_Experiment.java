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

package org.firstinspires.ftc.teamcode.Experimental;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Implementations.Camera.BluePropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Backstage;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold_Frontstage;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name="Experimentare Culori", group = "Robot")

public class Camera_Experiment extends LinearOpMode {

    private RedPropThreshold_Frontstage redProp;

    private BluePropThreshold_Backstage blueProp;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {

        redProp=new RedPropThreshold_Frontstage();
        blueProp=new BluePropThreshold_Backstage();

        visionPortal = new VisionPortal.Builder()
                .addProcessors(redProp,blueProp)
                .setCamera(hardwareMap.get(WebcamName.class,"Camera2"))
                .setCameraResolution(new Size(640,480))
                .build();

        while(visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING){

        }

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            telemetry.addLine("Position RED Prop: "+redProp.getPropPosition());
            telemetry.addLine("**********");
            telemetry.addLine("LeftRED side: " +redProp.PrintLeft());
            telemetry.addLine("CenterRED side: "+redProp.PrintCenter());
            telemetry.addLine("--------");
            telemetry.addLine("--------");
            telemetry.addLine("--------");

            telemetry.addLine("Position BLUE Prop: "+blueProp.getPropPosition());
            telemetry.addLine("**********");
            telemetry.addLine("LeftBLUE side: " +blueProp.PrintLeft());
            telemetry.addLine("CenterBLUE side: "+blueProp.PrintCenter());

            telemetry.update();
        }

    }
}
