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

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.BACKWARDS;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Implementations.Constants.Direction.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Robot;
import org.firstinspires.ftc.vision.VisionPortal;

import java.io.IOException;
import java.util.concurrent.TimeUnit;


public class BlueFrontStage {

    private int PARKING = 1; //-1 for left parking and 1 for right

    private static double target;
    private Robot robot;
    private int tagID;

    private final double ZERO_OFFSET = 70.0 - 3.85;
    private double TargetPosInDegrees = 70.0 - 3.85;
    private Telemetry telemetry;

    public BlueFrontStage(Robot robot, Telemetry telemetry, int tagID) {
        this.robot = robot;
        this.telemetry = telemetry;
        this.tagID = tagID;
    }

    public void init() {


        this.robot.camera.openFrontCam();
        target = this.robot.arm.ZERO_OFFSET;
        this.robot.wheels.setDirection();


    }

    public void frontStageLeftProp(int parking, int timer) {

        int parkDist;

        if (parking == -1) {

            parkDist = 70;

        } else {

            parkDist = 40;

        }


        this.robot.camera.openBackCam();

        setManualExposure(1, 1);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();

        robot.claw.setPosition(Claw.CLOSED);
        sleep(750);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);

        this.robot.wheels.GetDirection(telemetry);
        telemetry.update();

        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);

        this.robot.joint.setPosition(Joint.DOWN);


        this.robot.move.forward(FORWARD, 0.6, 45);
        sleep(175);

        this.robot.move.rotate(-1, 0.6, 51.5);
        sleep(200);


        this.robot.move.forward(FORWARD, 0.4, 12);
        sleep(200);

        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);


        this.robot.move.forward(BACKWARDS, 0.5, 5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        this.robot.joint.setPosition(Joint.UP);

        //AUTONOMIA CARE PARCHEAZA

        this.robot.move.rotate(1, 0.6, 51.5);

        this.robot.move.forward(BACKWARDS, 0.5, 50);

    }

    public void frontStageCenterProp(int parking, int timer) {

        int parkDist;

        if (parking == -1) {

            parkDist = 60;

        } else {

            parkDist = 60;

        }


        this.robot.camera.openBackCam();

        setManualExposure(1, 1);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();

        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);


        this.robot.move.lateral(RIGHT, 0.4, 8);
        sleep(175);

        this.robot.move.forward(FORWARD, 0.6, 60);//57
        sleep(175);
        this.robot.joint.setPosition(Joint.DOWN);
        sleep(500);
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(500);
        this.robot.move.forward(BACKWARDS, 0.5, 5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);
        this.robot.joint.setPosition(Joint.UP);

        this.robot.move.forward(BACKWARDS, 0.6, 50);
    }

    public void frontStageRightProp(int parking, double timer) {

        int parkDist;

        if (parking == -1) {

            parkDist = 40;

        } else {

            parkDist = 70;

        }

        this.robot.camera.openBackCam();

        setManualExposure(1, 1);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();


        robot.claw.setPosition(Claw.CLOSED);
        sleep(750);
        robot.joint.setPosition(Joint.DOWN);
        sleep(900);
        robot.claw.setPosition(Claw.CLOSED);
        sleep(900);


        this.robot.wheels.GetDirection(this.telemetry);
        this.telemetry.update();

        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(500);

        this.robot.joint.setPosition(Joint.DOWN);


        this.robot.move.lateral(RIGHT, 0.4, 24);
        sleep(175);

        this.robot.move.forward(FORWARD, 0.6, 36.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.INTERMEDIARY);
        sleep(250);
        this.robot.move.forward(BACKWARDS, 0.5, 5.5);
        sleep(175);
        this.robot.claw.setPosition(Claw.CLOSED);
        sleep(250);
        this.robot.joint.setPosition(Joint.UP);

        this.robot.move.forward(BACKWARDS, 0.6, 50);


    }

    private boolean setManualExposure(int exposureMS, int gain) {
        // Ensure Vision Portal has been setup.
        if (this.robot.camera.vision == null) {
            return false;
        }

        // Wait for the camera to be open
        if (this.robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING) {
            this.telemetry.addData("Camera", "Waiting");
            this.telemetry.update();
            while ((this.robot.camera.vision.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            this.telemetry.addData("Camera", "Ready");
            this.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!false) {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = this.robot.camera.vision.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure(7, TimeUnit.MILLISECONDS);
            sleep(20);

            this.telemetry.addLine("Exposure Ceva: " + exposureControl.getExposure(TimeUnit.MILLISECONDS));

            // Set Gain.
            GainControl gainControl = this.robot.camera.vision.getCameraControl(GainControl.class);

            this.telemetry.addLine("Gain Ceva: " + gainControl);


            if (gainControl != null) {
                boolean haide = gainControl.setGain(255);
                this.telemetry.addLine("Gain: " + gainControl.getGain());

            }
            sleep(20);
            return (true);
        } else {
            return (false);
        }
    }
}
