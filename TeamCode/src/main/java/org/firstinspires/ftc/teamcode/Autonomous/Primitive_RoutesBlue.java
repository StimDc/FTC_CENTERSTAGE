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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Implementations.Constants.Claw;
import org.firstinspires.ftc.teamcode.Implementations.Constants.Joint;

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

@Autonomous(name="Primitive Routes Blue", group = "Robot")
@Disabled
public class Primitive_RoutesBlue extends LinearOpMode {

    private int apriltagid,idTarget=-1;

    private Servo joint, claw;

    Joint jointPos;
    Claw clawPos;

    Primitive_Movement move;

    FtcDashboard dashboard;

    @Override
    public void runOpMode() {

    }

    public void BLUE_BACKSTAGE(String parking, int waittimer){

        move.InitCamera();

        while(move.Red_Prop_Pos()=="nope") {
            switch (move.Red_Prop_Pos()) {

                case "left":
                    idTarget = 1;
                    Backstage_LeftProp_Blue(parking,waittimer);
                    break;

                case "center":
                    idTarget = 2;
                    Backstage_CenterProp_Blue(parking,waittimer);
                    break;

                case "right":
                    idTarget = 3;
                    Backstage_RightProp_Blue(parking,waittimer);
                    break;

            }
        }

    }

    public void BLUE_FRONTSTAGE(String parking, int waittimer){

        move.InitCamera();

        while(move.Red_Prop_Pos()=="nope") {
            switch (move.Red_Prop_Pos()) {

                case "left":
                    idTarget = 1;
                    Frontstage_LeftProp_Blue(parking,waittimer);
                    break;

                case "center":
                    idTarget = 2;
                    Frontstage_CenterProp_Blue(parking,waittimer);
                    break;

                case "right":
                    idTarget = 3;
                    Frontstage_RightProp_Blue(parking,waittimer);
                    break;

            }
        }

    }
    public void Backstage_CenterProp_Blue(String parking, int waittimer) {
        

    }

    public void Backstage_LeftProp_Blue(String parking, int waittimer){

    }

    public void Backstage_RightProp_Blue(String parking, int waittimer){

    }

    public void Frontstage_CenterProp_Blue(String parking, int waittimer){

    }

    public void Frontstage_LeftProp_Blue(String parking, int waittimer){

    }

    public void Frontstage_RightProp_Blue(String parking, int waittimer){

    }

}
