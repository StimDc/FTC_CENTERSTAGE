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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Implementations.ChoiceMenu.ChoiceMenu;

import java.io.IOException;
import java.util.Dictionary;

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

@Autonomous(name="AUTONOMIE PEIMITIVE", group = "Robot")

public class Autonomie_Primitive extends LinearOpMode {
    Dictionary<String,String> paramaters;

    private Primitive_RoutesRed routeRed;
    private Primitive_RoutesBlue routeBlue;

    private String alliance; // RED=1 && BLUE=2
    private String startpoint; // FRONTSTAGE=1 && BACKSTAGE=2
    private String parking; // LEFT=1 && RIGHT=2
    private int wait;//how much to wait in seconds


    @Override
    public void runOpMode() {

        try {
            paramaters = ChoiceMenu.readFromFile();

        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        alliance=paramaters.get("alliance"); //RED or BLUE
        startpoint=paramaters.get("position");//FRONT_STAGE or BACK_STAGE
        parking=paramaters.get("parking");//LEFT or RIGHT

        String timer=paramaters.get("timer");// HOW TO CONVERT FROM STRING TO INT???

        try{
            wait = Integer.parseInt(timer);
        }
        catch (NumberFormatException ex){
            ex.printStackTrace();
        }

        if(alliance=="RED" && startpoint=="FRONT_STAGE"){

            routeRed.RED_FRONTSTAGE(parking,wait);

        }else if(alliance=="RED" && startpoint=="BACK_STAGE"){

            routeRed.RED_BACKSTAGE(parking,wait);

        }else if(alliance=="BLUE" && startpoint=="FRONT_STAGE"){

           routeBlue.BLUE_FRONTSTAGE(parking,wait);

        }else if(alliance=="BLUE" && startpoint=="BACK_STAGE"){

           routeBlue.BLUE_BACKSTAGE(parking,wait);

        }
    }
}
