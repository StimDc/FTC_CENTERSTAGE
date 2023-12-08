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

package org.firstinspires.ftc.teamcode.Implementations.ChoiceMenu;

import android.os.Environment;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Hashtable;

@Config
@Autonomous(name="Choice Menu", group = "Autonomie Primitiva")

public class ChoiceMenu extends LinearOpMode {
    ArrayList<ChoiceItem> buffer = new ArrayList<>();


    @Override
    @Experimental
    @ImplementedBy(name = "Andrei", date="07.12.23")
    public void runOpMode() {



        String[] alliances= {"BLUE", "RED"};
        ItemLogic("alliance", alliances);

        String[] positions= {"FRONT_STAGE","BACK_STAGE"};
        ItemLogic("position",positions);

        String[] parkings = {"LEFT", "RIGHT"};
        ItemLogic("parking", parkings);

        int timer =0;
        boolean timerChange = false;
        sleep(1000);
        while(!timerChange){
            telemetry.addLine("Please select the timer");
            if(gamepad1.dpad_up)
                timer+=500;

            if(gamepad1.dpad_down)
                timer-=500;

            telemetry.addLine(timer + "ms");
            if(gamepad1.a){
                timerChange = true;
                ChoiceItem timerChoice = new ChoiceItem("Timer", String.valueOf(timer));
                buffer.add(timerChoice);
                telemetry.addLine("TIMER CHOSEN: " + timer);
            }
            telemetry.update();
            sleep(200);
        }

        telemetry.update();


            try {
                writeToFile();
                readFromFile();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

        waitForStart();
    }

    @Experimental
    @ImplementedBy(name="Andrei", date="07.12.23")
    public void writeToFile() throws IOException {
        String logFilePath = String.format("%s/FIRST/data/autonomie.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
        FileWriter fw = new FileWriter(logFilePath);

        for(ChoiceItem choiceItem : buffer){
            telemetry.addLine(choiceItem.getText());
            fw.write(choiceItem.getText() + " " + choiceItem.getValue());
            fw.write('\n');

        }
        fw.write('#');
        fw.close();

    }

    @ImplementedBy(name="Andrei", date="08.12.23")
    public static Dictionary<String,String> readFromFile() throws IOException{
        String logFilePath = String.format("%s/FIRST/data/autonomie.txt", Environment.getExternalStorageDirectory().getAbsolutePath());
        File file = new File(logFilePath);
        BufferedReader br = new BufferedReader(new FileReader(file));
        String st;
        String split[];
        Dictionary<String, String> parsedFile = new Hashtable<>();
        while((st = br.readLine()) !=null){
            try {
                split = st.split(" ");
                parsedFile.put(split[0],split[1]);

            }catch(Exception e){
                break;
            }

        }

       return parsedFile;
    }

    @ImplementedBy(name = "Andrei", date="08.12.23")
    public void ItemLogic(String text,String[] choices){
        boolean change = false;
        int choice  =0;
        while(!change){
            telemetry.addLine("Please select the "+ text);
            if(gamepad1.dpad_up)
                choice = 1;
            if(gamepad1.dpad_down)
                choice = 0;
            telemetry.addLine(choices[choice]);
            if(gamepad1.a){
                change = true;
                ChoiceItem choiceItem = new ChoiceItem(text,choices[choice]);
                buffer.add(choiceItem);
                telemetry.addLine(text.toUpperCase() + " CHOSEN: " + choices[choice]);
            }
            telemetry.update();
            sleep(200);
        }
    }
}
