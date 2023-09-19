package org.firstinspires.ftc.teamcode.Implementations.DebugTools;

import org.firstinspires.ftc.teamcode.ControlPart.Sponsori;
import org.firstinspires.ftc.teamcode.Implementations.DebugTools.CatchingBugs;

public class ComputerTool {

    /**
    Debug code here without uploading to the robot
     */
    public static void main(String args[]){
        //CatchingBugs.getNameReport("Andrei");
       // CatchingBugs.getExperimental(Sponsori.class);
        ReportBuilder report = new ReportBuilder();
        report.addToReport("da").addToReport("nu");
        CatchingBugs.getExperimental(Sponsori.class,report);

    }
}
