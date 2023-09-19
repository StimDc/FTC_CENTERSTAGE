package org.firstinspires.ftc.teamcode.Implementations.DebugTools;

public class ReportBuilder {
    private String report;

    ReportBuilder(){
        this.report = "";
    }

    public ReportBuilder addToReport(String detail){
        this.report+="\n" + detail;
        return this;
    }

    public String getReport(){
        return this.report;
    }
}
