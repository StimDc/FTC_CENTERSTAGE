package org.firstinspires.ftc.teamcode.Implementations.ChoiceMenu;

public class ChoiceItem {
    private String text;
    private String value;

    public ChoiceItem(String text, String value){
        this.text = text;
        this.value = value;
    }
    public String getText(){
        return this.text;
    }
    public String getValue(){
        return this.value;
    }
}
