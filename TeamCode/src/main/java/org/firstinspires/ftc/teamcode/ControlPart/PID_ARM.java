package org.firstinspires.ftc.teamcode.ControlPart;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;



@Config
@TeleOp(name = "PID ARM")
public class PID_ARM extends OpMode {

    private PIDController controller;

    public static double p=0, i=0, d=0;
    public static double f=0.04;

    public static int target=0;

    public double val=0;

    private final double ticks_in_degrees=288/(360.0*0.36); /// gear ratio: 45/125=0.36

    private DcMotorEx elevator1, elevator2;


    @Override
    public void init() {
        controller=new PIDController(p,i,d);
        telemetry=new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        elevator1= hardwareMap.get(DcMotorEx.class,"e1");
        elevator2= hardwareMap.get(DcMotorEx.class,"e2");


        elevator1.setDirection(DcMotorSimple.Direction.REVERSE);
        elevator2.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {

        controller.setPID(p,i,d);
        int elepos=elevator1.getCurrentPosition();

        double pid=controller.calculate(elepos, target);
        double ff=Math.cos(Math.toRadians(target/ticks_in_degrees))*f;

        double power = pid + ff;

        elevator1.setPower(power);
        elevator2.setPower(power);

        telemetry.addData("pos ",elepos);
        telemetry.addData("target ", target);
        telemetry.update();

        /*

        if(gamepad2.left_trigger>0){
            val+=gamepad2.left_trigger;
        }

        if(gamepad2.right_trigger>0){
            val-=gamepad2.right_trigger;
        }

         */

       // target=(int) val;

    }

}
