package org.firstinspires.ftc.teamcode.ControlPart;

import static java.lang.Math.abs;
import static android.os.SystemClock.sleep;

import android.util.Size;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Implementations.Camera.RedPropThreshold;
import org.firstinspires.ftc.teamcode.Implementations.DebugTools.CatchingBugs;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.Experimental;
import org.firstinspires.ftc.teamcode.Implementations.Annotations.ImplementedBy;
import org.firstinspires.ftc.teamcode.Implementations.Robot.Wheels;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Testare Rotiri")

public class testareRotiri extends OpMode {

    public BNO055IMU imu;
    public Orientation angles;

    private double drive, strafe,turn;
    Wheels wheels;
    public double power;
    @Override
    public void init() {
        wheels = new Wheels(hardwareMap);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);



    }


    @Override
    public void loop() {


        if(Math.abs(gamepad1.left_stick_x) >0.4){
            power = gamepad1.left_stick_x/2;
            wheels.setPower(-power ,power,-power,power);
        }

        angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        telemetry.update();

    }


    



}
