package org.firstinspires.ftc.teamcode.ControlPart;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Autonomous.Primitive_Movement;
import org.firstinspires.ftc.teamcode.Implementations.Constants.PIDConstantsAngle;



@TeleOp(name = "PID ANGLE")
public class PID_ANGLE extends OpMode {
    double integralSum = 0;

    static double Kp = PIDConstantsAngle.Kp;
    static double Ki = PIDConstantsAngle.Ki;
    static double Kd = PIDConstantsAngle.Kd;



    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;

    private BNO055IMU imu;

    static double referenceAngle;

    DcMotor frontLeft, frontRight,backLeft,backRight;

    public void RunOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //init motors
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class,"FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        referenceAngle = Math.toRadians(90);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //waitForStart();
        while(true){
            telemetry.addData("Target IMU Angle", referenceAngle);
            telemetry.addData("Current IMU Angle",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            double power = PIDControl(referenceAngle,imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES).firstAngle);
            frontLeft.setPower(power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(power);
            telemetry.update();
        }
    }

@Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        //init motors
        frontLeft = hardwareMap.get(DcMotor.class, "FL");
        frontRight = hardwareMap.get(DcMotor.class,"FR");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        backRight = hardwareMap.get(DcMotor.class, "BR");

        referenceAngle=PIDConstantsAngle.angle;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        referenceAngle = 90;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

@Override
    public void loop() {

    referenceAngle=PIDConstantsAngle.angle;

    telemetry.addData("Target IMU Angle", referenceAngle);
        telemetry.addData("Current IMU Angle",imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        double power = PIDControl(referenceAngle,imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.RADIANS).firstAngle);
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        telemetry.update();
    }
    public double PIDControl(double reference,double state){
        double error = angleWrap(reference-state);
        telemetry.addData("Error: ",error);
        integralSum+=error*timer.seconds();
        double derivative = (error - lastError)/(timer.seconds());
        lastError = error;
        timer.reset();
        double output = (error * Kp) + (derivative * Kd) + (integralSum *Ki);
        return output;
    }
    public double angleWrap(double radians){
        while(radians>Math.PI){
            radians -=2*Math.PI;

        }
        while(radians < -Math.PI){
            radians +=2*Math.PI;
        }
        return radians;
    }
    public double angleWrapDegrees(double degrees){
        while(degrees > 180){
            degrees -=360;
        }
        while(degrees <-180){
            degrees +=360;
        }
        return degrees;
    }
}
