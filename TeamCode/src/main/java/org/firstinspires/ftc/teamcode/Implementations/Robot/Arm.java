package org.firstinspires.ftc.teamcode.Implementations.Robot;

import static android.os.SystemClock.sleep;

import static org.firstinspires.ftc.teamcode.Implementations.Constants.UniversalConsts.TICKS_IN_DEGREES;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Implementations.Constants.PIDConstantsArm;

public class Arm {
    public PIDController controller;
    public DcMotorEx elevator1, elevator2;
    public Telemetry telemetry;
    private static int target = 3;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry){
        this.controller = new PIDController(PIDConstantsArm.p,PIDConstantsArm.i,PIDConstantsArm.d);
        this.telemetry =new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.elevator1 = hardwareMap.get(DcMotorEx.class, "e1");
        this.elevator2 = hardwareMap.get(DcMotorEx.class, "e2");

        this.setDirection(DcMotorSimple.Direction.REVERSE);
        this.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setMode(DcMotor.RunMode mode){
        elevator1.setMode(mode);
        elevator2.setMode(mode);
    }

    public void setDirection(DcMotorSimple.Direction direction){
        elevator1.setDirection(direction);
        elevator2.setDirection(direction);
    }

    public void setPower(double power){
        elevator1.setPower(power);
        elevator2.setPower(power);
    }
    public void moveArm(int desiredTarget){//TODO: RESCRIS

        if(elevator1.getCurrentPosition()<desiredTarget){

            for(target = elevator1.getCurrentPosition();target<=desiredTarget;target++){
                controller.setPID(PIDConstantsArm.p, PIDConstantsArm.i, PIDConstantsArm.d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(250);
            }

        }else if(elevator1.getCurrentPosition()>desiredTarget){

            for(target = elevator1.getCurrentPosition();target>=0;target--){
                controller.setPID(PIDConstantsArm.p, PIDConstantsArm.i, PIDConstantsArm.d);
                int elepos = elevator1.getCurrentPosition();
                double pid = controller.calculate(elepos, target);
                double ff = Math.cos(Math.toRadians(target / TICKS_IN_DEGREES));

                double power = pid + ff;

                elevator1.setPower(power);
                elevator2.setPower(power);
                sleep(250);
            }
        }
    }



        //public static int target=0;


        private static final double MOTOR_CPR = 288.0;
        private static final double GEAR_RATIO = 125.0 / 45.0;
        private static final double ARM_TICKS_PER_DEGREE = MOTOR_CPR * GEAR_RATIO / 360.0;
       // private static final double MAX_ARM_HOLDING_POWER = <some calibrated value here>;
        private static final double ZERO_OFFSET = 70.0-3.85;
        public static double targetPosInDegrees=70.0-3.85;
        private double powerLimit;





        public void armTask()
        {
            double targetPosInTicks = (targetPosInDegrees - ZERO_OFFSET) * ARM_TICKS_PER_DEGREE;
            double currPosInTicks = this.elevator1.getCurrentPosition();
            double pidOutput = this.controller.calculate(currPosInTicks, targetPosInTicks);
            // This ff is assuming arm at horizontal position is 90-degree.
            double ff = PIDConstantsArm.f * Math.sin(Math.toRadians(ticksToRealWorldDegrees(currPosInTicks)));
            double power = pidOutput + ff;
            // Clip power to the range of -powerLimit to powerLimit.
            power = power < -powerLimit ? -powerLimit : Math.min(power, powerLimit);
            this.elevator1.setPower(power);
            this.elevator2.setPower(power);
        }

        public boolean isOnTarget(double toleranceInDegrees)
        {
            double currPosInDegrees = getPosition();
            return Math.abs(targetPosInDegrees - currPosInDegrees) <= toleranceInDegrees;
        }

        public void setPosition(double targetPosInDegrees, double powerLimit)
        {
            this.targetPosInDegrees = targetPosInDegrees;
            this.powerLimit = Math.abs(powerLimit);
        }


        public double ticksToRealWorldDegrees(double ticks)
        {
            return ticks / ARM_TICKS_PER_DEGREE + ZERO_OFFSET;
        }

        public double getPosition()
        {
            return ticksToRealWorldDegrees(elevator1.getCurrentPosition());
        }

        //
    // In your OpMode
    //
        static final double ARM_HOME_POS = 45.0;
        static final double ARM_LOADING_POS = 50.0;
        static final double ARM_SCORING_POS = 270.0;
     //   Arm arm;
      //  Elevator elevator;
      //  DriveBase driveBase;
        int state = 0;

}
