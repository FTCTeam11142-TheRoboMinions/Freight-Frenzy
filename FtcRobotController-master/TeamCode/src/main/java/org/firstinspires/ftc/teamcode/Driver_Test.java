package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Driver_Test", group = "Default")
public class Driver_Test extends OpMode {
    //Arm1
    private double liftPosScale = 50, liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    private double integrater = 0.001, intpower = 0.00075;
    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;
    int shootPos = 0;
    //Slide
    private double liftPosScale2 = 50, liftPowScale2 = 0.0025;
    private double liftPosCurrent2=0, liftPosDes2=0, liftPosError2=0, liftPow2=0;
    private double integrater2 = 0.001, intpower2 = 0.00075;
    double multiplier2 = 1, speedK2 = 1;
    boolean turtle2 = false, sloth2 = false;
    double rotPos2 = 0, foundPos2 = 1;
    int shootPos2 = 0;

    DcMotor leftvertical;
    DcMotor rightvertical;
    DcMotor lefthorizontal;
    DcMotor righthorzontal;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Slide;
    DcMotor Pick;


    public void init() {

        leftvertical = hardwareMap.dcMotor.get("lf");
        rightvertical = hardwareMap.dcMotor.get("rr");
        lefthorizontal = hardwareMap.dcMotor.get("lr");
        righthorzontal = hardwareMap.dcMotor.get("rf");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        //Slide = hardwareMap.dcMotor.get("Slide");
        Pick = hardwareMap.dcMotor.get("Pick");
        leftvertical.setDirection(DcMotorSimple.Direction.REVERSE);
        lefthorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        Arm1.setDirection(DcMotorSimple.Direction.REVERSE);

        leftvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lefthorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        righthorzontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveForward (double power) {
        //reset encoder
        leftvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        leftvertical.setTargetPosition(1 * 31);
        rightvertical.setTargetPosition(1 * 31);

        leftvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        leftvertical.setPower(power);
        rightvertical.setPower(power);

        while (leftvertical.isBusy() && rightvertical.isBusy()){

        }
        StopDriving();

    }
    public void Arm (double power) {
        //reset encoder
        Arm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        Arm1.setTargetPosition(1 * 31);


        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        //set power
        Arm1.setPower(power);


        while (Arm1.isBusy()){

        }
        StopDriving();

    }
    public void RobotSpin (double power) {
        //reset encoder
        leftvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightvertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        righthorzontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lefthorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        leftvertical.setTargetPosition(1 * -31);
        rightvertical.setTargetPosition(1 * 31);
        righthorzontal.setTargetPosition(1 * -31);
        lefthorizontal.setTargetPosition(1 * 31);

        leftvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightvertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        righthorzontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lefthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        leftvertical.setPower(power);
        rightvertical.setPower(power);
        righthorzontal.setPower(power);
        lefthorizontal.setPower(power);

        while (leftvertical.isBusy() && rightvertical.isBusy() && righthorzontal.isBusy() && lefthorizontal.isBusy()){

        }
        StopDriving();

    }
    public void DriveSide (double power){
        //reset
        righthorzontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lefthorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //target position
        righthorzontal.setTargetPosition(1 * 31);
        lefthorizontal.setTargetPosition(1 * 31);

        righthorzontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lefthorizontal.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        righthorzontal.setPower(power);
        lefthorizontal.setPower(power);

        while (righthorzontal.isBusy() && lefthorizontal.isBusy()) {

        }
        StopDriving();
    }

    public void Spin(double power) {
        //reset encoder
        Top.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Top.setTargetPosition(1 * 31);
        Top.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        Top.setPower(power);

        while (Top.isBusy()) {

        }
        StopDriving();

    }
    public void StopDriving(){

        leftvertical.setPower(0);
        righthorzontal.setPower(0);
        rightvertical.setPower(0);
        lefthorizontal.setPower(0);
        Top.setPower(0);
    }
    public void loop() {
        if(gamepad1.right_stick_y != 0){DriveSide(gamepad1.right_stick_y);}else{StopDriving();}
        if(gamepad1.right_stick_x != 0){DriveForward(gamepad1.right_stick_x*-1);}
        if(gamepad1.left_trigger != 0){RobotSpin(gamepad1.left_trigger / 2);}
        if(gamepad1.right_trigger != 0){RobotSpin((gamepad1.right_trigger *-1)/2);}
        /*DriveForward(gamepad1.right_stick_x*-1);
        Spin((gamepad1.left_trigger / 2);
        Spin((gamepad1.right_trigger *-1) /2);*/


        //Forward, Back, Left, Right
        /*leftvertical.setPower(gamepad1.right_stick_x*-1);
        rightvertical.setPower(gamepad1.right_stick_x*-1);
        //Forward and Backward - left stick - up/down
        lefthorizontal.setPower(gamepad1.right_stick_y);
        righthorzontal.setPower(gamepad1.right_stick_y);

        //Spin
        lefthorizontal.setPower(gamepad1.left_trigger / 2);
        righthorzontal.setPower(-gamepad1.left_trigger / 2);
        leftvertical.setPower(gamepad1.left_trigger / 2);
        rightvertical.setPower(-gamepad1.left_trigger / 2);
        // Spin other way
        lefthorizontal.setPower(-gamepad1.right_trigger / 2);
        righthorzontal.setPower(gamepad1.right_trigger / 2);
        leftvertical.setPower(-gamepad1.right_trigger / 2);
        rightvertical.setPower(gamepad1.right_trigger / 2);*/
        //Spin Carousel
        if (gamepad2.a) {
            Top.setPower(0.2);
        } else if (gamepad2.y){
            Top.setPower(-0.2);
        }else{
            Top.setPower(0);
        }

        //Intake
        if (gamepad2.x) {
            Pick.setPower(1);
        } else if (gamepad2.b) {
            Pick.setPower(-1);
        } else {
            Pick.setPower(0);
        }

        //PID for Arm1
        liftPosCurrent = Arm1.getCurrentPosition();
        liftPosDes += speedK*liftPosScale*gamepad2.left_stick_y/2;                //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
//      integrater += liftPosError;                                             //unnecessary
        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
        Arm1.setPower(liftPow);

        // PID for Slide
        /*liftPosCurrent2 = Slide.getCurrentPosition();
        liftPosDes2 += speedK2*liftPosScale2*gamepad2.right_stick_x;                //input scale factor
        liftPosError2 = liftPosDes2 - liftPosCurrent2;
//      integrater2 += liftPosError2;                                              //unnecessary
        liftPow2 = Math.min(Math.max(liftPowScale2*liftPosError2, -1.00), 1.00);   //proportional gain
        if(liftPow2 >= 1){ liftPosDes2 = liftPosCurrent2+(1/liftPowScale2); }       //AntiWindup Code
        if(liftPow2 <= -1) {liftPosDes2 = liftPosCurrent2-(1/liftPowScale2); }      //AntiWindup Code
        Slide.setPower(liftPow2);*/
    }
}