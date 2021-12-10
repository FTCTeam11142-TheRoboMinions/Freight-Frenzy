package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="Red Just Carousel")
public class Red_Just_Carousel extends LinearOpMode {    //Declare motors
    DcMotor Fvertical;
    DcMotor Fhorizontal;
    DcMotor Bvertical;
    DcMotor Bhorizontal;
    DcMotor Top;
    DcMotor Arm1;
    //DcMotor Slide;
    DcMotor Pick;

    public void runOpMode() { //code will run once only

        //initilize motors
        Fvertical = hardwareMap.dcMotor.get("lr");
        Bvertical = hardwareMap.dcMotor.get("rf");
        Fhorizontal = hardwareMap.dcMotor.get("lf");
        Bhorizontal = hardwareMap.dcMotor.get("rr");
        Arm1 = hardwareMap.dcMotor.get("Spin");
        Top = hardwareMap.dcMotor.get("TOP");
        //Slide = hardwareMap.dcMotor.get("Slide");
        Pick = hardwareMap.dcMotor.get("Pick");

        Bhorizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        Bvertical.setDirection(DcMotorSimple.Direction.REVERSE);


        //set modes
        Fvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bvertical.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Fhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Bhorizontal.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Top.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            //DriveSide(0.5,20);
            //RobotSpin(0.5,30);
            //DriveSide(0.5,20);
            DriveForward(0.1, -30);
            Arm(0.5, 20);
            Spin(0.1, -25);
            DriveSide(0.5,20);
            Arm(0.5, -20);
            DriveForward(0.2, 23);
            RobotSpin(0.5, 15);
            DriveForward(0.2, 28);
            DriveForward(0.2, -2);
            DriveSide(0.4,20);
        }
    }
    public void DriveForward (double power, int distance) {
        //reset encoder
        Fvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Fvertical.setTargetPosition(distance * 31);
        Bvertical.setTargetPosition(distance * 31);


        Fvertical.setMode(RunMode.RUN_TO_POSITION);
        Bvertical.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);

        while (Fvertical.isBusy() && Bvertical.isBusy()){

        }
        StopDriving();

    }
    public void Arm (double power, int distance) {
        //reset encoder
        Arm1.setMode(RunMode.STOP_AND_RESET_ENCODER);


        //set target position
        Arm1.setTargetPosition(distance * 31);


        Arm1.setMode(RunMode.RUN_TO_POSITION);


        //set power
        Arm1.setPower(power);


        while (Arm1.isBusy()){

        }
        StopDriving();

    }
    public void RobotSpin (double power, int distance) {
        //reset encoder
        Fvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bvertical.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Fhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Fvertical.setTargetPosition(distance * -31);
        Bvertical.setTargetPosition(distance * 31);
        Fhorizontal.setTargetPosition(distance * -31);
        Bhorizontal.setTargetPosition(distance * 31);

        Fvertical.setMode(RunMode.RUN_TO_POSITION);
        Bvertical.setMode(RunMode.RUN_TO_POSITION);
        Fhorizontal.setMode(RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Fvertical.setPower(power);
        Bvertical.setPower(power);
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);

        while (Fvertical.isBusy() && Bvertical.isBusy() && Fhorizontal.isBusy() && Bhorizontal.isBusy()){

        }
        StopDriving();

    }
    public void DriveSide (double power, int distance){
        //reset
        Fhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);
        Bhorizontal.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //target position
        Fhorizontal.setTargetPosition(distance * 31);
        Bhorizontal.setTargetPosition(distance * 31);

        Fhorizontal.setMode(RunMode.RUN_TO_POSITION);
        Bhorizontal.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Fhorizontal.setPower(power);
        Bhorizontal.setPower(power);

        while (Fhorizontal.isBusy() && Bhorizontal.isBusy()) {

        }
        StopDriving();
    }

    public void Spin(double power, int distance) {
        //reset encoder
        Top.setMode(RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        Top.setTargetPosition(distance * 31);
        Top.setMode(RunMode.RUN_TO_POSITION);

        //set power
        Top.setPower(power);

        while (Top.isBusy()) {

        }
        StopDriving();

    }
    public void StopDriving(){

        Fvertical.setPower(0);
        Fhorizontal.setPower(0);
        Bvertical.setPower(0);
        Bhorizontal.setPower(0);
        Top.setPower(0);
    }
}
