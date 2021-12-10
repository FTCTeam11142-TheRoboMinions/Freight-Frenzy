package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "Luna Teleop2")
public class LunaTeleop2 extends OpMode {
    private double liftPosScale = 50, liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    private double integrater = 0.001, intpower = 0.00075;
    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;
    int shootPos = 0;
    float xx;
    float yy;
    float RR;
    float Ll;
    boolean slow = false;
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    //DcMotor intake;
    //DcMotor arm;
    DcMotor carousel;
    DcMotor arm;
    //DcMotor Slide;
    DcMotor spin;
    //Servo intake;
    //Servo drop;
    // above initializes all the aspects we need to make our robot function
    /*private double liftPosScale = 50, liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    private double integrater = 0.001, intpower = 0.00075;
    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;
    int shootPos = 0;*/
    @Override
    public void init() {
        // defining all the hardware
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightRear = hardwareMap.dcMotor.get("rr");
        rightFront = hardwareMap.dcMotor.get("rf");
        //intake = hardwareMap.servo.get("intake");
        arm = hardwareMap.dcMotor.get("arm");
        //drop = hardwareMap.servo.get("drop");
        spin = hardwareMap.dcMotor.get("spin");
        carousel = hardwareMap.dcMotor.get("carousel");
        //this puts the motors in reverse
        rightFront.setDirection(DcMotor.Direction.REVERSE);//alternating between forward and reverse depending on motor placement
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        if(gamepad1.a){
            slow =!slow;
        }
        if(slow){
            xx = gamepad1.right_stick_x/4;
            yy = -gamepad1.left_stick_y/4;
            RR = -gamepad1.right_trigger/4;
            Ll = -gamepad1.left_trigger/4;
        } else{
            xx = gamepad1.right_stick_x;
            yy = -gamepad1.left_stick_y;
            RR = -gamepad1.right_trigger;
            Ll = -gamepad1.left_trigger;
        }
        float x1 = xx;
        float y1 = yy;
        float r1 = RR;
        float r2 = Ll;
        telemetry.update();
        // Reset variables
        float leftFrontPower = 0;
        float leftBackPower = 0;
        float rightFrontPower = 0;
        float rightBackPower = 0;
        boolean cw = gamepad2.right_bumper;
        boolean ccw = gamepad2.left_bumper;
        // Handle regular movement
        leftFrontPower += y1;
        leftBackPower += y1;
        rightFrontPower += y1;
        rightBackPower += y1;
        // Handle strafing movement
        leftFrontPower += x1;
        leftBackPower -= x1;
        rightFrontPower -= x1;
        rightBackPower += x1;
        // Handle clockwise turning movement
        leftFrontPower -= r1*0.75;
        leftBackPower -= r1*0.75;
        rightFrontPower += r1*0.75;
        rightBackPower += r1*0.75;
        // Handle counterclockwise turning movement
        leftFrontPower += r2*0.75;
        leftBackPower += r2*0.75;
        rightFrontPower -= r2*0.75;
        rightBackPower -= r2*0.75;
        if (cw == true) {
            carousel.setPower(0.2);
        } else if (ccw == true){
            carousel.setPower(-0.2);
        }else{
            carousel.setPower(0);
        }
        spin.setPower(gamepad2.right_stick_x/4);
        //arm.setPower(gamepad2.right_stick_y/2);
        //spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        /*if (gamepad2.x) {
            drop.setPosition(0.5);
        }else{
            drop.setPosition(0);
        }
        if (gamepad2.b) {
            intake.setPower(1);
        }else if (gamepad2.a) {
            intake.setPower(-1);
        } else{
            intake.setPower(0);
        }*/
        // Scale movement
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));
        if (max > 1) {
            leftFrontPower = (float) Range.scale(leftFrontPower, -max, max, -.30, .30);
            leftBackPower = (float) Range.scale(leftBackPower, -max, max, -.30, .30);
            rightFrontPower = (float) Range.scale(rightFrontPower, -max, max, -.30, .30);
            rightBackPower = (float) Range.scale(rightBackPower, -max, max, -.30, .30);
        }
        leftRear.setPower(-leftBackPower);
        leftFront.setPower(-leftFrontPower);
        rightFront.setPower(-rightFrontPower);
        rightRear.setPower(-rightBackPower);

        liftPosCurrent = arm.getCurrentPosition();
        liftPosDes += speedK*liftPosScale*gamepad2.right_stick_y/2;                //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
//      integrater += liftPosError;                                             //unnecessary
        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
        arm.setPower(liftPow);
    }
}