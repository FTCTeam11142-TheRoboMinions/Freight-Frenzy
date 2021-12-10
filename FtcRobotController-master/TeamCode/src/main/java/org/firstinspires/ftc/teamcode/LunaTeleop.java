package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name = "Luna Teleop")
public class LunaTeleop extends OpMode {
    private double liftPosScale = 50, liftPowScale = 0.0025;
    private double liftPosCurrent=0, liftPosDes=0, liftPosError=0, liftPow=0;
    private double integrater = 0.001, intpower = 0.00075;
    double multiplier = 1, speedK = 1;
    boolean turtle = false, sloth = false;
    double rotPos = 0, foundPos = 1;
    int shootPos = 0;

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
    CRServo intake;
    Servo drop;
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
        intake = hardwareMap.crservo.get("intake");
        arm = hardwareMap.dcMotor.get("arm");
        drop = hardwareMap.servo.get("drop");
        spin = hardwareMap.dcMotor.get("spin");
        carousel = hardwareMap.dcMotor.get("carousel");
        //this puts the motors in reverse
        rightFront.setDirection(DcMotor.Direction.REVERSE);//alternating between forward and reverse depending on motor placement
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
    }
    @Override
    public void loop() {
        float x1 = gamepad1.right_stick_x;
        float y1 = -gamepad1.left_stick_y;
        float r1 = -gamepad1.right_trigger;
        float r2 = -gamepad1.left_trigger;
        float i = gamepad2.left_stick_y/2;
        float s = gamepad2.left_stick_x/2;
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
            //spin.setPower(-gamepad2.right_stick_x/4);
        



        //spin.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (gamepad2.x) {
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
        }
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

        liftPosCurrent = spin.getCurrentPosition();
        liftPosDes += speedK*liftPosScale*-gamepad2.right_stick_x/4;                //input scale factor
        liftPosError = liftPosDes - liftPosCurrent;
//      integrater += liftPosError;                                             //unnecessary
        liftPow = Math.min(Math.max(liftPowScale*liftPosError, -1.00), 1.00);   //proportional gain
        if(liftPow >= 1){ liftPosDes = liftPosCurrent+(1/liftPowScale); }       //AntiWindup Code
        if(liftPow <= -1) {liftPosDes = liftPosCurrent-(1/liftPowScale); }      //AntiWindup Code
        spin.setPower(liftPow);
    }
}