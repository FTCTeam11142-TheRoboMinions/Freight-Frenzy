package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware {

    // Define Motor Variables
    DcMotor rightFront;
    DcMotor leftFront;
    DcMotor leftRear;
    DcMotor rightRear;
    DcMotor carousel;
    DcMotor arm;
    //DcMotor spin;
    CRServo intake;
    Servo drop;

    // Define Servo Variables
    //public Servo claw;
    // public CRServo joint;


    // Define Sensor Variables
    public BNO055IMU imu;
    public Orientation straight = null;

    // Local OpMode members
    com.qualcomm.robotcore.hardware.HardwareMap hardwareMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    // Constructor (Creating Object Type)
    public Hardware() {

    }

    // Initialize standard Hardware interfaces
    public void init(com.qualcomm.robotcore.hardware.HardwareMap hwMap){
        hardwareMap = hwMap;

        // Find Motors in phone config
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightRear = hardwareMap.get(DcMotor.class, "rr");
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftRear = hardwareMap.get(DcMotor.class, "lr");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        arm = hardwareMap.get(DcMotor.class, "arm");

        // Find Servos in phone config
        drop = hardwareMap.get(Servo.class,"drop");
        intake = hardwareMap.get(CRServo.class,"intake");


        //Find Sensors in phone config
        imu = hardwareMap.get(BNO055IMU.class, "imu");


        // Set all motors to zero power
        rightRear.setPower(0);
        rightRear.setPower(0);
        leftRear.setPower(0);
        leftFront.setPower(0);
        carousel.setPower(0);
        arm.setPower(0);

        // IMU Setup
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.accelUnit = BNO055IMU.AccelUnit.MILLI_EARTH_GRAVITY;
            /*
            We never use the accelerometer functions of the imu, so we set the
            accel unit to milli-earth-gravities as a joke
            */

        imu.initialize(parameters);

        straight = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);


        // Set Motor Direction
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.FORWARD);



        // Set the Motor's Encoder Setting
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        carousel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //


    }
}



