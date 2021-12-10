package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "PID TeleOp")
public class PID_Test extends LinearOpMode {



    private ElapsedTime Timer = new ElapsedTime();
/*
    public static PIDCoefficients pidCoffs = new PIDCoefficients(0,0,0);

    public static PIDCoefficients pidGains = new PIDCoefficients(0,0,0);
*/
    public float goal = 0;

    DcMotorEx ArmMotor;

    public void runOpMode() {
        ArmMotor = hardwareMap.get(DcMotorEx.class, "arm");

        ArmMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        ArmMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                Controller();
                ArmMotor.setPower(PID_Control(goal, ArmMotor.getCurrentPosition()));
            }


        }
    }

    public void Controller() {
        float x1 = gamepad1.right_stick_y;
        float x2 = -gamepad1.left_stick_y;

        if ((gamepad1.right_stick_y > -gamepad1.left_stick_y)) {

            goal += x1;

            telemetry.addData("Set Goal", x1);
            telemetry.update();

        } else {

            goal -= -x2;

            telemetry.addData("Set Goal", -x2);
            telemetry.update();
        }
    }

    double Kp = 0;
    double Ki = 0;
    double Kd = 0;


    double Kf = 1;

    double intergalsum = 0;
    double lastError = 0;


    public double PID_Control(double goal, double current)
    {
        double error = goal - current;
        intergalsum += error * Timer.seconds();
        double derivative = (error - lastError) / Timer.seconds();
        lastError = error;


        double output = (error * Kp) + (derivative * Kd) + (intergalsum * Ki) ;

        return output;
    }
}

/**
 Second Attempet (Under)
 **/

/*
    public void Controller() {
        float x1 = gamepad1.right_stick_y;
        float x2 = -gamepad1.left_stick_y;

        if ((gamepad1.right_stick_y > -gamepad1.left_stick_y)) {

            targetVelocity += x1;

            telemetry.addData("Set targetVelocity", x1);
            telemetry.update();

        } else {

            targetVelocity -= -x2;

            telemetry.addData("Set targetVelocity", -x2);
            telemetry.update();
        }
    }

    /*public void PID()
    {

        double currentVelocity = ArmMotor.getVelocity();
        double error = targetVelocity - currentVelocity;

        integral += error * Timer.time();


        double deltaError = error - lastError;
        double derivative = deltaError / Timer.time();

        pidGains.p = pidCoffs.p * error;
        pidGains.p = pidCoffs.i * integral;
        pidGains.p = pidCoffs.d * derivative;

        ArmMotor.setVelocity(pidGains.p+ pidGains.i + pidGains.d + targetVelocity);

        lastError = error;
    }*/
