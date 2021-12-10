package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

@Autonomous
public class Example_2 extends LinearOpMode {

    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera webcam;
    static RingPipeline pipeline;

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 537.7;
    static final double DRIVE_GEAR_REDUCTION = 1.35;
    static final double WHEEL_DIAMETER_CM = 9.60;

    // Finds the amount of encoder ticks per CM
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    DcMotor[] wheels = new DcMotor[1];

    @Override
    public void runOpMode() {

        TIseBot.init(hardwareMap);

        wheels[0] = TIseBot.arm;

        for (DcMotor wheel : wheels){
            wheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            wheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new RingPipeline();
            webcam.setPipeline(pipeline);

            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        }

        telemetry.addData("Path0",  "Starting at %7d ", TIseBot.arm.getCurrentPosition(), //TIseBot.rightFront.getCurrentPosition(), TIseBot.leftRear.getCurrentPosition(), TIseBot.rightRear.getCurrentPosition());

                telemetry.update());

        waitForStart();

        //Dectecting Bottom
        if ((pipeline.region1Avg() > pipeline.region2Avg()))
        {
            if ((pipeline.region1Avg() > pipeline.region3Avg()))
            {
                telemetry.addLine("Entering Bottom");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,100);
                telemetry.addLine("Forward");
                telemetry.update();
                sleep(1000);

                PIDDrive(10, 0.5);
                sleep(100);
                telemetry.addLine("leaving if");
                sleep(2000);
            }
            //dectecting top
            else {
                telemetry.addLine(" Entering Top");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2, 100);
                telemetry.addLine("Forward");
                telemetry.update();
                sleep(1000);

                PIDDrive(10, 0.5);
                sleep(100);
                telemetry.addLine("leaving if");
                sleep(2000);
            }



        }

        telemetry.update();


    }
    public static class RingPipeline extends OpenCvPipeline {

        /** Most important section of the code: Colors **/
        static final Scalar CRIMSON = new Scalar(220, 20, 60);
        static final Scalar AQUA = new Scalar(79, 195, 247);
        static final Scalar PARAKEET = new Scalar(3, 192, 74);
        static final Scalar GOLD = new Scalar(255, 215, 0);
        static final Scalar CYAN = new Scalar(0, 139, 139);

        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(100, 400);
        static final int REGION1_WIDTH = 200;
        static final int REGION1_HEIGHT = 200;
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(575,400);
        static final int REGION2_WIDTH = 200;
        static final int REGION2_HEIGHT = 200;
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1059,340);
        static final int REGION3_WIDTH = 200;
        static final int REGION3_HEIGHT = 200;


        Point region1_pointA = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x, REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(REGION1_TOPLEFT_ANCHOR_POINT.x + REGION1_WIDTH, REGION1_TOPLEFT_ANCHOR_POINT.y + REGION1_HEIGHT);

        Point region2_pointA = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x, REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(REGION2_TOPLEFT_ANCHOR_POINT.x + REGION2_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION2_HEIGHT);

        Point region3_pointA = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x, REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(REGION3_TOPLEFT_ANCHOR_POINT.x + REGION3_WIDTH, REGION2_TOPLEFT_ANCHOR_POINT.y + REGION3_HEIGHT);

        Mat region1_G, region2_G, region3_G;
        Mat BGR = new Mat();
        Mat G = new Mat();
        int avg1, avg2, avg3;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToG(Mat input)
        {
            Imgproc.cvtColor(input, BGR, Imgproc.COLOR_RGB2BGR);
            Core.extractChannel(BGR, G, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToG(firstFrame);

            region1_G = G.submat(new Rect(region1_pointA, region1_pointB));
            region2_G = G.submat(new Rect(region2_pointA, region2_pointB));
            region3_G = G.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {

            inputToG(input);

            avg1 = (int) Core.mean(region1_G).val[0];
            avg2 = (int) Core.mean(region2_G).val[0];
            avg3 = (int) Core.mean(region3_G).val[0];


            Imgproc.rectangle(input, region1_pointA, region1_pointB, CRIMSON,2);
            Imgproc.rectangle(input, region2_pointA, region2_pointB, AQUA,2);
            Imgproc.rectangle(input, region3_pointA, region3_pointB, PARAKEET,2);


            return input;

        }

        public int region1Avg() {
            return avg1;
        }
        public int region2Avg() {
            return avg2;
        }
        public int region3Avg() {
            return avg3;
        }

    }


    public void PIDDrive(double distanceCM, double tolerance) { // TODO: Adjust Tolerance

        int[] newWheelTarget = new int[1];

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newWheelTarget[0] = TIseBot.arm.getCurrentPosition() + (int) (distanceCM * COUNTS_PER_CM);

            double[] P = new double[1];

            double kp = 1;
            double kd = 0.000075; // Constant of derivation
            double ki = 0.000006;

            double dt = 20; //Delta time = 20 ms/cycle
            double dtS = dt / 1000;

            double[] error = new double[1];
            double[] previousError = new double[1];
            double[] power = new double[1];

            double[] area = new double[1];
            double[] previousArea = new double[1];


            error[0] = tolerance + 1;

            while ((Math.abs(error[0]) > tolerance) && opModeIsActive()) {// TODO: replace error[0] with avgError

                telemetry.addData("Path1", "Running to %7d ", newWheelTarget[0]); //newWheelTarget[1], newWheelTarget[2], newWheelTarget[3]);
                telemetry.addData("Path2", "Running at %7d ", wheels[0].getCurrentPosition());// wheels[1].getCurrentPosition(), wheels[2].getCurrentPosition(), wheels[3].getCurrentPosition());

                telemetry.addData("DistanceCM: ", (int) (distanceCM * COUNTS_PER_CM));

                telemetry.addData("power: ", power[0]);


                telemetry.addData("Proportion:", P[0]);
                telemetry.addData("Derivative:", kd * ((error[0] - previousError[0]) / dtS));
                telemetry.addData("Integral:", ki * area[0]);

                telemetry.addData("de(t)/dt", ((error[0] - previousError[0]) / dtS));

                telemetry.addData("error:", error[0]);
                telemetry.addData("previous error:", previousError[0]);

                telemetry.addData("âˆ«e(t)dt:", area[0]);
                telemetry.addData("previous âˆ«e(t)dt:", previousArea[0]);

                telemetry.addData("dtS", dtS);

                telemetry.update();

                previousError[0] = error[0];
                //previousError[1] = error[1];
                //previousError[2] = error[2];
                //previousError[3] = error[3];

                error[0] = (int) (distanceCM * COUNTS_PER_CM) - wheels[0].getCurrentPosition();
                //error[1] = (int)(distanceCM * COUNTS_PER_CM) - wheels[1].getCurrentPosition();
                //error[2] = (int)(distanceCM * COUNTS_PER_CM) - wheels[2].getCurrentPosition();
                //error[3] = (int)(distanceCM * COUNTS_PER_CM) - wheels[3].getCurrentPosition();

                P[0] = Math.abs(error[0]) / (int) (distanceCM * COUNTS_PER_CM);
                //P[1] = Math.abs(error[1])/(int)(distanceCM * COUNTS_PER_CM);
                //P[2] = Math.abs(error[2])/(int)(distanceCM * COUNTS_PER_CM);
                //P[3] = Math.abs(error[3])/(int)(distanceCM * COUNTS_PER_CM);

                previousArea[0] = area[0];
                //previousArea[1] = area[1];
                //previousArea[2] = area[2];
                //previousArea[3] = area[3];

                area[0] = error[0] * dtS + previousArea[0];
                //area[1] = error[1] * dtS + previousArea[1];
                //area[2] = error[2] * dtS + previousArea[2];
                //area[3] = error[3] * dtS + previousArea[3];

                power[0] = kp * P[0] + kd * ((error[0] - previousError[0]) / dtS) + (ki * area[0]);
                //power[1] = kp * P[1] + kd * ((error[1] - previousError[1]) / dtS) + (ki * area[1]);
                //power[2] = kp * P[2] + kd * ((error[2] - previousError[2]) / dtS) + (ki * area[2]);
                //power[3] = kp * P[3] + kd * ((error[3] - previousError[3]) / dtS) + (ki * area[3]);

                wheels[0].setPower(power[0]);
                //wheels[1].setPower(power[1]);
                //wheels[2].setPower(power[2]);
                //wheels[3].setPower(power[3]);

                sleep((long) dt);

                // Stop all motion;
                wheels[0].setPower(0);


                // Resets encoders
                wheels[0].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                wheels[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            }
        }

    }

    public void DriveForward(double power,int distance)
    {

        //reset encoder
        TIseBot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        TIseBot.leftRear.setTargetPosition(distance * 5);
        TIseBot.rightRear.setTargetPosition(distance * 5);
        TIseBot.leftFront.setTargetPosition(distance * 5);
        TIseBot.rightFront.setTargetPosition(distance * 5);

        //Go to Position
        TIseBot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TIseBot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TIseBot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TIseBot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        TIseBot.leftRear.setPower(power);
        TIseBot.rightRear.setPower(power);
        TIseBot.leftFront.setPower(power);
        TIseBot.rightFront.setPower(power);


        while (TIseBot.leftRear.isBusy() && TIseBot.rightRear.isBusy() && TIseBot.leftFront.isBusy() && TIseBot.rightFront.isBusy()) {
        }
        StopDriving();
    }
    public void StopDriving(){
        TIseBot.leftRear.setPower(0);
        TIseBot.rightRear.setPower(0);
        TIseBot.leftFront.setPower(0);
        TIseBot.rightFront.setPower(0);
        //TIseBot.arm.setPower(0);
    }

}







