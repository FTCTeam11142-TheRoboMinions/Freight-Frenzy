package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
public class Example extends LinearOpMode {

    Hardware TIseBot = new Hardware();
    private ElapsedTime runtime = new ElapsedTime();

    OpenCvCamera webcam;
    static RingPipeline pipeline;

    // Constants to find the amount of encoder ticks per CM
    static final double COUNTS_PER_MOTOR_REV = 384.5;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_CM = 10.16;

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
        //telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
        

        //Dectecting Bottom
        if ((pipeline.region1Avg() > pipeline.region2Avg()))
        {
            if ((pipeline.region1Avg() > pipeline.region3Avg()))
            {
                telemetry.addLine("Entering Bottom");
                telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
                telemetry.update();
                //sleep(1000);

                DriveForward(0.5,-15);

                DriveSide(0.5, -140);
                telemetry.addLine("Side");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,-100);
                telemetry.addLine("Forward");
                telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
                telemetry.update();
                sleep(1000);

                encoderArm(700, 0.1);
                sleep(100);
                telemetry.addLine("leaving if");
                sleep(1000);
            }
            //dectecting top
            else
            {
                telemetry.addLine(" Entering Top");
                telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
                telemetry.update();
                //sleep(1000);
                telemetry.addLine("Top1");
                telemetry.update();
                DriveForward(0.2,-20);

                sleep(750);

                DriveSide(0.2, -170);
                telemetry.addLine("Side");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,-170);
                telemetry.addLine("Forward");
                telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
                telemetry.update();
                sleep(1000);

               encoderArm(940, 1);
                sleep(100);
                telemetry.addLine("leaving if");
                sleep(2000);
            }

        }else {
            //dectecting middle
            if (pipeline.region2Avg() > pipeline.region3Avg()) {

                telemetry.addLine("Entering Middle");
                telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
                telemetry.update();
                //sleep(1000);

                DriveForward(0.5,-20);

                sleep(750);

                DriveSide(0.5, -170);
                telemetry.addLine("Side");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,-135);
                telemetry.addLine("Forward");
                telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
                telemetry.update();
                sleep(1000);

                encoderArm(990, 0.1);
                sleep(100);
                telemetry.addLine("leaving if");
                sleep(2000);

            } else
            {
                //dectecting top
                telemetry.addLine(" Entering Top");
                telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
                telemetry.update();
                //sleep(1000);
                telemetry.addLine("Top2");
                telemetry.update();
                DriveForward(0.2,-20);

                sleep(750);

                DriveSide(0.2, -170);
                telemetry.addLine("Side");
                telemetry.update();
                sleep(1000);

                DriveForward(0.2,-170);
                telemetry.addLine("Forward");
                telemetry.addData("whole error",(10*COUNTS_PER_CM)-wheels[0].getCurrentPosition());
                telemetry.update();
                sleep(1000);

                encoderArm(940, 1);
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

    public void encoderArm(double counts, double speed){
        int newArmTarget = 0;

        if (opModeIsActive()) {
            newArmTarget  = TIseBot.arm.getCurrentPosition() + (int) counts;

            TIseBot.arm.setTargetPosition(newArmTarget);
            TIseBot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        TIseBot.arm.setPower(speed);

        while (TIseBot.arm.isBusy()){
            TIseBot.arm.setPower(speed);
            telemetry.addData("Target Position: ", newArmTarget);
            telemetry.addData("Current Position: ", TIseBot.arm.getCurrentPosition());
            telemetry.update();
        }

        TIseBot.arm.setPower(0);
        TIseBot.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void DriveForward(double power,int distance)
    {

        //reset encoder
        TIseBot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //TIseBot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //TIseBot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        TIseBot.leftRear.setTargetPosition(distance * 5);
        //TIseBot.rightRear.setTargetPosition(distance * 5);
        //TIseBot.leftFront.setTargetPosition(distance * 5);
        TIseBot.rightFront.setTargetPosition(distance * 5);

        //Go to Position
        TIseBot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // TIseBot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // TIseBot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TIseBot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        TIseBot.leftRear.setPower(power);
        // TIseBot.rightRear.setPower(power);
        // TIseBot.leftFront.setPower(power);
        TIseBot.rightFront.setPower(power);


        while (TIseBot.leftRear.isBusy()  && TIseBot.rightFront.isBusy()) {
        }
        StopDriving();
    }
    public void DriveSide(double power,int distance)
    {

        //reset encoder
        //TIseBot.leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TIseBot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //TIseBot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //set target position
        //TIseBot.leftRear.setTargetPosition(distance * 5);
        TIseBot.rightRear.setTargetPosition(distance * 5);
        TIseBot.leftFront.setTargetPosition(distance * 5);
        //TIseBot.rightFront.setTargetPosition(distance * 5);

        //Go to Position
        //TIseBot.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TIseBot.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        TIseBot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //TIseBot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power
        //TIseBot.leftRear.setPower(power);
        TIseBot.rightRear.setPower(power);
        TIseBot.leftFront.setPower(power);
        //TIseBot.rightFront.setPower(power);


        while (TIseBot.rightRear.isBusy()  && TIseBot.leftFront.isBusy()) {
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






