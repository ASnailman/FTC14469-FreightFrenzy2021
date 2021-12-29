package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name="SpectatorRed_V3", group="MecanumDrive")
public class SpectatorRed_V3 extends LinearOpMode {

    OpenCvWebcam webcam;
    BarcodeDeterminationPipeline pipeline;
    static int DifferenceLeft;
    static int DifferenceCenter;
    static int DifferenceRight;

    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor Arm;
    static DcMotor Rail;
    static DcMotor Intake;
    static DcMotor CarouselMotor;
    static DcMotor BucketMotor;
    static NormalizedColorSensor colorsensor;
    static RevBlinkinLedDriver ColorStrip;
    static Servo IntakeServo;
    static Servo GateServo;
    static MoveDirection Direction;

    BNO055IMU IMU;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    BNO055IMU ACCIMU;
    Orientation orientation;
    double globalangle;
    double current_value;
    double prev_value = 0;
    double final_value;

    ElapsedTime ET = new ElapsedTime();
    byte AXIS_MAP_SIGN_BYTE = 0x6; //rotates control hub 180 degrees around z axis by negating x and y signs
    byte AXIS_MAP_CONFIG_BYTE = 0x6; //rotates control hub 90 degrees around y axis by swapping x and z axis

    static final int Top_Arm_Left = -390;
    static final int Top_Arm_Right = 390;
    static final int Middle_Arm_Left = -290;
    static final int Middle_Arm_Right = 290;
    static final int Low_Arm_Left = -150;
    static final int Low_Arm_Right = 150;

    static final double OriginalBucketPosition = 0;
    static final double TopBucketPosition = 130;
    static final double MirrorTopBucketPosition = -130;
    static final double MiddleBucketPosition = 120;
    static final double MirrorMiddleBucketPosition = -120;
    static final double LowBucketPosition = 100;
    static final double MirrorLowBucketPosition = -100;

    static final double OpenGatePosition = 0.5;
    static final double OpenIntakePosition = 0.6;
    static final double ClosingGatePosition = 0.2;
    static final double ClosingIntakePosition = 0.8;

    boolean BucketIsEmpty = true;
    boolean white;
    boolean yellow;
    boolean unknown;

    int programorder1 = 0;
    int programorder2 = 0;
    Mech_Drive MechDrive;
    Bucket_Control BucketControl;
    Arm_Control ArmControl;

    boolean left;
    boolean center;
    boolean right;

    @Override
    public void runOpMode() {

        /**
         * NOTE: Many comments have been omitted from this sample for the
         * sake of conciseness. If you're just starting out with EasyOpenCv,
         * you should take a look at {@link InternalCamera1Example} or its
         * webcam counterpart, {@link WebcamExample} first.
         */

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        Rail = hardwareMap.get(DcMotor.class, "rail");
        CarouselMotor = hardwareMap.get(DcMotor.class, "carouselmotor");
        BucketMotor = hardwareMap.get(DcMotor.class, "BucketMotor");
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
        ColorStrip = hardwareMap.get(RevBlinkinLedDriver.class, "colorstrip");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        GateServo = hardwareMap.get(Servo.class, "GateServo");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        //rotate 180 around z axis and 90 degrees vertically around y axis
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);
        sleep(100);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_SIGN, AXIS_MAP_SIGN_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, AXIS_MAP_CONFIG_BYTE & 0x0F);
        IMU.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.IMU.bVal & 0x0F);
        sleep(100);

        AttachmentSetDirection();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;
        //Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Arm.setTargetPosition(0);
        Rail.setTargetPosition(300);

        //Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        IntakeServo.scaleRange(0,1);
        GateServo.scaleRange(0,1);

        IntakeServo.setPosition(ClosingIntakePosition);
        GateServo.setPosition(ClosingGatePosition);

        MechDrive = new Mech_Drive(FrontLeft, FrontRight, BackLeft, BackRight, MoveDirection.REVERSE, telemetry);
        BucketControl = new Bucket_Control(BucketMotor);
        ArmControl = new Arm_Control(Arm);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BarcodeDeterminationPipeline();
        webcam.setPipeline(pipeline);
        pipeline.InitTelemetry(telemetry);

        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        while (opModeIsActive()) {

            // We are going to write this autonomous program as a state machine
            // In other words, it will keep looping over and over and doing specific activities at specific times

            // This is the part of the program that is linear (i.e. only happen once)
            // The program1_seq is a counter that dictates that whatever has been completed won't be repeated again
            // This section can consist of methods that are called to put a task in the RUN state or even a one-time activity
            // such as DC Motor's RUN_TO_POSITION or setting a servo position
            // For features that require a background task, you should check the state of a task first before deciding what to do

            // The style of sequencing below is implemented so that only activities are done one after another (i.e. not in parallel)
            // program1_seq consists of a major sequence followed by a minor sequence (e.g. 1.2 mean major sequence 1 and subsequence 2)

            switch (programorder1) {

                case 0:
                    left = pipeline.position == BarcodeDeterminationPipeline.ShippingElementPosition.LEFT;
                    center = pipeline.position == BarcodeDeterminationPipeline.ShippingElementPosition.CENTER;
                    right = pipeline.position == BarcodeDeterminationPipeline.ShippingElementPosition.RIGHT;
                    programorder1++;
                    break;

                case 1:
                    if (MechDrive.GetTaskState() == Task_State.INIT) {
                        MechDrive.SetTargets(90, 975, 0.5);
                    }
                    else if (MechDrive.GetTaskState() == Task_State.DONE) {
                        programorder1++;
                    }
                    break;

                case 2:
                    if (MechDrive.GetTaskState() == Task_State.READY) {
                        MechDrive.SetTargets(180, 775, 0.5);
                    }
                    else if (MechDrive.GetTaskState() == Task_State.DONE) {
                        programorder1++;
                    }
                    break;

                case 3:
                    if (left) {
                        Rail.setTargetPosition(750);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.5);

                        if (Rail.getCurrentPosition() >= 720 && Rail.getCurrentPosition() <= 780) {
                            if (ArmControl.GetTaskState() == Task_State.INIT) {

                                ArmControl.SetTargetPosition(Low_Arm_Right, -0.6, 0.6);
                            }
                            else if (ArmControl.GetTaskState() == Task_State.DONE) {
                                programorder1++;
                            }
                        }
                    }
                    else if (center) {
                        Rail.setTargetPosition(750);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.5);

                        if (Rail.getCurrentPosition() >= 720 && Rail.getCurrentPosition() <= 780) {
                            if (ArmControl.GetTaskState() == Task_State.INIT) {

                                ArmControl.SetTargetPosition(Middle_Arm_Right, -0.6, 0.6);
                            }
                            else if (ArmControl.GetTaskState() == Task_State.DONE) {
                                programorder1++;
                            }
                        }
                    }
                    else if (right) {
                        Rail.setTargetPosition(1000);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.5);

                        if (Rail.getCurrentPosition() >= 970 && Rail.getCurrentPosition() <= 1030) {
                            if (ArmControl.GetTaskState() == Task_State.INIT) {

                                ArmControl.SetTargetPosition(Top_Arm_Right, -0.6, 0.6);
                            }
                            else if (ArmControl.GetTaskState() == Task_State.DONE) {
                                programorder1++;
                            }
                        }
                    }
                    break;

                case 4:
                    if (left) {
                        if (BucketControl.GetTaskState() == Task_State.INIT) {
                            BucketControl.SetTargetPosition(MirrorLowBucketPosition);
                        }
                        else if (BucketControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    else if (center) {
                        if (BucketControl.GetTaskState() == Task_State.INIT) {
                            BucketControl.SetTargetPosition(MirrorMiddleBucketPosition);
                        }
                        else if (BucketControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    else if (right) {
                        if (BucketControl.GetTaskState() == Task_State.INIT) {
                            BucketControl.SetTargetPosition(MirrorTopBucketPosition);
                        }
                        else if (BucketControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    break;

                case 5:

                    GateServo.setPosition(OpenGatePosition);
                    programorder1++;
                    ET.reset();
                    break;

                case 6:
                    if (left) {
                        if (BucketControl.GetTaskState() == Task_State.READY) {
                            if (ET.milliseconds() > 1500) {
                                GateServo.setPosition(ClosingGatePosition);
                                BucketControl.SetTargetPosition(2);
                            }
                        }
                        else if (BucketControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    else if (center) {
                        if (BucketControl.GetTaskState() == Task_State.READY) {
                            if (ET.milliseconds() > 1500) {
                                GateServo.setPosition(ClosingGatePosition);
                                BucketControl.SetTargetPosition(2);
                            }
                        }
                        else if (BucketControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    else if (right) {
                        if (BucketControl.GetTaskState() == Task_State.READY) {
                            if (ET.milliseconds() > 1500) {
                                GateServo.setPosition(ClosingGatePosition);
                                BucketControl.SetTargetPosition(2);
                            }
                        }
                        else if (BucketControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    break;

                case 7:
                    if (left) {
                        if (ArmControl.GetTaskState() == Task_State.READY) {
                            ArmControl.SetTargetPosition(120, 0.001, 0.001);
                        }
                        else if (ArmControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    else if (center) {
                        if (ArmControl.GetTaskState() == Task_State.READY) {
                            ArmControl.SetTargetPosition(120, 0.001, 0.001);
                        }
                        else if (ArmControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    else if (right) {
                        if (ArmControl.GetTaskState() == Task_State.READY) {
                            ArmControl.SetTargetPosition(120, 0.001, 0.001);
                        }
                        else if (ArmControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                        }
                    }
                    break;

                case 8:
                    if (left) {
                        if (ArmControl.GetTaskState() == Task_State.READY) {
                            ArmControl.SetTargetPosition(-3, -0.1, 0.1);
                        }
                        else if (ArmControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                            ET.reset();
                        }
                    }
                    else if (center) {
                        if (ArmControl.GetTaskState() == Task_State.READY) {
                            ArmControl.SetTargetPosition(-3, -0.1, 0.1);
                        }
                        else if (ArmControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                            ET.reset();
                        }
                    }
                    else if (right) {
                        if (ArmControl.GetTaskState() == Task_State.READY) {
                            ArmControl.SetTargetPosition(-3, -0.1, 0.1);
                        }
                        else if (ArmControl.GetTaskState() == Task_State.DONE) {
                            programorder1++;
                            ET.reset();
                        }
                    }
                    break;

                case 9:

                    if (ET.milliseconds() > 1000) {
                        programorder1++;
                    }
                    break;

                case 10:
                    Rail.setTargetPosition(300);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    programorder1++;
                    break;

                case 11:
                    if (MechDrive.GetTaskState() == Task_State.READY) {
                        MechDrive.SetTargets(-90, 610, 0.5);
                    }
                    else if (MechDrive.GetTaskState() == Task_State.DONE) {
                        programorder1++;
                    }
                    break;

                case 12:
                    if (MechDrive.GetTaskState() == Task_State.READY) {
                        MechDrive.SetTargets(0, 1450, 0.5);
                    }
                    else if (MechDrive.GetTaskState() == Task_State.DONE) {
                        programorder1++;
                    }
                    break;

                case 13:
                    CarouselMotor.setPower(-1);
                    programorder1++;
                    ET.reset();
                    break;

                case 14:
                    if (ET.milliseconds() > 1550) {
                        programorder1++;
                    }
                    break;

                case 15:
                    CarouselMotor.setPower(0);
                    programorder1++;
                    break;

                case 16:
                    if (MechDrive.GetTaskState() == Task_State.READY) {
                        MechDrive.SetTargets(90, 990, 0.5);
                    }
                    else if (MechDrive.GetTaskState() == Task_State.DONE) {
                        programorder1++;
                    }
                    break;

                case 17:
                    if (MechDrive.GetTaskState() == Task_State.READY) {
                        MechDrive.SetTargets(0, 200, 0.5);
                    }
                    else if (MechDrive.GetTaskState() == Task_State.DONE) {
                        programorder1++;
                    }
                    break;

                default:
                    break;

            }

            // THIS IS THE PART OF THE PROGRAM THAT IS REPETITIVE
            // THEY ARE CALLED 'BACKGROUND TASKS" BUT FOR SIMPLICITY, WE SHALL CALL THEM 'TASKS'

            // Detect for objects in the bucket
            BucketObjectColorDetector();

            if (BucketIsEmpty) {

                if (yellow) {
                    Intake.setPower(0);
                    ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    IntakeServo.setPosition(ClosingIntakePosition);
                    BucketIsEmpty = false;
                }
                else if (white) {
                    Intake.setPower(0);
                    ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                    IntakeServo.setPosition(ClosingIntakePosition);
                    BucketIsEmpty = false;
                }
                else if (unknown) {
                    ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
            }
            else {
                /* If intake is enabled, just assume bucket is empty */
                if (Intake.getPower() >= 0.9) {
                    BucketIsEmpty = true;
                }
            }

            MechDrive.Task(GyroContinuity());
            BucketControl.BucketTask();
            ArmControl.ArmTask();
        }
    }

    public static class BarcodeDeterminationPipeline extends OpenCvPipeline
    {
        /*
         * An enum to define the skystone position
         */
        public enum ShippingElementPosition
        {
            NONE,
            LEFT,
            CENTER,
            RIGHT
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar YELLOW = new Scalar(255, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(115,125);
        static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(600,165);
        static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1100,205);
        static final int REGION_WIDTH = 75;
        static final int REGION_HEIGHT = 75;

        static final int SHIPPING_ELEMENT_THRESHOLD = 100;

        Telemetry telemetry_vision;

        /*
         * Points which actually define the sample region rectangles, derived from above values
         *
         * Example of how points A and B work to define a rectangle
         *
         *   ------------------------------------
         *   | (0,0) Point A                    |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                                  |
         *   |                  Point B (70,50) |
         *   ------------------------------------
         *
         */
        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region2_pointA = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x,
                REGION2_TOPLEFT_ANCHOR_POINT.y);
        Point region2_pointB = new Point(
                REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
        Point region3_pointA = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x,
                REGION3_TOPLEFT_ANCHOR_POINT.y);
        Point region3_pointB = new Point(
                REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1, avg2, avg3;

        // Volatile since accessed by OpMode thread w/o synchronization
        //private volatile FarBlueOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition position = FarBlueOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition.LEFT;
        private volatile ShippingElementPosition position = ShippingElementPosition.NONE;

        /*
         * This function takes the RGB frame, converts to YCrCb,
         * and extracts the Cb channel to the 'Cb' variable
         */
        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat firstFrame)
        {
            /*
             * We need to call this in order to make sure the 'Cb'
             * object is initialized, so that the submats we make
             * will still be linked to it on subsequent frames. (If
             * the object were to only be initialized in processFrame,
             * then the submats would become delinked because the backing
             * buffer would be re-allocated the first time a real frame
             * was crunched)
             */
            inputToCb(firstFrame);

            /*
             * Submats are a persistent reference to a region of the parent
             * buffer. Any changes to the child affect the parent, and the
             * reverse also holds true.
             */
            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
            region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            /*
             * Overview of what we're doing:
             *
             * We first convert to YCrCb color space, from RGB color space.
             * Why do we do this? Well, in the RGB color space, chroma and
             * luma are intertwined. In YCrCb, chroma and luma are separated.
             * YCrCb is a 3-channel color space, just like RGB. YCrCb's 3 channels
             * are Y, the luma channel (which essentially just a B&W image), the
             * Cr channel, which records the difference from red, and the Cb channel,
             * which records the difference from blue. Because chroma and luma are
             * not related in YCrCb, vision code written to look for certain values
             * in the Cr/Cb channels will not be severely affected by differing
             * light intensity, since that difference would most likely just be
             * reflected in the Y channel.
             *
             * After we've converted to YCrCb, we extract just the 2nd channel, the
             * Cb channel. We do this because stones are bright yellow and contrast
             * STRONGLY on the Cb channel against everything else, including SkyStones
             * (because SkyStones have a black label).
             *
             * We then take the average pixel value of 3 different regions on that Cb
             * channel, one positioned over each stone. The brightest of the 3 regions
             * is where we assume the SkyStone to be, since the normal stones show up
             * extremely darkly.
             *
             * We also draw rectangles on the screen showing where the sample regions
             * are, as well as drawing a solid rectangle over top the sample region
             * we believe is on top of the SkyStone.
             *
             * In order for this whole process to work correctly, each sample region
             * should be positioned in the center of each of the first 3 stones, and
             * be small enough such that only the stone is sampled, and not any of the
             * surroundings.
             */

            /*
             * Get the Cb channel of the input frame after conversion to YCrCb
             */
            inputToCb(input);

            /*
             * Compute the average pixel value of each submat region. We're
             * taking the average of a single channel buffer, so the value
             * we need is at index 0. We could have also taken the average
             * pixel value of the 3-channel image, and referenced the value
             * at index 2 here.
             */
            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];

            /*
             * Draw a rectangle showing sample region 1 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 2 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region2_pointA, // First point which defines the rectangle
                    region2_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines

            /*
             * Draw a rectangle showing sample region 3 on the screen.
             * Simply a visual aid. Serves no functional purpose.
             */
            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region3_pointA, // First point which defines the rectangle
                    region3_pointB, // Second point which defines the rectangle
                    RED, // The color the rectangle is drawn in
                    4); // Thickness of the rectangle lines

            /*
             * Now that we found the max, we actually need to go and
             * figure out which sample region that value was from
             */

            DifferenceLeft = Avg1() - SHIPPING_ELEMENT_THRESHOLD;
            DifferenceCenter = Avg2() - SHIPPING_ELEMENT_THRESHOLD;
            DifferenceRight = Avg3() - SHIPPING_ELEMENT_THRESHOLD;

            if ((DifferenceLeft > -15) && (DifferenceLeft < 15)) { // Was it from region 1?

                position = ShippingElementPosition.LEFT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region1_pointA, // First point which defines the rectangle
                        region1_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        4); // Negative thickness means solid fill
            }

            else if ((DifferenceCenter > -15) && (DifferenceCenter < 15)) { // Was it from region 2?

                position = ShippingElementPosition.CENTER; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region2_pointA, // First point which defines the rectangle
                        region2_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        4); // Negative thickness means solid fill
            }

            else if ((DifferenceRight > -15) && (DifferenceRight < 15)) { // Was it from region 3?

                position = ShippingElementPosition.RIGHT; // Record our analysis

                /*
                 * Draw a solid rectangle on top of the chosen region.
                 * Simply a visual aid. Serves no functional purpose.
                 */
                Imgproc.rectangle(
                        input, // Buffer to draw on
                        region3_pointA, // First point which defines the rectangle
                        region3_pointB, // Second point which defines the rectangle
                        GREEN, // The color the rectangle is drawn in
                        4); // Negative thickness means solid fill
            }

            else {
                position = ShippingElementPosition.NONE;
            }

            telemetry_vision.addData("Avg1", Avg1());
            telemetry_vision.addData("Avg2", Avg2());
            telemetry_vision.addData("Avg3", Avg3());
            telemetry_vision.addData("Position", getAnalysis());
            telemetry_vision.update();

            /*
             * Render the 'input' buffer to the viewport. But note this is not
             * simply rendering the raw camera feed, because we called functions
             * to add some annotations to this buffer earlier up.
             */
            return input;
        }

        public int Avg1 () {
            return avg1;
        }

        public int Avg2 () {
            return avg2;
        }

        public int Avg3 () {
            return avg3;
        }

        /*
         * Call this from the OpMode thread to obtain the latest analysis
         */

        public ShippingElementPosition getAnalysis()
        {
            return position;
        }

        public void InitTelemetry(Telemetry Obj) {
            telemetry_vision = Obj;
        }
    }

    private void AttachmentSetDirection () {

        CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeServo.setDirection(Servo.Direction.FORWARD);
        GateServo.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        //Arm.setDirection(DcMotor.Direction.FORWARD);
        Rail.setDirection(DcMotor.Direction.FORWARD);
    }

    private int BucketObjectColorDetector() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = colorsensor.getNormalizedColors();
        colorsensor.setGain(30);

        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addData("H:", HSV[0]);
        telemetry.addData("S:", HSV[1]);
        telemetry.addData("V:", HSV[2]);

        int Yellow = 2;
        int White = 1;
        int Unkwown = 0;

        if (HSV[1] >= 0 && HSV[1] <= 0.45) {
            if (HSV[2] >= 0.3 && HSV[2] <= 1) {
                telemetry.addData("Color:", "White");
                telemetry.update();
                white = true;
                yellow = false;
                unknown = false;
                return White;
            } else {
                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                unknown = true;
                yellow = false;
                white = false;
                return Unkwown;
            }
        } else if (HSV[1] >= 0.5 && HSV[1] <= 0.8) {
            if (HSV[2] >= 0.6 && HSV[2] <= 1) {
                telemetry.addData("Color:", "Yellow");
                telemetry.update();
                yellow = true;
                white = false;
                unknown = false;
                return Yellow;
            } else {
                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                unknown = true;
                yellow = false;
                white = false;
                return Unkwown;
            }
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            unknown = true;
            yellow = false;
            white = false;
            return Unkwown;
        }
    }

    private double GyroContinuity() {

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        current_value = orientation.firstAngle;

        final_value = current_value - prev_value;

        if (final_value < -180)
            final_value += 360;
        else if (final_value > 180)
            final_value -= 360;

        globalangle += final_value;

        prev_value = current_value;

        return -globalangle;

    }

    private void SetMotorPower(double x) {

        FrontLeft.setPower(x);
        FrontRight.setPower(x);
        BackLeft.setPower(x);
        BackRight.setPower(x);

    }

    private void TopArmPosition () {

        Rail.setTargetPosition(1000);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setPower(0.5);

        if (Rail.getCurrentPosition() >= 970 && Rail.getCurrentPosition() <= 1030) {
            Arm.setTargetPosition(Top_Arm_Right);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.3);
        }
    }

    private void MiddleArmPosition () {

        Rail.setTargetPosition(750);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setPower(0.5);

        if (Rail.getCurrentPosition() >= 720 && Rail.getCurrentPosition() <= 780) {
            Arm.setTargetPosition(Middle_Arm_Right);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.3);
        }
    }

    private void LowArmPosition () {

        Rail.setTargetPosition(750);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setPower(0.5);

        if (Rail.getCurrentPosition() >= 720 && Rail.getCurrentPosition() <= 780) {
            Arm.setTargetPosition(Low_Arm_Right);
            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Arm.setPower(0.3);
        }
    }

}
