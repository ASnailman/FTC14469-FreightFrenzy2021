package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name="AutonomousTest", group="MecanumDrive")
public class Autonomous_Test extends LinearOpMode {

        //OpenCvInternalCamera phoneCam;
        OpenCvWebcam webcam;
        SpectatorRedOpenCV.SkystoneDeterminationPipeline pipeline;
        static int DifferenceLeft;
        static int DifferenceCenter;
        static int DifferenceRight;
        static boolean BarcodeLeft;
        static boolean BarcodeCenter;
        static boolean BarcodeRight;

    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor Arm;
    static DcMotor Rail;
    static DcMotor Intake;
    static DcMotor CarouselMotor;
    static NormalizedColorSensor colorsensor;
    static RevBlinkinLedDriver ColorStrip;
    static Servo BucketServo;
    static Servo IntakeServo;
    static Servo GateServo;
    static MoveDirection Direction;
    static MoveDirection DiagDirection;
    static DistanceSensor DistancesensorForward;
    static DistanceSensor DistancesensorRight;
    static ColorSensor Colorsensor;
    static Servo BackServo;
    BNO055IMU IMU;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    BNO055IMU ACCIMU;
    Orientation orientation;
    double globalangle;
    double current_value;
    double prev_value = 0;
    double final_value;
    double FRpower;
    double FLpower;
    double BRpower;
    double BLpower;
    double SteeringOutput;
    double power_x_old;
    double power_y_old;
    ElapsedTime ET = new ElapsedTime();
    boolean servo_power;
    byte AXIS_MAP_SIGN_BYTE = 0x6; //rotates control hub 180 degrees around z axis by negating x and y signs
    byte AXIS_MAP_CONFIG_BYTE = 0x6; //rotates control hub 90 degrees around y axis by swapping x and z axis

    boolean top_level_event;
    boolean middle_level_event;
    boolean low_level_event;
    boolean barrier_event;

    static final int Top_Arm_Left = -350;
    static final int Top_Arm_Right = 350;

    static final int Middle_Arm_Left = -250;
    static final int Middle_Arm_Right = 250;

    static final int Low_Arm_Left = -150;
    static final int Low_Arm_Right = 150;

    static final double OriginalBucketPosition = 0.52;

    static final double TopBucketPosition = 0.18;
    static final double MirrorTopBucketPosition = 0.88;

    static final double MiddleBucketPosition = 0.22;
    static final double MirrorMiddleBucketPosition = 0.82;

    static final double LowBucketPosition = 0.28;
    static final double MirrorLowBucketPosition = 0.76;

    static final double OpenGatePosition = 0.5;
    static final double OpenIntakePosition = 0.6;
    static final double ClosingGatePosition = 0.2;
    static final double ClosingIntakePosition = 0.8;

    boolean BucketIsEmpty = true;
    boolean white;
    boolean yellow;
    boolean unknown;

        @Override
        public void runOpMode() {

            BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
            BackRight = hardwareMap.get(DcMotor.class, "BackRight");
            FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
            FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
            Intake = hardwareMap.get(DcMotor.class, "Intake");
            Arm = hardwareMap.get(DcMotor.class, "arm");
            Rail = hardwareMap.get(DcMotor.class, "rail");
            CarouselMotor = hardwareMap.get(DcMotor.class, "carouselmotor");
            colorsensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
            ColorStrip = hardwareMap.get(RevBlinkinLedDriver.class, "colorstrip");
            BucketServo = hardwareMap.get(Servo.class, "BucketServo");
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
            SetDirection(MoveDirection.REVERSE);
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            IMU.initialize(parameters);

            orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            globalangle = 0;
            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Arm.setTargetPosition(0);
            Rail.setTargetPosition(0);

            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            BucketServo.scaleRange(0,1);
            IntakeServo.scaleRange(0,1);
            GateServo.scaleRange(0,1);

            BucketServo.setPosition(OriginalBucketPosition);
            IntakeServo.setPosition(OpenIntakePosition);
            GateServo.setPosition(ClosingGatePosition);

            /**
             * NOTE: Many comments have been omitted from this sample for the
             * sake of conciseness. If you're just starting out with EasyOpenCv,
             * you should take a look at {@link InternalCamera1Example} or its
             * webcam counterpart, {@link WebcamExample} first.
             */

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            pipeline = new SpectatorRedOpenCV.SkystoneDeterminationPipeline();
            webcam.setPipeline(pipeline);

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

            while (opModeIsActive())
            {
                telemetry.addData("Analysis", pipeline.getAnalysis());
                telemetry.addData("Avg1", pipeline.Avg1());
                telemetry.addData("Avg2", pipeline.Avg2());
                telemetry.addData("Avg3", pipeline.Avg3());
                telemetry.update();

                // Don't burn CPU cycles busy-looping in this sample
                sleep(50);
            }
        }

        public static class SkystoneDeterminationPipeline extends OpenCvPipeline
        {
            /*
             * An enum to define the skystone position
             */
            public enum ShippingElementPosition
            {
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
            static final Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(625,105);
            static final Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(1175,85);
            static final int REGION_WIDTH = 75;
            static final int REGION_HEIGHT = 75;

            static final int SHIPPING_ELEMENT_THRESHOLD = 55;

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
            private volatile SpectatorRedOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition position = SpectatorRedOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition.LEFT;

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
                if ((DifferenceLeft > -30) && (DifferenceLeft < 30)) { // Was it from region 1?
                    BarcodeLeft = true;
                    position = SpectatorRedOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition.LEFT; // Record our analysis
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
                } else {
                    BarcodeCenter = false;
                    BarcodeRight = false;
                }

                DifferenceCenter = Avg2() - SHIPPING_ELEMENT_THRESHOLD;
                if ((DifferenceCenter > -30) && (DifferenceCenter < 30)) { // Was it from region 2?
                    BarcodeCenter = true;
                    position = SpectatorRedOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition.CENTER; // Record our analysis
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
                } else {
                    BarcodeLeft = false;
                    BarcodeRight = false;
                }

                DifferenceRight = Avg3() - SHIPPING_ELEMENT_THRESHOLD;
                if ((DifferenceRight > -30) && (DifferenceRight < 30)) { // Was it from region 3?
                    BarcodeRight = true;
                    position = SpectatorRedOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition.RIGHT; // Record our analysis
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
                } else {
                    BarcodeLeft = false;
                    BarcodeCenter = false;
                }

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

            public SpectatorRedOpenCV.SkystoneDeterminationPipeline.ShippingElementPosition getAnalysis()
            {
                return position;
            }
        }

    private void AttachmentSetDirection () {

        CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        BucketServo.setDirection(Servo.Direction.FORWARD);
        IntakeServo.setDirection(Servo.Direction.FORWARD);
        GateServo.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Rail.setDirection(DcMotor.Direction.FORWARD);

    }

    private void SetDirection (MoveDirection direction) {

        Direction = direction;

        if (Direction == MoveDirection.FORWARD) {
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
            BackRight.setDirection(DcMotor.Direction.FORWARD);
        } else if (Direction == MoveDirection.REVERSE) {
            FrontLeft.setDirection(DcMotor.Direction.FORWARD);
            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.FORWARD);
            BackRight.setDirection(DcMotor.Direction.REVERSE);
        }
    }

}
