package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;

@TeleOp(name="Meet3Teleop", group="MecanumDrive")
public class Meet3Teleop extends LinearOpMode {

    static DcMotor FrontLeft;
    static DcMotor BackLeft;
    static DcMotor FrontRight;
    static DcMotor BackRight;
    static DcMotor Arm;
    static DcMotor Rail;
    static MoveDirection Direction;
    static DcMotor Intake;
    static CRServo CarouselLeft;
    static CRServo CarouselRight;
    static NormalizedColorSensor colorsensor;
    static RevBlinkinLedDriver ColorStrip;
    BNO055IMU IMU;
    static DcMotor Bucket;
    static Servo IntakeServo;
    static Servo GateServo;
    static Servo ElementServo;
    boolean top_level_event;
    boolean middle_level_event;
    boolean low_level_event;
    boolean mirror_event = false;
    //boolean reset_low_level_event;
    boolean barrier_event;
    boolean barrier_correction_event;
    boolean servo_left_event;
    boolean servo_default_event;
    boolean servo_power;
    ElapsedTime ET = new ElapsedTime();

    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_x_already_pressed = false;
    boolean button_y_already_pressed = false;
    boolean button_a_already_pressed2 = false;
    boolean button_b_already_pressed2 = false;
    boolean button_x_already_pressed2 = false;
    boolean button_y_already_pressed2 = false;
    boolean button_dpad_up_already_pressed = false;
    boolean button_dpad_down_already_pressed = false;
    boolean button_bumper_left_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;
    boolean button_right_trigger_already_pressed = false;
    boolean button_bumper_left_already_pressed2 = false;
    boolean button_bumper_right_already_pressed2 = false;
    boolean button_dpad_up_already_pressed2 = false;
    boolean button_dpad_down_already_pressed2 = false;
    boolean button_dpad_left_already_pressed2 = false;
    boolean button_dpad_right_already_pressed2 = false;
    boolean button_dpad_down_already_pressed3 = false;
    boolean button_left_trigger_already_pressed = false;
    boolean button_left_trigger_already_pressed2 = false;
    boolean button_right_trigger_already_pressed2 = false;

    boolean press1 = true;
    boolean press2 = false;
    boolean press3 = false;
    boolean press4 = false;

    boolean white;
    boolean yellow;
    boolean unknown;

    boolean BucketIsEmpty = true;

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

    Bucket_Control BucketMotor;
    Arm_Control ArmMotor;

    int bucketresetorder = 0;
    int triggerresetorder = 0;
    int lowhuborder = 0;
    int middlehuborder = 0;
    int tophuborder = 0;

    int servoseqleft = 0;
    int servoseqright = 0;
    double servoposition;

    int armorder = 0;
    int armposition;

    @Override
    public void runOpMode() {

        // Retrieve hardware object references
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        Rail = hardwareMap.get(DcMotor.class, "rail");
        CarouselLeft = hardwareMap.get(CRServo.class, "carouselleft");
        CarouselRight = hardwareMap.get(CRServo.class, "carouselright");
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
        ColorStrip = hardwareMap.get(RevBlinkinLedDriver.class, "colorstrip");
        Bucket = hardwareMap.get(DcMotor.class, "BucketMotor");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        GateServo = hardwareMap.get(Servo.class, "GateServo");
        ElementServo = hardwareMap.get(Servo.class, "ElementServo");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        AttachmentSetDirection();

        // Initialize all four drive motors
        SetDirection(MoveDirection.REVERSE);

        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize the rail controller (we will use the default DC motor controller for it)
        Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rail.setTargetPosition(0);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Initialize intake and outtake servos
        IntakeServo.scaleRange(0,1);
        GateServo.scaleRange(0,1);
        IntakeServo.setPosition(OpenIntakePosition);
        GateServo.setPosition(OpenGatePosition);

        // Construct controllers for the bucket and arm (we will use our own controllers instead of the default one)
        BucketMotor = new Bucket_Control(Bucket);
        ArmMotor = new Arm_Control(Arm);

        waitForStart();

        while (opModeIsActive()) {

            /****************************************
             Bucket object sensing
             ***************************************/

            WhiteYellowDetector();

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

            /****************************************
             Robot Movement (Mecanum Drive)
             ***************************************/

            double y = -gamepad1.left_stick_y * 0.65;
            //double x = gamepad1.left_stick_x * 0.55;
            double x = gamepad1.left_stick_x * 0.65;
            double rx = gamepad1.right_stick_x * 0.65;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx) / denominator;
            double BLPower = (y - x + rx) / denominator;
            double FRPower = (y - x - rx) / denominator;
            double BRPower = (y + x - rx) / denominator;

            FrontLeft.setPower(FLPower);
            BackLeft.setPower(BLPower);
            FrontRight.setPower(FRPower);
            BackRight.setPower(BRPower);

            /*********************************************************************
             Dpad Up - Trigger the final bucket reset sequence and run the intake
             *********************************************************************/
            if (button_dpad_up_already_pressed == false) {
                if (gamepad1.dpad_up) {
                    triggerresetorder = 1;
                    button_dpad_up_already_pressed = true;
                }
            }
            else {
                if (!gamepad1.dpad_up) {
                    button_dpad_up_already_pressed = false;
                }
            }

            /******************************
             * Dpad Down - Stop the intake
             ******************************/
            if (gamepad1.dpad_down) {
                 Intake.setPower(0);
            }

            // THIS IS THE SEQUENCE MANAGER TO COMPLETE THE BUCKET RESET PROCEDURE AND START THE INTAKE
            switch (triggerresetorder) {
                case 1:
                    // Let the arm hang loosely while the rail moves down
                    // This should get the arm back to its zero position
                    ArmMotor.Override();
                    Rail.setTargetPosition(400);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    triggerresetorder++;
                    break;

                case 2:
                    // Once the bucket is inside the robot, move it back to its zero position
                    // We will also move the arm back to it zero position
                    if (Rail.getCurrentPosition() > 370 && Rail.getCurrentPosition() < 430) {

                        if (BucketMotor.GetTaskState() == Task_State.READY) {

                            BucketMotor.SetTargetPosition(OriginalBucketPosition);

                            if (mirror_event) {
                                ArmMotor.SetTargetPosition(0, -0.5, 0.5);
                            }
                            else {
                                ArmMotor.SetTargetPosition(-5, -0.5, 0.5);
                            }
                            triggerresetorder++;
                            mirror_event = false;
                        }
                    }
                    break;
                case 3:
                    // We will now allow the bucket and arm motors to be calibrated
                    // At the same time, we will also run the intake
                    if (BucketMotor.GetTaskState() == Task_State.DONE) {
                        // Open the intake gate and turn on the intake
                        BucketMotor.Calibrate();
                        ArmMotor.Calibrate();
                        IntakeServo.setPosition(OpenIntakePosition);
                        Intake.setPower(1);

                        // Move the rail down and allow the bucket to hang loosely
                        Rail.setTargetPosition(0);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.5);
                        triggerresetorder++;
                    }
                default:
                    break;
            }


            /****************************************
             Run CarouselMotor
             ***************************************/
            //if (gamepad1.dpad_right) {
                //CarouselMotor.setPower(1.0);
            //}
            //if (gamepad1.dpad_left) {
                //CarouselMotor.setPower(-1.0);
            //}
            //if (gamepad1.a) {
                //CarouselMotor.setPower(0);
            //}

            /***********************************************************
             Button Y - Top Level (For Opposite, press dpad_down first)
             ***********************************************************/
            if (button_dpad_down_already_pressed2 == false) {
                if (gamepad2.dpad_down) {
                    button_dpad_down_already_pressed2 = true;
                    mirror_event = true;
                }
            } else {
                if (!gamepad2.dpad_down) {
                    button_dpad_down_already_pressed2 = false;
                }
            }

            if (button_y_already_pressed2 == false) {
                if (gamepad2.y) {
                    tophuborder = 1;
                    button_y_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.y) {
                    button_y_already_pressed2 = false;
                }
            }

            // SEQUENCE MANAGER TO DUMP ONTO THE TOP LEVEL OF THE ALLIANCE HUB
            switch (tophuborder) {

                case 1:
                    // Always check for INIT OR READY the first time
                    if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {

                        // Lock the bucket in the zero position first before raising the rail or arm to prevent it from falling
                        BucketMotor.SetTargetPosition(0);
                    }
                    else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                        tophuborder++;
                    }

                    // Close the intake and outtake gates to prevent the object from falling out of the bucket
                    GateServo.setPosition(ClosingGatePosition);
                    IntakeServo.setPosition(ClosingIntakePosition);
                    break;

                case 2:

                    // Move the rail to its highest position
                    Rail.setTargetPosition(1030);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);

                    if (Rail.getCurrentPosition() >= 1000 && Rail.getCurrentPosition() <= 1060) {
                        tophuborder++;
                    }
                    break;

                case 3:

                    // Move the arm to its highest position
                    if (mirror_event) {
                        if (ArmMotor.GetTaskState() == Task_State.INIT || ArmMotor.GetTaskState() == Task_State.READY) {

                            ArmMotor.SetTargetPosition(430, -0.7, 0.7);
                        } else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            tophuborder++;
                        }
                    } else {
                        if (ArmMotor.GetTaskState() == Task_State.INIT || ArmMotor.GetTaskState() == Task_State.READY) {

                            ArmMotor.SetTargetPosition(-430, -0.65, 0.65);
                        }
                        else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            tophuborder++;
                        }
                    }
                    break;

                case 4:

                    // Tip the bucket to get it ready to dump the object out
                    if (mirror_event) {
                        if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {

                            BucketMotor.SetTargetPosition(MirrorTopBucketPosition);
                        }
                        else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                            tophuborder++;
                        }
                    }
                    else {
                        if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {

                            BucketMotor.SetTargetPosition(TopBucketPosition);
                        }
                        else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                            tophuborder++;
                        }
                    }
                    break;

                default:
                    break;
            }

            /***************************************************
             Button X - Attachment Reset (Intermediate position
             ***************************************************/
            if (button_x_already_pressed2 == false) {
                if (gamepad2.x) {
                    bucketresetorder = 1;
                    button_x_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed2 = false;
                }
            }

            // SEQUENCE MANAGER TO TUCK THE ARM & BUCKET BACK INTO ITS INTERMEDIATE RESET POSITION
            switch (bucketresetorder) {

                case 1:

                    // Close the outtake gate first
                    GateServo.setPosition(ClosingGatePosition);
                    bucketresetorder++;
                    break;

                case 2:

                    // Deliberately tip the bucket at an angle so it doesn't hit the chassis when the rail moves down later
                    if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {

                        if (mirror_event) {
                            BucketMotor.SetTargetPosition(20);
                        }
                        else {
                            BucketMotor.SetTargetPosition(-20);
                        }
                    }
                    else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                        bucketresetorder++;
                    }
                    break;

                case 3:

                    // Let gravity bring the arm down
                    // But use a very small amount of opposite force to ensure the arm's fall is not completely free fall
                    // We do this by clipping the command from the arm control
                    if (ArmMotor.GetTaskState() == Task_State.INIT ||
                            ArmMotor.GetTaskState() == Task_State.READY) {

                        if (mirror_event) {
                            //ArmMotor.SetTargetPosition(120, 0.001, 0.01);
                            //ArmMotor.SetTargetPosition(120, 0.001, 0.001);
                            ArmMotor.SetTargetPosition(80, 0.00002, 0.00002);
                        }
                        else {
                            //ArmMotor.SetTargetPosition(-120, -0.6, -0.0001);
                            ArmMotor.SetTargetPosition(-80, -0.00002, -0.00002);
                        }

                    }
                    else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                        bucketresetorder++;
                    }
                    break;

                case 4:

                    // Once the arm drops to a lower position, we will clip the motor command with a larger opposite force
                    // This will allow the motor to "brake" the arm as it continues to drop
                    // The idea here is to catch the arm to prevent it swinging wildly non-stop
                    if (ArmMotor.GetTaskState() == Task_State.INIT ||
                            ArmMotor.GetTaskState() == Task_State.READY) {

                        if (mirror_event) {
                            //ArmMotor.SetTargetPosition(-2, -0.105, 0.6);
                            //ArmMotor.SetTargetPosition(-10, -0.5, 0.5);
                            ArmMotor.SetTargetPosition(0, 0.1, 0.1);
                        }
                        else {
                            //ArmMotor.SetTargetPosition(2, -0.6, 0.2);
                            //ArmMotor.SetTargetPosition(10, -0.5, 0.5);
                            ArmMotor.SetTargetPosition(0, -0.1, -0.1);
                        }

                    }
                    else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                        bucketresetorder++;
                    }
                    break;

                case 5:
                    // Once the arm stabilizes, we will give the arm control a larger power range to allow
                    // it to bring the arm to its desired position. Since we deliberately angled the bucket earlier,
                    // we may deliberately set a target position to overshoot the arm's zero position so that
                    // when the rail lowers later, the bottom of the bucket won't hit the robot's chassis
                    if (ArmMotor.GetTaskState() == Task_State.INIT ||
                            ArmMotor.GetTaskState() == Task_State.READY) {

                        if (mirror_event) {
                            //ArmMotor.SetTargetPosition(-2, -0.105, 0.6);
                            ArmMotor.SetTargetPosition(-10, -0.5, 0.5);
                        }
                        else {
                            //ArmMotor.SetTargetPosition(2, -0.6, 0.2);
                            ArmMotor.SetTargetPosition(0, -0.5, 0.5);
                        }

                    }
                    else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                        bucketresetorder++;
                    }
                    break;
                default:
                    break;
            }


            /****************************************
             Right Bumper - Open Gate
             ***************************************/

            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper) {
                    GateServo.setPosition(OpenGatePosition);
                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                }
            }

            /*****************************************************************
             * Left Bumper - Auto Calibration (To recalibrate bucket and arm)
             *****************************************************************/

            // Press and hold the left bumper to raise the rail slightly
            // The bucket control will automatically straighten the bucket to its zero position
            // The arm control will automatically straighten the bucket to its zero position
            // Then release the left bumper to lower the rail
            // The arm and bucket motor encoders will be reset
            if (button_bumper_left_already_pressed2 == false) {

                if (gamepad2.left_bumper) {

                    Rail.setTargetPosition(800);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    ArmMotor.SetTargetPosition(0, -0.5, 0.5);
                    BucketMotor.SetTargetPosition(0);
                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                    BucketMotor.Calibrate();
                    ArmMotor.Calibrate();
                    Rail.setTargetPosition(0);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                }
            }

            /***************************************
             * Dpad Left - Lift shipping element arm
             ***************************************/

            if (button_dpad_left_already_pressed2 == false) {
                if (gamepad2.dpad_left) {
                    servoseqleft = 1;

                    button_dpad_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_left) {
                    button_dpad_left_already_pressed2 = false;
                    servoseqleft = 0;
                }
            }

            // SEQUENCE MANAGER FOR LIFTING THE SHIPPING ELEMENT ARM
            switch (servoseqleft) {

                case 1:
                    servoposition = servoposition - 0.003;
                    servoposition = Range.clip(servoposition, 0, 1);
                    ElementServo.setPosition(servoposition);
                    break;

                default:
                    break;

            }

            /******************************************
             * Dpad Right - Lower shipping element arm
             ******************************************/

            if (button_dpad_right_already_pressed2 == false) {
                if (gamepad2.dpad_right) {

                    servoseqright = 1;

                    button_dpad_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_right) {
                    servoseqright = 0;
                    button_dpad_right_already_pressed2 = false;
                }
            }

            // SEQUENCE MANAGER FOR LOWERING THE SHIPPING ELEMENT ARM
            switch (servoseqright) {

                case 1:
                    servoposition = servoposition + 0.003;
                    servoposition = Range.clip(servoposition, 0, 1);
                    ElementServo.setPosition(servoposition);
                    break;

                default:
                    break;

            }

            // BACKGROUND TASKS FOR THE ARM AND MOTOR CONTROL
            ArmMotor.ArmTask();
            BucketMotor.BucketTask();

            //telemetry.addData("Rail Current Position", Rail.getCurrentPosition());
            //telemetry.addData("encoder", FrontRight.getCurrentPosition());
            //telemetry.addData("encoder", BackRight.getCurrentPosition());
            //telemetry.addData("Original bucket position", OriginalBucketPosition);
            //telemetry.addData("High bucket position", TopBucketPosition);
            //telemetry.addData("Middle bucket position", MiddleBucketPosition);
            //telemetry.addData("TopHub Sequence", tophuborder);
            //telemetry.addData("BucketResetSequence", bucketresetorder);
            telemetry.addData("ArmPower", Arm.getPower());
            telemetry.addData("Arm Current Position", Arm.getCurrentPosition());
            telemetry.addData("BucketPower", Bucket.getPower());
            telemetry.addData("bucket position", Bucket.getCurrentPosition());
            telemetry.addData("rail", Rail.getCurrentPosition());
            //telemetry.addData("mirror event", mirror_event);
            //telemetry.addData("task state", ArmMotor.GetTaskState());
            //telemetry.addData("vibration Y", IMU.getLinearAcceleration().yAccel);
            //telemetry.addData("vibration X", IMU.getLinearAcceleration().xAccel);
            telemetry.update();

        }
    }

    private void SetDirection (MoveDirection direction){

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

    private void AttachmentSetDirection () {

        CarouselRight.setDirection(DcMotorSimple.Direction.FORWARD);
        CarouselLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        IntakeServo.setDirection(Servo.Direction.FORWARD);
        GateServo.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Rail.setDirection(DcMotor.Direction.FORWARD);

    }

    private int WhiteYellowDetector() {

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

}