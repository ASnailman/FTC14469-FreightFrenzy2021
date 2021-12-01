package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MecanumDriveTeleopV2", group="MecanumDrive")
public class OfficialTeleop_V2 extends LinearOpMode {

    static DcMotor FrontLeft;
    static DcMotor BackLeft;
    static DcMotor FrontRight;
    static DcMotor BackRight;
    static DcMotor Arm;
    static DcMotor Rail;
    static MoveDirection Direction;
    static DcMotor Intake;
    static DcMotor CarouselMotor;
    static NormalizedColorSensor colorsensor;
    static RevBlinkinLedDriver ColorStrip;
    BNO055IMU IMU;
    static Servo BucketServo;
    static Servo IntakeServo;
    static Servo GateServo;
    boolean top_level_event;
    boolean middle_level_event;
    boolean low_level_event;
    //boolean reset_low_level_event;
    boolean barrier_event;
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

    boolean press1 = true;
    boolean press2 = false;
    boolean press3 = false;
    boolean press4 = false;

    boolean white;
    boolean yellow;
    boolean unknown;

    boolean BucketIsEmpty = true;

    static final double OriginalBucketPosition = 0.49;

    static final double TopBucketPosition = 0.1;
    static final double MirrorTopBucketPosition = 0.9;

    static final double MiddleBucketPosition = 0.1;
    static final double MirrorMiddleBucketPosition = 0.9;

    static final double LowBucketPosition = 0.1;
    static final double MirrorLowBucketPosition = 0.9;

    static final double OpenGatePosition = 0.5;
    static final double OpenIntakePosition = 0.7;
    static final double ClosingGatePosition = 0.3;
    static final double ClosingIntakePosition = 1;

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

        //BucketCRServoCtrl_Thread cr_thread = new BucketCRServoCtrl_Thread();
        //cr_thread.start();

        AttachmentSetDirection();
        SetDirection(MoveDirection.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm.setTargetPosition(0);
        Rail.setTargetPosition(0);

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BucketServo.scaleRange(0,1);
        IntakeServo.scaleRange(0,1);
        GateServo.scaleRange(0,1);

        BucketServo.setPosition(OriginalBucketPosition);
        IntakeServo.setPosition(OpenIntakePosition);
        GateServo.setPosition(OpenGatePosition);

        waitForStart();

        ET.reset();

        while (opModeIsActive()) {

            /****************************************
             Yellow & White Sensing
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
             Movement
             ***************************************/

            double y = -gamepad1.left_stick_y * 0.63;
            //double x = gamepad1.left_stick_x * 0.55;
            double x = gamepad1.left_stick_x * 0.64;
            double rx = gamepad1.right_stick_x * 0.52;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx) / denominator;
            double BLPower = (y - x + rx) / denominator;
            double FRPower = (y - x - rx) / denominator;
            double BRPower = (y + x - rx) / denominator;

            FrontLeft.setPower(FLPower);
            BackLeft.setPower(BLPower);
            FrontRight.setPower(FRPower);
            BackRight.setPower(BRPower);

            /****************************************
             Run Intake
             ***************************************/

            if (gamepad1.dpad_up) {
                IntakeServo.setPosition(OpenIntakePosition);
                Intake.setPower(1);
                Rail.setTargetPosition(0);
                Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Rail.setPower(0.5);
            }
            if (gamepad1.dpad_down) {
                Intake.setPower(0);
            }

            /****************************************
             Run CarouselMotor
             ***************************************/

            if (gamepad1.dpad_right) {
                CarouselMotor.setPower(0.8);
            }
            if (gamepad1.dpad_left) {
                CarouselMotor.setPower(-0.8);
            }
            if (gamepad1.x) {
                CarouselMotor.setPower(0);
            }

            /****************************************
             Top Level (For Opposite, press dpad_down first)
             ***************************************/

            if (button_y_already_pressed == false) {
                if (gamepad2.y) {
                    IntakeServo.setPosition(ClosingIntakePosition);
                    Rail.setTargetPosition(1000);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    ET.reset();
                    button_y_already_pressed = true;
                    top_level_event = true;
                } else {
                    if (top_level_event == true) {
                        if (ET.milliseconds() > 1000) {
                            if (gamepad2.dpad_down) {
                                Arm.setTargetPosition(350);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
                                BucketServo.setPosition(MirrorTopBucketPosition);
                                top_level_event = false;
                            } else {
                                Arm.setTargetPosition(-350);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
                                BucketServo.setPosition(TopBucketPosition);
                                top_level_event = false;
                            }
                        }
                    }
                }
            } else {
                if (!gamepad2.y) {
                    button_y_already_pressed = false;
                }
            }

            /****************************************
             Middle Level (For Opposite, press dpad_down first)
             ***************************************/

            if (button_b_already_pressed == false) {
                if (gamepad2.b) {
                    IntakeServo.setPosition(ClosingIntakePosition);
                    Rail.setTargetPosition(750);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    ET.reset();
                    button_b_already_pressed = true;
                    middle_level_event = true;
                } else {
                    if (middle_level_event == true) {
                        if (ET.milliseconds() > 1000) {
                            if (gamepad2.dpad_down) {
                                Arm.setTargetPosition(250);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
                                BucketServo.setPosition(MirrorMiddleBucketPosition);
                                middle_level_event = false;
                            } else {
                                Arm.setTargetPosition(-250);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
                                BucketServo.setPosition(MiddleBucketPosition);
                                middle_level_event = false;
                            }
                        }
                    }
                }
            } else {
                if (!gamepad2.b) {
                    button_b_already_pressed = false;
                }
            }

            /****************************************
             Low Level (For Opposite, press dpad_down first)
             ***************************************/

            if (button_a_already_pressed == false) {
                if (gamepad2.a) {
                    IntakeServo.setPosition(ClosingIntakePosition);
                    Rail.setTargetPosition(750);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    ET.reset();
                    button_a_already_pressed = true;
                    low_level_event = true;
                } else {
                    if (low_level_event == true) {
                        if (ET.milliseconds() > 1000) {
                            if (gamepad2.dpad_down) {
                                Arm.setTargetPosition(150);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
                                BucketServo.setPosition(MirrorLowBucketPosition);
                                low_level_event = false;
                            } else {
                                Arm.setTargetPosition(-150);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
                                BucketServo.setPosition(LowBucketPosition);
                                low_level_event = false;
                            }
                        }
                    }
                }
            } else {
                if (!gamepad2.a) {
                    button_a_already_pressed = false;
                }
            }

            /****************************************
             Reset Position For Barrier Passing
             ***************************************/

            if (button_x_already_pressed == false) {
                if (gamepad2.x) {
                    GateServo.setPosition(ClosingGatePosition);
                    BucketServo.setPosition(OriginalBucketPosition);
                    button_x_already_pressed = true;
                    barrier_event = true;
                }
                else {
                    if (barrier_event == true) {
                        if (BucketServo.getPosition() <= OriginalBucketPosition) {
                            Arm.setTargetPosition(10);
                            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Arm.setPower(0.2);
                        }
                        if (Arm.getCurrentPosition() <= 10) {
                            Rail.setTargetPosition(300);
                            Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Rail.setPower(0.5);
                            barrier_event = false;
                        }
                    }
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed = false;
                }
            }

            /****************************************
             Default Position For Barrier Passing (Start of Program)
             ***************************************/

            if (button_dpad_up_already_pressed == false) {
                if (gamepad2.dpad_up) {
                    Rail.setTargetPosition(300);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    button_dpad_up_already_pressed = true;
                }
            } else {
                if (!gamepad2.dpad_up) {
                    button_dpad_up_already_pressed = false;
                }
            }

            /****************************************
             Open Gate
             ***************************************/

            if (button_dpad_right_already_pressed2 == false) {
                if (gamepad2.dpad_right) {
                    GateServo.setPosition(OpenGatePosition);
                    button_dpad_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_right) {
                    button_dpad_right_already_pressed2 = false;
                }
            }

            /********************************
             * Calibrate rail zero position
             ********************************/
            if (button_right_trigger_already_pressed == false) {
                if (gamepad1.right_trigger > 0) {
                    Rail.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Rail.setPower(0);
                    ET.reset();
                    while (ET.milliseconds() < 1000) {

                    }

                    Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //Rail.setTargetPosition(0);
                    //Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //Rail.setPower(0.5);
                    button_right_trigger_already_pressed = true;
                }
            }
            else {
                if (gamepad1.right_trigger == 0) {
                    button_right_trigger_already_pressed = false;
                }
            }

            /****************************
             Shipping Element PLacement Right Bumper
             ****************************/

            //first press

            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper && press1 && !press2 && !press3 && !press4) {
                    Rail.setTargetPosition(750);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    ET.reset();
                        if (ET.milliseconds() > 1000) {
                                Arm.setTargetPosition(150);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
                        }
                    BucketServo.setPosition(1);
                    button_bumper_right_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                    press1 = false;
                    press2 = true;
                    press3 = false;
                    press4 = false;
                }
            }

            //second press

            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper && !press1 && press2 && !press3 && !press4) {
                    Rail.setTargetPosition(1000);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    if (ET.milliseconds() > 1000) {
                            Arm.setTargetPosition(450);
                            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Arm.setPower(0.2);
                    }
                    button_bumper_right_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                    press1 = false;
                    press2 = false;
                    press3 = true;
                    press4 = false;
                }
            }

            //third press

            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper && !press1 && !press2 && press3 && !press4) {
                    Arm.setTargetPosition(400);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(0.2);
                    button_bumper_right_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                    press1 = false;
                    press2 = false;
                    press3 = false;
                    press4 = true;
                }
            }

            //fourth press

            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper && !press1 && !press2 && !press3 && press4) {
                    BucketServo.setPosition(0.5);
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(0.2);
                    ET.reset();
                    barrier_event = true;
                    if (barrier_event == true) {
                        if (ET.milliseconds() > 1000) {
                            Rail.setTargetPosition(300);
                            Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Rail.setPower(0.5);
                            barrier_event = false;
                        }
                    }
                    button_bumper_right_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                    press1 = true;
                    press2 = false;
                    press3 = false;
                    press4 = false;
                }
            }

            /****************************
             Shipping Element PLacement Left Bumper
             ****************************/

            //first press

            if (button_bumper_left_already_pressed2 == false) {
                if (gamepad2.left_bumper && press1 && !press2 && !press3 && !press4) {
                    Rail.setTargetPosition(750);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    ET.reset();
                    if (ET.milliseconds() > 1000) {
                        Arm.setTargetPosition(-150);
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Arm.setPower(0.2);
                    }
                    BucketServo.setPosition(0);
                    button_bumper_left_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                    press1 = false;
                    press2 = true;
                    press3 = false;
                    press4 = false;
                }
            }

            //second press

            if (button_bumper_left_already_pressed2 == false) {
                if (gamepad2.left_bumper && !press1 && press2 && !press3 && !press4) {
                    Rail.setTargetPosition(1000);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    if (ET.milliseconds() > 1000) {
                        Arm.setTargetPosition(-450);
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Arm.setPower(0.2);
                    }
                    button_bumper_left_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                    press1 = false;
                    press2 = false;
                    press3 = true;
                    press4 = false;
                }
            }

            //third press

            if (button_bumper_left_already_pressed2 == false) {
                if (gamepad2.left_bumper && !press1 && !press2 && press3 && !press4) {
                    Arm.setTargetPosition(-400);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(0.2);
                    button_bumper_left_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                    press1 = false;
                    press2 = false;
                    press3 = false;
                    press4 = true;
                }
            }

            //fourth press

            if (button_bumper_left_already_pressed2 == false) {
                if (gamepad2.left_bumper && !press1 && !press2 && !press3 && press4) {
                    BucketServo.setPosition(0.5);
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(0.2);
                    ET.reset();
                    barrier_event = true;
                    if (barrier_event == true) {
                        if (ET.milliseconds() > 1000) {
                            Rail.setTargetPosition(300);
                            Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Rail.setPower(0.5);
                            barrier_event = false;
                        }
                    }
                    button_bumper_left_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                    press1 = true;
                    press2 = false;
                    press3 = false;
                    press4 = false;
                }
            }

            telemetry.addData("Arm Current Position", Arm.getCurrentPosition());
            telemetry.addData("Rail Current Position", Rail.getCurrentPosition());
            telemetry.addData("encoder", FrontRight.getCurrentPosition());
            telemetry.addData("encoder", BackRight.getCurrentPosition());
            telemetry.addData("FLPower", FLPower);
            telemetry.addData("BLPower", BLPower);
            telemetry.addData("FRPower", FRPower);
            telemetry.addData("BRPower", BRPower);
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

        CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        BucketServo.setDirection(Servo.Direction.FORWARD);
        IntakeServo.setDirection(Servo.Direction.FORWARD);
        GateServo.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Rail.setDirection(DcMotor.Direction.FORWARD);

    }

    private class BucketCRServoCtrl_Thread extends Thread {

        ElapsedTime ET;
        boolean run_state = Boolean.FALSE;
        double run_duration;
        DcMotorSimple.Direction run_direction;
        double run_speed;

        public void run() {

            ET = new ElapsedTime();
            ET.reset();

            try {
                while (!isInterrupted()) {

                    if (run_state == Boolean.TRUE) {

                        //BucketServo.setDirection(run_direction);
                        //BucketServo.setPower(run_speed);

                        if (ET.milliseconds() >= run_duration) {
                            //BucketServo.setPower(0);
                            run_state = Boolean.FALSE;
                            ET.reset();
                        }
                    }
                    else {
                        ET.reset();
                    }
                }
            } catch (Exception e) {
                telemetry.addLine("CR Servo Control thread crashed");
                telemetry.update();
            }
        }

        void SetRunState(boolean state) {

            run_state = state;
        }

        void SetDuration(double duration) {

            run_duration = duration;
        }

        void SetRunDirection(DcMotorSimple.Direction direction) {

            run_direction = direction;
        }

        void SetRunSpeed(double speed) {

            run_speed = speed;
        }

        boolean GetRunState() {
            return run_state;
        }
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
