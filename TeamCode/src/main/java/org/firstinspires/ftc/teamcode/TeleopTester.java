package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleopTester", group="MecanumDrive")
public class TeleopTester extends LinearOpMode {

    static DcMotor FrontLeft;
    static DcMotor BackLeft;
    static DcMotor FrontRight;
    static DcMotor BackRight;
    static DcMotor Arm;
    static DcMotor Rail;
    static MoveDirection Direction;
    static DcMotor Intake;
    //static DcMotor CarouselMotor;
    static NormalizedColorSensor colorsensor;
    static RevBlinkinLedDriver ColorStrip;
    BNO055IMU IMU;

    static DcMotorEx BucketMotor;
    static PIDFCoefficients PIDCoefficient;
    double kp;
    double ki;
    double kd;

    static Servo IntakeServo;
    static Servo GateServo;
    boolean top_level_event;
    boolean middle_level_event;
    boolean low_level_event;
    boolean mirror_event;
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
    boolean button_dpad_left_already_pressed = false;
    boolean button_dpad_left_already_pressed2 = false;
    boolean button_dpad_right_already_pressed = false;
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

    static final double OriginalBucketPosition_Base = 0.5;
    static double OriginalBucketPosition = OriginalBucketPosition_Base;

    static final double TopBucketPosition_Base = 0.16;
    static double TopBucketPosition = TopBucketPosition_Base;

    static final double MirrorTopBucketPosition_Base = 0.86;
    static double MirrorTopBucketPosition = MirrorTopBucketPosition_Base;

    static final double MiddleBucketPosition_Base = 0.20;
    static double MiddleBucketPosition = MiddleBucketPosition_Base;

    static final double MirrorMiddleBucketPosition_Base = 0.80;
    static double MirrorMiddleBucketPosition = MirrorMiddleBucketPosition_Base;

    static final double LowBucketPosition_Base = 0.26;
    static double LowBucketPosition = LowBucketPosition_Base;

    static final double MirrorLowBucketPosition_Base = 0.74;
    static double MirrorLowBucketPosition = MirrorLowBucketPosition_Base;

    static final double OpenGatePosition = 0.5;
    static final double OpenIntakePosition = 0.6;
    static final double ClosingGatePosition = 0.2;
    static final double ClosingIntakePosition = 0.8;

    @Override
    public void runOpMode() {

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        Rail = hardwareMap.get(DcMotor.class, "rail");
        //CarouselMotor = hardwareMap.get(DcMotor.class, "carouselmotor");
        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
        ColorStrip = hardwareMap.get(RevBlinkinLedDriver.class, "colorstrip");
        BucketMotor = hardwareMap.get(DcMotorEx.class, "BucketMotor");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        GateServo = hardwareMap.get(Servo.class, "GateServo");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");


        //BucketCRServoCtrl_Thread cr_thread = new BucketCRServoCtrl_Thread();
        //cr_thread.start();

        AttachmentSetDirection();
        SetDirection(MoveDirection.REVERSE);

        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BucketMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        BucketMotor.setTargetPosition(0);
        Arm.setTargetPosition(0);
        Rail.setTargetPosition(0);

        BucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //BucketServo.scaleRange(0,1);
        IntakeServo.scaleRange(0,1);
        GateServo.scaleRange(0,1);

        //BucketServo.setPosition(OriginalBucketPosition);
        IntakeServo.setPosition(OpenIntakePosition);
        GateServo.setPosition(OpenGatePosition);

        PIDCoefficient = BucketMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        kp = PIDCoefficient.p;
        ki = PIDCoefficient.i;
        kd = PIDCoefficient.d;

        waitForStart();

        ET.reset();

        BucketMotor.getTargetPositionTolerance();

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

            double y = -gamepad1.left_stick_y * 0.75;
            //double x = gamepad1.left_stick_x * 0.55;
            double x = gamepad1.left_stick_x * 0.825;
            double rx = gamepad1.right_stick_x * 0.6;

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

            //if (gamepad1.dpad_up) {
              //  IntakeServo.setPosition(OpenIntakePosition);
                //Intake.setPower(1);
                //Rail.setTargetPosition(0);
                //Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //Rail.setPower(0.5);
            //}
            //if (gamepad1.dpad_down) {
              //  Intake.setPower(0);
            //}

            /****************************************
             Run CarouselMotor
             ***************************************/

            //if (gamepad1.dpad_right) {
                //CarouselMotor.setPower(1.0);
            //}



            if (button_dpad_left_already_pressed == false) {
                if (gamepad1.dpad_left) {
                    BucketMotor.setTargetPosition(0);
                    //BucketMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDCoefficient);
                    BucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BucketMotor.setPower(0.3);
                    //BucketMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    button_dpad_left_already_pressed = true;
                }
            } else {
                if (!gamepad1.dpad_left) {
                    button_dpad_left_already_pressed = false;
                }
            }


            if (gamepad1.x) {
                //CarouselMotor.setPower(0);
            }

            /****************************************
             Top Level (For Opposite, press dpad_down first)
             ***************************************/

            if (button_y_already_pressed == false) {
                if (gamepad2.y) {
                    IntakeServo.setPosition(ClosingIntakePosition);
                    GateServo.setPosition(ClosingGatePosition);
                    Rail.setTargetPosition(1000);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    button_y_already_pressed = true;
                    top_level_event = true;
                } else {
                    if (gamepad2.dpad_down) {
                        mirror_event = true;
                    }
                    if (top_level_event == true) {
                        if (Rail.getCurrentPosition() >= 970 && Rail.getCurrentPosition() <= 1030) {
                            if (mirror_event) {
                                Arm.setTargetPosition(390);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.3);

                                if (Arm.getCurrentPosition() >= 360 && Arm.getCurrentPosition() <= 420) {
                                    //BucketServo.setPosition(MirrorTopBucketPosition);
                                    top_level_event = false;
                                    mirror_event = false;
                                }
                            } else {
                                Arm.setTargetPosition(-390);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.3);

                                if (Arm.getCurrentPosition() >= -420 && Arm.getCurrentPosition() <= -360) {
                                    //BucketServo.setPosition(TopBucketPosition);
                                    top_level_event = false;
                                }
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
                    GateServo.setPosition(ClosingGatePosition);
                    Rail.setTargetPosition(750);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    button_b_already_pressed = true;
                    middle_level_event = true;
                } else {
                    if (gamepad2.dpad_down) {
                        mirror_event = true;
                    }
                    if (middle_level_event == true) {
                        if (Rail.getCurrentPosition() >= 720 && Rail.getCurrentPosition() <= 780) {
                            if (mirror_event) {
                                Arm.setTargetPosition(290);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.3);

                                if (Arm.getCurrentPosition() >= 260 && Arm.getCurrentPosition() <= 320) {
                                    //sleep(2000);
                                    //BucketServo.setPosition(MirrorMiddleBucketPosition);
                                    middle_level_event = false;
                                    mirror_event = false;
                                }
                            } else {
                                Arm.setTargetPosition(-290);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.3);

                                if (Arm.getCurrentPosition() >= -320 && Arm.getCurrentPosition() <= -260) {
                                    //sleep(2000);
                                    //BucketServo.setPosition(MiddleBucketPosition);
                                    middle_level_event = false;
                                }
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
                    GateServo.setPosition(ClosingGatePosition);
                    Rail.setTargetPosition(750);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    button_a_already_pressed = true;
                    low_level_event = true;
                } else {
                    if (gamepad2.dpad_down) {
                        mirror_event = true;
                    }
                    if (low_level_event == true) {
                        if (Rail.getCurrentPosition() >= 720 && Rail.getCurrentPosition() <= 780) {
                            if (mirror_event) {
                                Arm.setTargetPosition(150);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.3);

                                if (Arm.getCurrentPosition() >= 120 && Arm.getCurrentPosition() <= 180) {
                                    //BucketServo.setPosition(MirrorLowBucketPosition);
                                    low_level_event = false;
                                    mirror_event = false;
                                }
                            } else {
                                Arm.setTargetPosition(-150);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.3);

                                if (Arm.getCurrentPosition() >= -180 && Arm.getCurrentPosition() <= -120) {
                                    //BucketServo.setPosition(LowBucketPosition);
                                    low_level_event = false;
                                }
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
                    //BucketServo.setPosition(OriginalBucketPosition);
                    button_x_already_pressed = true;
                    barrier_event = true;
                }
                else {
                    if (barrier_event == true) {
                        //if (BucketServo.getPosition() == OriginalBucketPosition) {
                            Arm.setTargetPosition(0);
                            Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Arm.setPower(0.2);
                        //}

                        if (Arm.getCurrentPosition() == 0) {
                            Rail.setTargetPosition(300);
                            Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Rail.setPower(0.5);
                            barrier_event = false;
                        }

                        //if (Rail.getCurrentPosition() >= 570 && Rail.getCurrentPosition() <= 630) {
                        //    Rail.setTargetPosition(300);
                        //    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        //    Rail.setPower(0.5);
                        //    barrier_event = false;
                        //}

                        mirror_event = false;
                    }
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed = false;
                }
            }

            /****************************************
             Open Gate
             ***************************************/

            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper) {
                    GateServo.setPosition(OpenGatePosition);
                    //IntakeServo.setPosition(OpenIntakePosition);
                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.right_bumper) {
                    //GateServo.setPosition(ClosingGatePosition);
                    //IntakeServo.setPosition(ClosingIntakePosition);
                    button_bumper_right_already_pressed2 = false;
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

            /********************************
             * Calibrate Arm zero position
             ********************************/
            if (button_left_trigger_already_pressed == false) {
                if (gamepad1.left_trigger > 0) {
                    Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Arm.setPower(0);
                    ET.reset();
                    while (ET.milliseconds() < 1000) {

                    }
                    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //Rail.setTargetPosition(0);
                    //Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //Rail.setPower(0.5);
                    button_left_trigger_already_pressed = true;
                }
            }
            else {
                if (gamepad1.left_trigger == 0) {
                    button_left_trigger_already_pressed = false;
                }
            }

            /********************************
             * Bucket Barrier Return Auto Fix
             ********************************/
            if (button_bumper_left_already_pressed2 == false) {

                if (gamepad2.left_bumper) {

                    Rail.setTargetPosition(750);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    button_bumper_left_already_pressed2 = true;
                    barrier_correction_event = true;
                }
                else if (barrier_correction_event)
                {
                    if (Rail.getCurrentPosition() >= 720 && Rail.getCurrentPosition() <= 780) {
                        //BucketServo.setPosition(OriginalBucketPosition);
                        //sleep(1500);
                        //Rail.setTargetPosition(300);
                        //Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        //Rail.setPower(0.5);
                        barrier_correction_event = false;
                    }
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            /********************************
             PID Coefficient Tuner
             ********************************/

            if (button_dpad_left_already_pressed2 == false) {
                if (gamepad2.dpad_left) {

                    kp = kp - 0.1;

                    button_dpad_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_left) {
                    button_dpad_left_already_pressed2 = false;
                }
            }

            if (button_dpad_left_already_pressed == false) {
                if (gamepad1.dpad_left) {

                    kp = kp + 0.1;

                    button_dpad_left_already_pressed = true;
                }
            } else {
                if (!gamepad1.dpad_left) {
                    button_dpad_left_already_pressed = false;
                }
            }

            if (button_dpad_up_already_pressed2 == false) {
                if (gamepad2.dpad_up) {

                    ki = ki - 0.001;

                    button_dpad_up_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_up) {
                    button_dpad_up_already_pressed2 = false;
                }
            }

            if (button_dpad_up_already_pressed == false) {
                if (gamepad1.dpad_up) {

                    ki = ki + 0.001;

                    button_dpad_up_already_pressed = true;
                }
            } else {
                if (!gamepad1.dpad_up) {
                    button_dpad_up_already_pressed = false;
                }
            }

            if (button_dpad_right_already_pressed2 == false) {
                if (gamepad2.dpad_right) {

                    kd = kd - 0.0001;

                    button_dpad_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.dpad_right) {
                    button_dpad_right_already_pressed2 = false;
                }
            }

            if (button_dpad_right_already_pressed == false) {
                if (gamepad1.dpad_right) {

                    kd = kd + 0.0001;

                    button_dpad_right_already_pressed = true;
                }
            } else {
                if (!gamepad1.dpad_right) {
                    button_dpad_right_already_pressed = false;
                }
            }

            if (button_right_trigger_already_pressed2 == false) {
                if (gamepad2.right_trigger > 0) {

                    PIDCoefficient.p = kp;
                    PIDCoefficient.i = ki;
                    PIDCoefficient.d = kd;
                    BucketMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, PIDCoefficient);

                    button_right_trigger_already_pressed2 = true;
                }
            } else {
                if (gamepad2.right_trigger == 0) {
                    button_right_trigger_already_pressed2 = false;
                }
            }

            /********************************
             Move Bucket
             ********************************/
            if (button_bumper_right_already_pressed == false) {
                if (gamepad1.right_bumper) {

                    BucketMotor.setTargetPosition(250);
                    BucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BucketMotor.setPower(0.3);
                    //BucketMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    button_bumper_right_already_pressed = true;
                }
            } else {
                if (!gamepad1.right_bumper) {
                    button_bumper_right_already_pressed = false;
                }
            }

            if (button_bumper_left_already_pressed == false) {
                if (gamepad1.left_bumper) {

                    BucketMotor.setTargetPosition(-250);
                    BucketMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    BucketMotor.setPower(0.3);
                    //BucketMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    button_bumper_left_already_pressed = true;
                }
            } else {
                if (!gamepad1.left_bumper) {
                    button_bumper_left_already_pressed = false;
                }
            }

            telemetry.addData("Arm Current Position", Arm.getCurrentPosition());
            telemetry.addData("Rail Current Position", Rail.getCurrentPosition());
            telemetry.addData("encoder", FrontRight.getCurrentPosition());
            telemetry.addData("encoder", BackRight.getCurrentPosition());
            telemetry.addData("bucket position", BucketMotor.getCurrentPosition());
            telemetry.addData("Original bucket position", OriginalBucketPosition);
            telemetry.addData("High bucket position", TopBucketPosition);
            telemetry.addData("Middle bucket position", MiddleBucketPosition);
            telemetry.addData("Low bucket position", LowBucketPosition);
            telemetry.addData("Shooter P", PIDCoefficient.p);
            telemetry.addData("Shooter I", PIDCoefficient.i);
            telemetry.addData("Shooter D", PIDCoefficient.d);
            telemetry.addData("Shooter kp", kp);
            telemetry.addData("Shooter ki", ki);
            telemetry.addData("Shooter kd", kd);
            telemetry.addData("PositionTolerance", BucketMotor.getTargetPositionTolerance());
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

        //CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        BucketMotor.setDirection(DcMotor.Direction.FORWARD);
        IntakeServo.setDirection(Servo.Direction.FORWARD);
        GateServo.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Rail.setDirection(DcMotor.Direction.FORWARD);

    }

    private void BucketPositionSet (double power, double targetdistance) {

        while (BucketMotor.getCurrentPosition() <= targetdistance) {

            BucketMotor.setDirection(DcMotorSimple.Direction.FORWARD);

            BucketMotor.setPower(power);
            telemetry.addData("BucketMotor", -BackLeft.getCurrentPosition());
            telemetry.update();

        }

        while (-BucketMotor.getCurrentPosition() >= targetdistance) {

            BucketMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            BucketMotor.setPower(-power);

            telemetry.addData("BucketMotor", -BackLeft.getCurrentPosition());
            telemetry.update();

        }

        BucketMotor.setPower(0);
        sleep(100);

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
