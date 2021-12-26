package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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
    //static DcMotor CarouselMotor;
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

    hk_BucketControl BucketMotor;
    hk_Arm_Control ArmMotor;

    int bucketresetorder = 0;
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
        Bucket = hardwareMap.get(DcMotor.class, "BucketMotor");
        IntakeServo = hardwareMap.get(Servo.class, "IntakeServo");
        GateServo = hardwareMap.get(Servo.class, "GateServo");
        ElementServo = hardwareMap.get(Servo.class, "ElementServo");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        //BucketCRServoCtrl_Thread cr_thread = new BucketCRServoCtrl_Thread();
        //cr_thread.start();

        AttachmentSetDirection();
        SetDirection(MoveDirection.REVERSE);

        //Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Arm.setTargetPosition(0);
        Rail.setTargetPosition(0);

        //Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        IntakeServo.scaleRange(0,1);
        GateServo.scaleRange(0,1);

        IntakeServo.setPosition(OpenIntakePosition);
        GateServo.setPosition(OpenGatePosition);

        BucketMotor = new hk_BucketControl(Bucket);
        ArmMotor = new hk_Arm_Control(Arm);

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

            double y = -gamepad1.left_stick_y * 0.5;
            //double x = gamepad1.left_stick_x * 0.55;
            double x = gamepad1.left_stick_x * 0.5;
            double rx = gamepad1.right_stick_x * 0.5;

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

            //if (gamepad1.dpad_right) {
                //CarouselMotor.setPower(1.0);
            //}
            //if (gamepad1.dpad_left) {
                //CarouselMotor.setPower(-1.0);
            //}
            //if (gamepad1.x) {
                //CarouselMotor.setPower(0);
            //}

            /****************************************
             Top Level (For Opposite, press dpad_down first)
             ***************************************/

            if (button_dpad_down_already_pressed2 == false) {
                if (gamepad2.dpad_down) {
                    mirror_event = true;
                }
            } else {
                if (!gamepad2.dpad_down) {
                    //mirror_event = false;
                    button_dpad_down_already_pressed2 = false;
                }
            }

            if (button_y_already_pressed == false) {
                if (gamepad2.y) {
                    tophuborder = 1;
                }
            } else {
                if (!gamepad2.y) {
                    button_y_already_pressed = false;
                }
            }

            switch (tophuborder) {

                case 1:
                    if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {
                        BucketMotor.SetTargetPosition(0);
                    }
                    else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                        tophuborder++;
                    }
                    GateServo.setPosition(ClosingGatePosition);
                    IntakeServo.setPosition(ClosingIntakePosition);
                    break;

                case 2:
                    Rail.setTargetPosition(1000);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    if (Rail.getCurrentPosition() >= 970 && Rail.getCurrentPosition() <= 1030) {
                        tophuborder++;
                    }
                    break;

                case 3:

                    if (mirror_event) {
                        if (ArmMotor.GetTaskState() == Task_State.INIT || ArmMotor.GetTaskState() == Task_State.READY) {

                            ArmMotor.SetTargetPosition(390, 0.6);
                        } else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            tophuborder++;
                        }
                    } else if (!mirror_event) {
                        if (ArmMotor.GetTaskState() == Task_State.INIT || ArmMotor.GetTaskState() == Task_State.READY) {

                            ArmMotor.SetTargetPosition(-390, 0.6);
                        }
                        else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            tophuborder++;
                        }
                    }
                    break;

                case 4:
                    if (mirror_event) {
                        if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {

                            BucketMotor.SetTargetPosition(MirrorTopBucketPosition);
                        }
                        else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                            tophuborder++;
                            mirror_event = false;
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

            /****************************************
             Reset Position For Barrier Passing
             ***************************************/

            if (button_x_already_pressed == false) {
                if (gamepad2.x) {
                    bucketresetorder = 1;
                    button_x_already_pressed = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed = false;
                }
            }

            switch (bucketresetorder) {

                case 1:
                    GateServo.setPosition(ClosingGatePosition);
                    bucketresetorder++;
                    break;

                case 2:
                    if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {
                        BucketMotor.SetTargetPosition(OriginalBucketPosition);
                    }
                    else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                        bucketresetorder++;
                    }
                    break;

                case 3:
                    if (ArmMotor.GetTaskState() == Task_State.INIT ||
                            ArmMotor.GetTaskState() == Task_State.READY) {

                        ArmMotor.SetTargetPosition(0, -0.00003);
                    }
                    else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                        bucketresetorder++;
                        ET.reset();
                    }
                    break;

                case 4:
                    if (ET.milliseconds() > 10000) {
                        bucketresetorder++;
                    }

                    break;

                case 5:
                    if (ArmMotor.GetTaskState() == Task_State.INIT ||
                            ArmMotor.GetTaskState() == Task_State.READY) {

                        ArmMotor.Calibrate();

                        Rail.setTargetPosition(300);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.5);
                    }
                    else if (ArmMotor.GetTaskState() == Task_State.DONE && BucketMotor.GetTaskState() == Task_State.DONE) {
                        bucketresetorder++;
                    }
                    break;

                default:
                    break;
            }

            /****************************************
             Open Gate
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
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            /********************************
             * Pick up shipping element
             ********************************/

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

            switch (servoseqleft) {

                case 1:
                    servoposition = servoposition - 0.003;
                    servoposition = Range.clip(servoposition, 0, 1);
                    ElementServo.setPosition(servoposition);
                    break;

                default:
                    break;

            }

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

            switch (servoseqright) {

                case 1:
                    servoposition = servoposition + 0.003;
                    servoposition = Range.clip(servoposition, 0, 1);
                    ElementServo.setPosition(servoposition);
                    break;

                default:
                    break;

            }

            /********************************
             * Arm position manual movement
             ********************************/

            //if (button_a_already_pressed2 == false) {
                //if (gamepad2.a) {
                    //armorder = 1;

                    //button_a_already_pressed2 = true;
                //}
            //} else {
                //if (!gamepad2.a) {
                    //armorder = 0;
                    //button_a_already_pressed2 = false;
                //}
            //}

            //switch (armorder) {

                //case 1:
                    //armposition = armposition - 2;
                    //Arm.setTargetPosition(armposition);
                    //Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //Arm.setPower(0.3);
                    //break;

                //default:
                    //break;

            //}

            //if (button_b_already_pressed2 == false) {
                //if (gamepad2.b) {

                    //armorder = 1;

                    //button_b_already_pressed2 = true;
                //}
            //} else {
                //if (!gamepad2.b) {
                    //armorder = 0;
                    //button_b_already_pressed2 = false;
                //}
            //}

            //switch (armorder) {

                //case 1:
                    //armposition = armposition + 2;
                    //Arm.setTargetPosition(armposition);
                    //Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    //Arm.setPower(0.3);
                    //break;

                //default:
                    //break;

            //}

            ArmMotor.ArmTask();
            BucketMotor.BucketTask();

            //telemetry.addData("Rail Current Position", Rail.getCurrentPosition());
            //telemetry.addData("encoder", FrontRight.getCurrentPosition());
            //telemetry.addData("encoder", BackRight.getCurrentPosition());
            //telemetry.addData("Original bucket position", OriginalBucketPosition);
            //telemetry.addData("High bucket position", TopBucketPosition);
            //telemetry.addData("Middle bucket position", MiddleBucketPosition);
            telemetry.addData("TopHub Sequence", tophuborder);
            telemetry.addData("BucketResetSequence", bucketresetorder);
            telemetry.addData("ArmPower", Arm.getPower());
            telemetry.addData("Arm Current Position", Arm.getCurrentPosition());
            telemetry.addData("BucketPower", Bucket.getPower());
            telemetry.addData("bucket position", Bucket.getCurrentPosition());

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
        //CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        IntakeServo.setDirection(Servo.Direction.FORWARD);
        GateServo.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        //.setDirection(DcMotor.Direction.FORWARD);
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