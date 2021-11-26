package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MecanumDriveTeleop", group="MecanumDrive")
public class OfficialTeleop extends LinearOpMode {

    static DcMotor FrontLeft;
    static DcMotor BackLeft;
    static DcMotor FrontRight;
    static DcMotor BackRight;
    static DcMotor Arm;
    static DcMotor Rail;
    static MoveDirection Direction;
    static DcMotor Intake;
    static DcMotor CarouselMotor;
    BNO055IMU IMU;
    static CRServo BucketServo;
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
        BucketServo = hardwareMap.get(CRServo.class, "BucketServo");
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



        waitForStart();

        ET.reset();

        while (opModeIsActive()) {

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
                Intake.setPower(0.85);
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
                CarouselMotor.setPower(0.6);
            }
            if (gamepad1.dpad_left) {
                CarouselMotor.setPower(-0.6);
            }
            if (gamepad1.x) {
                CarouselMotor.setPower(0);
            }

            /****************************************
             Top Level (For Opposite, press dpad_down first)
             ***************************************/

            if (button_y_already_pressed == false) {
                if (gamepad2.y) {
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
                                top_level_event = false;
                            } else {
                                Arm.setTargetPosition(-350);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
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
                                middle_level_event = false;
                            } else {
                                Arm.setTargetPosition(-250);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
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
                                low_level_event = false;
                            } else {
                                Arm.setTargetPosition(-150);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.2);
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
                    if (servo_power == true) {
                        ET.reset();
                        while (ET.milliseconds() < 855) {
                            BucketServo.setPower(-0.5);
                        }
                        BucketServo.setPower(0);
                    } else if (servo_power == false) {
                        ET.reset();
                        while (ET.milliseconds() < 855) {
                            BucketServo.setPower(0.5);
                        }
                        BucketServo.setPower(0);
                    }
                    Arm.setTargetPosition(0);
                    Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Arm.setPower(0.2);
                    ET.reset();
                    button_x_already_pressed = true;
                    barrier_event = true;
                }
                else {
                    if (barrier_event == true) {
                        if (ET.milliseconds() > 1000) {
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
             Move Servo Left and Right
             ***************************************/

            if (button_dpad_left_already_pressed2 == false) {
                if (gamepad2.dpad_left) {
                    BucketServo.setPower(0.5);
                    button_dpad_left_already_pressed2 = true;
                    servo_left_event = true;
                    ET.reset();
                }
                else {
                    if (servo_left_event == true) {
                        if (ET.milliseconds() > 850) {
                            BucketServo.setPower(0);
                            servo_left_event = false;
                            servo_power = true;
                        }
                    }
                }
            } else {
                if (!gamepad2.dpad_left) {
                    button_dpad_left_already_pressed2 = false;
                }
            }

            if (button_dpad_right_already_pressed2 == false) {
                if (gamepad2.dpad_right) {
                    BucketServo.setPower(-0.5);
                    button_dpad_right_already_pressed2 = true;
                    servo_default_event = true;
                    ET.reset();
                }
                else {
                    if (servo_default_event == true) {
                        if (ET.milliseconds() > 850) {
                            BucketServo.setPower(0);
                            servo_default_event = false;
                            servo_power = false;
                        }
                    }
                }
            } else {
                if (!gamepad2.dpad_right) {
                    button_dpad_right_already_pressed2 = false;
                }
            }

            /****************************
             * Bucket micro adjustments
             ****************************/
            if (button_bumper_right_already_pressed == false) {
                if (gamepad1.right_bumper) {
                    Rail.setTargetPosition(800);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    BucketServo.setPower(-0.5);
                    ET.reset();
                    while (ET.milliseconds() < 20) {

                    }
                    BucketServo.setPower(0);
                    button_bumper_right_already_pressed = true;
                }
            }
            else {
                if (!gamepad1.right_bumper) {
                    button_bumper_right_already_pressed = false;
                }
            }

            if (button_bumper_left_already_pressed == false) {
                if (gamepad1.left_bumper) {
                    Rail.setTargetPosition(800);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    BucketServo.setPower(0.5);
                    ET.reset();
                    while (ET.milliseconds() < 20) {

                    }
                    BucketServo.setPower(0);
                    button_bumper_left_already_pressed = true;
                }
            }
            else {
                if (!gamepad1.left_bumper) {
                    button_bumper_left_already_pressed = false;
                }
            }

            /****************************
             * Arm micro adjustments
             ****************************/
            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper) {
                    Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Arm.setPower(-0.5);
                    ET.reset();
                    while (ET.milliseconds() < 20) {

                    }
                    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BucketServo.setPower(0);
                    button_bumper_right_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                }
            }

            if (button_bumper_left_already_pressed2 == false) {
                if (gamepad2.left_bumper) {
                    Arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    Arm.setPower(0.5);
                    ET.reset();
                    while (ET.milliseconds() < 20) {

                    }
                    Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    BucketServo.setPower(0);
                    button_bumper_left_already_pressed2 = true;
                }
            }
            else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
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


            telemetry.addData("Arm Current Position", Arm.getCurrentPosition());
            telemetry.addData("Rail Current Position", Rail.getCurrentPosition());
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
        BucketServo.setDirection(CRServo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
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

                        BucketServo.setDirection(run_direction);
                        BucketServo.setPower(run_speed);

                        if (ET.milliseconds() >= run_duration) {
                            BucketServo.setPower(0);
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

}
