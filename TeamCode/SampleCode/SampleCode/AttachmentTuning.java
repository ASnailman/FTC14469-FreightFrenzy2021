package SampleCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MoveDirection;

@TeleOp(name="AttachmentTuning", group="MecanumDrive")
public class AttachmentTuning extends LinearOpMode {

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
    static Servo BucketServo;
    double intake_power = 0.5;
    int arm_position = 0;
    int rail_position = 0;
    //int time = 0;
    double bucketservo_position = 0.49;

    boolean middle_level_event;
    boolean mirror_event;

    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_x_already_pressed = false;
    boolean button_y_already_pressed = false;
    boolean button_a_already_pressed2 = false;
    boolean button_b_already_pressed2 = false;
    boolean button_x_already_pressed2 = false;
    boolean button_y_already_pressed2 = false;
    boolean button_bumper_left_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;
    boolean button_bumper_left_already_pressed2 = false;
    boolean button_bumper_right_already_pressed2 = false;
    boolean button_dpad_up_already_pressed2 = false;
    boolean button_dpad_down_already_pressed2 = false;
    boolean button_dpad_right_already_pressed2 = false;


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
        BucketServo = hardwareMap.get(Servo.class, "BucketServo");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        //BucketCRServoCtrl_Thread cr_thread = new BucketCRServoCtrl_Thread();
        //cr_thread.start();

        BucketServo.scaleRange(0,1);
        BucketServo.setPosition(0.4);

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

        Arm.setTargetPosition(-20);
        Rail.setTargetPosition(0);

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {

            /****************************************
             Movement
             ***************************************/

            double y = -gamepad1.left_stick_y * 0.6;
            //double x = gamepad1.left_stick_x * 0.55;
            double x = gamepad1.left_stick_x * 0.61;
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
             Increase Intake Power (G1): right bumper = power + 0.01, left bumper = power - 0.01
             ***************************************/

            if (button_bumper_right_already_pressed == false) {
                if (gamepad1.right_bumper) {
                    intake_power = intake_power + 0.01;
                    button_bumper_right_already_pressed = true;
                }
            } else {
                if (!gamepad1.right_bumper) {
                    button_bumper_right_already_pressed = false;
                }
            }

            if (button_bumper_left_already_pressed == false) {
                if (gamepad1.left_bumper) {
                    intake_power = intake_power - 0.01;
                    button_bumper_left_already_pressed = true;
                }
            } else {
                if (!gamepad1.left_bumper) {
                    button_bumper_left_already_pressed = false;
                }
            }

            /****************************************
             Run Intake (G1): dpad_up = power, dpad_down = 0
             ***************************************/

            if (gamepad1.dpad_up) {
                Intake.setPower(intake_power);
            }
            if (gamepad1.dpad_down) {
                Intake.setPower(0);
            }

            /****************************************
             Run CarouselMotor (G1): dpad_right = power 100%, dpad_down = power 0%
             ***************************************/

            if (gamepad1.dpad_right) {
                CarouselMotor.setPower(0.5);
            }
            if (gamepad1.dpad_left) {
                CarouselMotor.setPower(0);
            }

            /****************************************
             Increase BucketServo Position: a = increase position by 0.02, b = decrease sleep by 0.02
             ***************************************/

            if (button_a_already_pressed == false) {
                if (gamepad1.a) {
                    bucketservo_position = bucketservo_position + 0.01;
                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad1.a) {
                    button_a_already_pressed = false;
                }
            }

            if (button_b_already_pressed == false) {
                if (gamepad1.b) {
                    bucketservo_position = bucketservo_position - 0.01;
                    button_b_already_pressed = true;
                }
            } else {
                if (!gamepad1.b) {
                    button_b_already_pressed = false;
                }
            }

            /****************************************
             Run BucketServo (G1): x = set position y = set position 0
             ***************************************/

            if (button_x_already_pressed == false) {
                if (gamepad1.x) {
                        BucketServo.setPosition(bucketservo_position);
                        button_x_already_pressed = true;
                    }
                } else {
                    if (!gamepad1.x) {
                        button_x_already_pressed = false;
                    }
                }

                if (button_y_already_pressed == false) {
                    if (gamepad1.y) {
                        BucketServo.setPosition(0);
                        button_y_already_pressed = true;
                    }
                } else {
                    if (!gamepad1.y) {
                        button_y_already_pressed = false;
                    }
                }

                /****************************************
                 Increase Arm Position (G2): right bumper = power + 10, left bumper = power - 10
                 ***************************************/

                if (button_bumper_right_already_pressed2 == false) {
                    if (gamepad2.right_bumper) {
                        arm_position = arm_position + 50;
                        button_bumper_right_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.right_bumper) {
                        button_bumper_right_already_pressed2 = false;
                    }
                }

                if (button_bumper_left_already_pressed2 == false) {
                    if (gamepad2.left_bumper) {
                        arm_position = arm_position - 50;
                        button_bumper_left_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.left_bumper) {
                        button_bumper_left_already_pressed2 = false;
                    }
                }

                /****************************************
                 Run Arm (G2): dpad_up = position, dpad_down = 0 Run Arm Default
                 ***************************************/

                if (button_dpad_up_already_pressed2 == false) {
                    if (gamepad2.dpad_up) {
                        Arm.setTargetPosition(arm_position);
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Arm.setPower(0.1);
                        button_dpad_up_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.dpad_up) {
                        button_dpad_up_already_pressed2 = false;
                    }
                }

                if (button_dpad_down_already_pressed2 == false) {
                    if (gamepad2.dpad_down) {
                        Arm.setTargetPosition(0);
                        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Arm.setPower(0.1);
                        button_dpad_down_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.dpad_down) {
                        button_dpad_down_already_pressed2 = false;
                    }
                }

                /****************************************
                 Increase Rail Position (G2): Button X = power + 10, Button Y = power - 10
                 ***************************************/

                if (button_x_already_pressed2 == false) {
                    if (gamepad2.x) {
                        rail_position = rail_position + 50;
                        button_x_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.x) {
                        button_x_already_pressed2 = false;
                    }
                }

                if (button_y_already_pressed2 == false) {
                    if (gamepad2.y) {
                        rail_position = rail_position - 50;
                        button_y_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.y) {
                        button_y_already_pressed2 = false;
                    }
                }

                /****************************************
                 Run Rail (G2): Button A = Position, Button B = 0
                 ***************************************/

                if (button_a_already_pressed2 == false) {
                    if (gamepad2.a) {
                        Rail.setTargetPosition(rail_position);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.1);
                        button_a_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.a) {
                        button_a_already_pressed2 = false;
                    }
                }

                if (button_b_already_pressed2 == false) {
                    if (gamepad2.b) {
                        Rail.setTargetPosition(0);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.1);
                        button_b_already_pressed2 = true;
                    }
                } else {
                    if (!gamepad2.b) {
                        button_b_already_pressed2 = false;
                    }
                }

            if (button_dpad_right_already_pressed2 == false) {
                if (gamepad2.dpad_right) {
                    //IntakeServo.setPosition(ClosingIntakePosition);
                    Rail.setTargetPosition(750);
                    Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rail.setPower(0.5);
                    button_dpad_right_already_pressed2 = true;
                    middle_level_event = true;
                } else {
                    if (gamepad2.dpad_down) {
                        mirror_event = true;
                    }
                    if (middle_level_event == true) {
                        if (Rail.getCurrentPosition() >= 720 && Rail.getCurrentPosition() <= 780) {
                            if (mirror_event) {
                                Arm.setTargetPosition(250);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.3);

                                if (Arm.getCurrentPosition() >= 220 && Arm.getCurrentPosition() <= 280) {
                                    sleep(2000);
                                    BucketServo.setPosition(0.79);
                                    middle_level_event = false;
                                }
                            } else {
                                Arm.setTargetPosition(-250);
                                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                                Arm.setPower(0.3);

                                if (Arm.getCurrentPosition() >= -280 && Arm.getCurrentPosition() <= -220) {
                                    sleep(2000);
                                    BucketServo.setPosition(0.19);
                                    middle_level_event = false;
                                }
                            }
                        }
                    }
                }
            } else {
                if (!gamepad2.dpad_right) {
                    button_dpad_right_already_pressed2 = false;
                }
            }

                telemetry.addData("Intake Power", intake_power);
                telemetry.addData("Arm Controller Position", arm_position);
                telemetry.addData("Rail Controller Position", rail_position);
                telemetry.addData("Arm Current Position", Arm.getCurrentPosition());
                telemetry.addData("Rail Current Position", Rail.getCurrentPosition());
                //telemetry.addData("time", time);
                telemetry.addData("BucketServo Position", bucketservo_position);
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

    }
