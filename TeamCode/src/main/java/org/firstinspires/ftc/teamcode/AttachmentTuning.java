package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    static CRServo CarouselServo;
    static Servo BucketServo;
    double intake_power = 0.5;
    int arm_position = 0;
    int rail_position = 0;

    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_x_already_pressed = false;
    boolean button_y_already_pressed = false;
    boolean button_x_already_pressed2 = false;
    boolean button_y_already_pressed2 = false;
    boolean button_bumper_left_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;
    boolean button_bumper_left_already_pressed2 = false;
    boolean button_bumper_right_already_pressed2 = false;

    @Override
    public void runOpMode() {

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        Rail = hardwareMap.get(DcMotor.class, "rail");
        //CarouselServo = hardwareMap.get(CRServo.class, "carouselservo");
        BucketServo = hardwareMap.get(Servo.class, "BucketServo");

        AttachmentSetDirection();
        SetDirection(MoveDirection.REVERSE);

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setTargetPosition(0);
        Rail.setTargetPosition(0);

        waitForStart();

        while (opModeIsActive()) {

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
             Run CarouselServo (G1): dpad_right = power 100%, dpad_down = power 0%
             ***************************************/

            if (gamepad1.dpad_right) {
                CarouselServo.setPower(1);
            }
            if (gamepad1.dpad_left) {
                CarouselServo.setPower(0);
            }

            ///****************************************
            // Run CarouselMotor (G1): dpad_right = power 100%, dpad_down = power 0%
            // ***************************************/

            //if (gamepad1.dpad_right) {
            //    CarouselServo.setPower(1);
            //}
            //if (gamepad1.dpad_left) {
            //    CarouselServo.setPower(0);
            //}

            ///****************************************
            // Run BucketServo (G2): dpad_right = 180 degrees, dpad_down = 0 degrees
            // ***************************************/

            //if (gamepad2.dpad_right) {
            //    BucketServo.setPosition(180);
            //}
            //if (gamepad2.dpad_left) {
            //    BucketServo.setPower(0);
            //}

            ///****************************************
            // Run BucketServo (G2): dpad_right = power 100%, dpad_down = power 0%
            // ***************************************/

            //if (gamepad2.dpad_right) {
            //    CarouselServo.setPower(1);
            //}
            //if (gamepad2.dpad_left) {
            //    CarouselServo.setPower(0);
            //}

            /****************************************
             Increase Arm Position (G2): right bumper = power + 10, left bumper = power - 10
             ***************************************/

            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper) {
                    arm_position = arm_position + 10;
                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                }
            }

            if (button_bumper_left_already_pressed2 == false) {
                if (gamepad2.left_bumper) {
                    arm_position = arm_position - 10;
                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            /****************************************
             Run Arm (G2): dpad_up = position, dpad_down = 0
             ***************************************/

            if (gamepad2.dpad_up) {
                Arm.setTargetPosition(arm_position);
            }
            if (gamepad2.dpad_down) {
                Arm.setTargetPosition(0);
            }

            /****************************************
             Increase Rail Position (G2): Button X = power + 10, Button Y = power - 10
             ***************************************/

            if (button_x_already_pressed2 == false) {
                if (gamepad2.x) {
                    rail_position = rail_position + 10;
                    button_x_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed2 = false;
                }
            }

            if (button_y_already_pressed2 == false) {
                if (gamepad2.y) {
                    rail_position = rail_position - 10;
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

            if (gamepad2.a) {
                Rail.setTargetPosition(rail_position);
            }
            if (gamepad2.b) {
                Rail.setTargetPosition(0);
            }

            telemetry.addData("Intake Power", intake_power);
            telemetry.addData("Arm Power", arm_position);
            telemetry.addData("Rail Power", rail_position);
            telemetry.update();

        }
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

    private void AttachmentSetDirection () {

        //BucketServo.setDirection(DcMotor.Direction.FORWARD);
        //CarouselServo.setDirection(DcMotor.Direction.FORWARD);
        BucketServo.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Rail.setDirection(DcMotor.Direction.FORWARD);

    }

}
