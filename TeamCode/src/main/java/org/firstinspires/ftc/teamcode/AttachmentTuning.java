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
    double arm_power = 0.5;
    double rail_power = 0.5;

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
        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Rail = hardwareMap.get(DcMotor.class, "Rail");
        CarouselServo = hardwareMap.get(CRServo.class, "carouselservo");
        BucketServo = hardwareMap.get(Servo.class, "BucketServo");

        AttachmentSetDirection();
        SetDirection(MoveDirection.FORWARD);

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

            /****************************************
             Increase Arm Power (G2): right bumper = power + 0.01, left bumper = power - 0.01
             ***************************************/

            if (button_bumper_right_already_pressed2 == false) {
                if (gamepad2.right_bumper) {
                    arm_power = arm_power + 0.01;
                    button_bumper_right_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.right_bumper) {
                    button_bumper_right_already_pressed2 = false;
                }
            }

            if (button_bumper_left_already_pressed2 == false) {
                if (gamepad2.left_bumper) {
                    arm_power = arm_power - 0.01;
                    button_bumper_left_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.left_bumper) {
                    button_bumper_left_already_pressed2 = false;
                }
            }

            /****************************************
             Run Arm (G2): dpad_up = power, dpad_down = 0
             ***************************************/

            if (gamepad2.dpad_up) {
                Arm.setPower(arm_power);
            }
            if (gamepad2.dpad_down) {
                Arm.setPower(0);
            }

            /****************************************
             Increase Rail Power (G2): Button X = power + 0.01, Button Y = power - 0.01
             ***************************************/

            if (button_x_already_pressed2 == false) {
                if (gamepad2.x) {
                    rail_power = rail_power + 0.01;
                    button_x_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.x) {
                    button_x_already_pressed2 = false;
                }
            }

            if (button_y_already_pressed2 == false) {
                if (gamepad2.y) {
                    rail_power = rail_power - 0.01;
                    button_y_already_pressed2 = true;
                }
            } else {
                if (!gamepad2.y) {
                    button_y_already_pressed2 = false;
                }
            }

            /****************************************
             Run Rail (G2): Button A = power, Button B = 0
             ***************************************/

            if (gamepad2.a) {
                Rail.setPower(rail_power);
            }
            if (gamepad2.b) {
                Rail.setPower(0);
            }

            telemetry.addData("Intake Power", intake_power);
            telemetry.addData("Arm Power", arm_power);
            telemetry.addData("Rail Power", rail_power);
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

        CarouselServo.setDirection(DcMotor.Direction.FORWARD);
        BucketServo.setDirection(Servo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Rail.setDirection(DcMotor.Direction.FORWARD);

    }

}
