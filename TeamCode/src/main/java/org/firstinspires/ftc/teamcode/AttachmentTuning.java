package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="AttachmentTuning", group="MecanumDrive")
public class AttachmentTuning extends LinearOpMode {

    static DcMotor FrontLeft;
    static DcMotor BackLeft;
    static DcMotor FrontRight;
    static DcMotor BackRight;
    static MoveDirection Direction;
    static DcMotor Intake;
    static CRServo CarouselServo;
    double intake_speed = 0.5;

    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_bumper_left_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;

    @Override
    public void runOpMode() {

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        CarouselServo = hardwareMap.get(CRServo.class, "carouselservo");

        CarouselServo.setDirection(DcMotorSimple.Direction.FORWARD);
        SetDirection(MoveDirection.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            if (button_bumper_right_already_pressed == false) {
                if (gamepad1.right_bumper) {
                    intake_speed = intake_speed + 0.01;
                    button_bumper_right_already_pressed = true;
                }
            } else {
                if (!gamepad1.right_bumper) {
                    button_bumper_right_already_pressed = false;
                }
            }

            if (button_bumper_left_already_pressed == false) {
                if (gamepad1.left_bumper) {
                    intake_speed = intake_speed - 0.01;
                    button_bumper_left_already_pressed = true;
                }
            } else {
                if (!gamepad1.left_bumper) {
                    button_bumper_left_already_pressed = false;
                }
            }

            if (gamepad1.dpad_up) {
                IntakeRunner(intake_speed);
            } //runs intake with speed based on adjustments from pressing bumpers

            if (gamepad1.dpad_down) {
                IntakeRunner(0);
            } //sets the intake power to 0, stops motor

            if (gamepad1.dpad_right) {
                CarouselServo.setPower(1);
            } //sets power 1, forwards
            if (gamepad1.dpad_left) {
                CarouselServo.setPower(0);
            } //sets power 0, still

            telemetry.addData("Drive Speed", intake_speed);
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

    private void IntakeRunner (double power) {

        Intake.setPower(power);

    }

}
