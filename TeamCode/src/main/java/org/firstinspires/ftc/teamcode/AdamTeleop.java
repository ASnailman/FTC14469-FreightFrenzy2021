package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="AdamTeleop", group="MechanumDriveTest")
public class AdamTeleop extends LinearOpMode {

    static CRServo CarouselServo;
    static DcMotor Rail;
    static DcMotor Intake;
    static DcMotor FrontLeft;
    static DcMotor BackLeft;
    static DcMotor FrontRight;
    static DcMotor BackRight;

    boolean b_realeased = false;
    boolean x_realeased = false;
    boolean y_realeased = false;
    boolean Up_released = false;
    boolean Dn_released = false;

    double x;
    double y;
    double rx;

    public void runOpMode() {

        CarouselServo = hardwareMap.get(CRServo.class, "CarouselServo");
        CarouselServo.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Intake.setTargetPosition(0);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Rail = hardwareMap.get(DcMotor.class, "Rail");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackRight");

        Rail.setTargetPosition(0);
        Intake.setTargetPosition(0);

        //Rail.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake.setDirection(DcMotorSimple.Direction.FORWARD);
        BackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();

        while (opModeIsActive()) {


            if (b_realeased == false) {
                if (gamepad1.b == true) {

                    //it only runs through the if statement 1 time and then never has to run through it again
                    CarouselServo.setPower(1);

                    b_realeased = true;
                }
            } else {

                if (!gamepad1.b) {
                    b_realeased = false;
                }
            }

            //------------------------------------------------------------------------------------------------------------

            if (x_realeased == false) {
                if (gamepad1.x == true) {

                    CarouselServo.setPower(-1);

                    x_realeased = true;
                }
            } else {

                if (!gamepad1.x) {
                    x_realeased = false;
                }
            }

            //----------------------------------------------------------------------------------------------------------

            if (y_realeased == false) {
                if (gamepad1.y == true) {

                    CarouselServo.setPower(0);

                    y_realeased = true;
                }
            } else {

                if (!gamepad1.y) {
                    y_realeased = false;
                }
            }

            //---------------------------------------------------------------------------------------------------------------
            //--------------------------------------------------------------------------------------------------------------
/*
            int position = Rail.getCurrentPosition();
            telemetry.addData("Encoder Position", position);

            if (Up_released == false){
                if (gamepad1.dpad_up == true);{

                    Rail.setTargetPosition(500);
                    Rail.setPower(-0.5);

                    Up_released = true;

                }
            } else {

                if (!gamepad1.dpad_up){
                    Up_released = false;
                }
            }

            //---------------------------------------------------------------------------------------------------------------

            if (Dn_released == false){
                if (gamepad1.dpad_down == true);{

                    Rail.setTargetPosition(0);
                    Rail.setPower(0.5);

                    Dn_released = true;

                }
            } else {

                if (!gamepad1.dpad_down) {
                    Dn_released = false;

                }
            }
*/
            //-----------------------------------------------------------------------------------------------------------------
            if (gamepad1.dpad_up) {
                Intake.setPower(1);
            }

            //-------------------------------------------------------------------------------------------------------------------

            if (gamepad1.dpad_down) {
                Intake.setPower(0);
            }
            //-----------------------------------------------------------------------------------------------------------------

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            //counteract strafing imperfections(this number must be changed through preference)
            double rx = gamepad1.right_stick_x;

            FrontRight.setPower(y - x - rx / denominator);

            FrontLeft.setPower(y + x + rx / denominator);

            BackRight.setPower(y + x - rx / denominator);

            BackLeft.setPower(y - x + rx / denominator);


        }
    }
}
