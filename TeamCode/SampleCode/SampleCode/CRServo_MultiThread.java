package SampleCode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MoveDirection;

@TeleOp(name="Test", group="MecanumDrive")
public class CRServo_MultiThread extends LinearOpMode {

    static DcMotor FrontLeft;
    static DcMotor BackLeft;
    static DcMotor FrontRight;
    static DcMotor BackRight;
    MoveDirection Direction;
    BNO055IMU IMU;
    static Servo BucketServo;
    double bucketservo_position = 0;

    static CRServo BucketCRServo;
    double bucketCRservo_speed = 0;
    double bucketCRservo_duration = 0;
    int bucketCRservo_direction = 0;

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


    @Override
    public void runOpMode() {

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");

        //BucketServo = hardwareMap.get(Servo.class, "BucketServo");
        BucketCRServo = hardwareMap.get(CRServo.class, "BucketServo");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        //AttachmentSetDirection();
        //SetDirection(MoveDirection.REVERSE);

        BucketCRServoCtrl_Thread cr_thread = new BucketCRServoCtrl_Thread();
        cr_thread.start();


        waitForStart();

        while (opModeIsActive()) {

            /****************************************
             Movement
             ***************************************/

            double y = -gamepad1.left_stick_y * 0.5;
            double x = gamepad1.left_stick_x * 0.55;
            double rx = gamepad1.right_stick_x * 0.5;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx) / denominator;
            double BLPower = (y - x + rx) / denominator;
            double FRPower = (y - x - rx) / denominator;
            double BRPower = (y + x - rx) / denominator;

            //FrontLeft.setPower(FLPower);
            //BackLeft.setPower(BLPower);
            //FrontRight.setPower(FRPower);
            //BackRight.setPower(BRPower);


            /****************************************
             Increase BucketServo Sleep Time (G1): x = increase sleep by 50, y = decrease sleep by 50
             ***************************************/

            if (button_y_already_pressed == false) {
                if (gamepad1.y) {
                    //bucketservo_position = bucketservo_position + 0.1;
                    bucketCRservo_speed = bucketCRservo_speed + 0.1;
                    button_y_already_pressed = true;
                }
            } else {
                if (!gamepad1.y) {
                    button_y_already_pressed = false;
                }
            }

            if (button_a_already_pressed == false) {
                if (gamepad1.a) {
                    //bucketservo_position = bucketservo_position - 0.1;
                    bucketCRservo_speed = bucketCRservo_speed - 0.1;
                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad1.a) {
                    button_a_already_pressed = false;
                }
            }

            if (button_x_already_pressed == false) {
                if (gamepad1.x) {
                    //BucketServo.setDirection(Servo.Direction.REVERSE);
                    bucketCRservo_duration = bucketCRservo_duration - 100;
                    button_x_already_pressed = true;

                }
            } else {
                if (!gamepad1.x) {
                    button_x_already_pressed = false;
                }
            }

            if (button_b_already_pressed == false) {
                if (gamepad1.b) {
                    //BucketServo.setDirection(Servo.Direction.FORWARD);
                    bucketCRservo_duration = bucketCRservo_duration + 100;
                    button_b_already_pressed = true;
                }
            } else {
                if (!gamepad1.b) {
                    button_b_already_pressed = false;
                }
            }

            /****************************************
             Run BucketServo (G2): dpad_right = setposition, dpad_down = setposition 0
             ***************************************/

            if (gamepad1.dpad_right) {

                if (cr_thread.GetRunState() == Boolean.FALSE) {
                    //BucketServo.setPosition(bucketservo_position);
                    cr_thread.SetRunDirection(DcMotorSimple.Direction.FORWARD);
                    cr_thread.SetRunSpeed(bucketCRservo_speed);
                    cr_thread.SetDuration(bucketCRservo_duration);
                    cr_thread.SetRunState(Boolean.TRUE);
                    bucketCRservo_direction = 1;
                }

            }

            if (gamepad1.dpad_left) {
                if (cr_thread.GetRunState() == Boolean.FALSE) {
                    //BucketServo.setPosition(0);
                    cr_thread.SetRunDirection(DcMotorSimple.Direction.REVERSE);
                    cr_thread.SetRunSpeed(bucketCRservo_speed);
                    cr_thread.SetDuration(bucketCRservo_duration);
                    cr_thread.SetRunState(Boolean.TRUE);
                    bucketCRservo_direction = -1;
                }
            }

            //telemetry.addData("time", time);
            //telemetry.addData("BucketServo Position", bucketservo_position);
            telemetry.addData("BucketServo Speed", bucketCRservo_speed);
            telemetry.addData("BucketServo Direction", bucketCRservo_direction);
            telemetry.addData("BucketServo Duration", bucketCRservo_duration);
            telemetry.addData("FLPower", FLPower);
            telemetry.addData("BLPower", BLPower);
            telemetry.addData("FRPower", FRPower);
            telemetry.addData("BRPower", BRPower);
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

        BucketServo.setDirection(Servo.Direction.FORWARD);

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

                        BucketCRServo.setDirection(run_direction);
                        BucketCRServo.setPower(run_speed);

                        if (ET.milliseconds() >= run_duration) {
                            BucketCRServo.setPower(0);
                            run_state = Boolean.FALSE;
                            ET.reset();
                        }
                    }
                }
            }
            catch (Exception e) {
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
