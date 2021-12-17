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
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FTCTrainingRobot", group="MecanumDriveTest")
public class FTCTrainingRobot extends LinearOpMode {

    static NormalizedColorSensor colorsensor;
    static RevBlinkinLedDriver ColorStrip;
    BNO055IMU IMU;
    double strip_color;
    boolean white;
    boolean yellow;
    boolean unknown;

    static DcMotor bucket_motor;
    Bucket_Control bucket_control_task;
    double bucket_target = 60;

    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;
    boolean button_bumper_right_already_pressed = false;
    boolean button_bumper_left_already_pressed = false;

    @Override
    public void runOpMode() {

        //colorsensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
        //ColorStrip = hardwareMap.get(RevBlinkinLedDriver.class, "colorstrip");
        //IMU = hardwareMap.get(BNO055IMU.class, "imu");

        bucket_motor = hardwareMap.get(DcMotor.class, "bucketmotor");
        bucket_control_task = new Bucket_Control(bucket_motor);

        waitForStart();

        while (opModeIsActive()) {

            /* WhiteYellowDetector();

            if (white) {
                ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
            }

            if (yellow) {
                ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
            }

            if (unknown) {
                ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            } */

            // TURN BUCKET CLOCKWISE
            if (button_bumper_right_already_pressed == false) {
                if (gamepad1.right_bumper) {

                    // Don't allow a new target position to be requested if the task is still in RUN mode
                    if (bucket_control_task.GetTaskState() == Task_State.INIT ||
                            bucket_control_task.GetTaskState() == Task_State.DONE ||
                            bucket_control_task.GetTaskState() == Task_State.OVERRIDE) {

                        // Each button press moves the bucket by +60 encoder ticks
                        bucket_target = bucket_target + 60;
                        bucket_control_task.SetTargetPosition(bucket_target);
                    }

                    button_bumper_right_already_pressed = true;
                }
            } else {
                if (!gamepad1.right_bumper) {
                    button_bumper_right_already_pressed = false;
                }
            }

            // TURN BUCKET COUNTER CLOCKWISE
            if (button_bumper_left_already_pressed == false) {
                if (gamepad1.left_bumper) {

                    // Don't allow a new target position to be requested if the task is still in RUN mode
                    if (bucket_control_task.GetTaskState() == Task_State.INIT ||
                            bucket_control_task.GetTaskState() == Task_State.DONE ||
                            bucket_control_task.GetTaskState() == Task_State.OVERRIDE) {

                        // Each button press moves the bucket by -60 encoder ticks
                        bucket_target = bucket_target - 60;
                        bucket_control_task.SetTargetPosition(bucket_target);
                    }

                    button_bumper_left_already_pressed = true;
                }
            } else {
                if (!gamepad1.left_bumper) {
                    button_bumper_left_already_pressed = false;
                }
            }

            // OVERRIDE BUCKET MOTOR POWER
            if (button_a_already_pressed == false) {
                if (gamepad1.a) {

                    bucket_control_task.OvControl(0);
                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad1.a) {
                    button_a_already_pressed = false;
                }
            }


            bucket_control_task.Task();

            //telemetry.addData("Colorstrip", ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN));
            telemetry.addData("motor command", bucket_motor.getPower());
            telemetry.addData("bucket position", bucket_motor.getCurrentPosition() );
            telemetry.update();

        }
    }

    private int WhiteYellowDetector() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = colorsensor.getNormalizedColors();
        colorsensor.setGain(70);

        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addData("H:", HSV[0]);
        telemetry.addData("S:", HSV[1]);
        telemetry.addData("V:", HSV[2]);

        int Yellow = 2;
        int White = 1;
        int Unkwown = 0;

        if (HSV[1] >= 0 && HSV[1] <= 0.25) {
            if (HSV[2] >= 0.93 && HSV[2] <= 1) {
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
        } else if (HSV[1] >= 0.4 && HSV[1] <= 0.8) {
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

    private int WhiteDetector() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = colorsensor.getNormalizedColors();
        colorsensor.setGain(70);

        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addData("H:", HSV[0]);
        telemetry.addData("S:", HSV[1]);
        telemetry.addData("V:", HSV[2]);

        int White = 1;
        int Unkwown = 0;

        if (HSV[1] >= 0 && HSV[1] <= 0.25) {
            if (HSV[2] >= 0.93 && HSV[2] <= 1) {
                telemetry.addData("Color:", "White");
                telemetry.update();
                white = true;
                return White;
            } else {
                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                unknown = true;
                return Unkwown;
            }
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            unknown = true;
            return Unkwown;
        }
    }

    private int YellowDetector() {

        float[] HSV = new float[3];
        NormalizedRGBA RGBA = colorsensor.getNormalizedColors();
        colorsensor.setGain(70);
        Color.colorToHSV(RGBA.toColor(), HSV);
        telemetry.addData("H:", HSV[0]);
        telemetry.addData("S:", HSV[1]);
        telemetry.addData("V:", HSV[2]);

        int Yellow = 1;
        int Unkwown = 0;

        if (HSV[1] >= 0.4 && HSV[1] <= 0.8) {
            if (HSV[2] >= 0.6 && HSV[2] <= 1) {
                telemetry.addData("Color:", "Yellow");
                telemetry.update();
                yellow = true;
                return Yellow;
            } else {
                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                unknown = true;
                return Unkwown;
            }
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            unknown = true;
            return Unkwown;
        }
    }

}
