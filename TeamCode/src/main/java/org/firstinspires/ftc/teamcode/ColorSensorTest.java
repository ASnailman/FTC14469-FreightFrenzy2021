package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="ColorTest", group="heehee")
public class ColorSensorTest extends LinearOpMode {

    static NormalizedColorSensor colorsensor;
    static Servo ColorStrip;
    BNO055IMU IMU;
    double strip_color;

    boolean button_a_already_pressed = false;
    boolean button_b_already_pressed = false;

    @Override
    public void runOpMode() {

        colorsensor = hardwareMap.get(NormalizedColorSensor.class, "colorsensor");
        ColorStrip = hardwareMap.get(Servo.class, "colorstrip");
        IMU = hardwareMap.get(BNO055IMU.class, "imu");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y) {
                WhiteDetector();
            }

            if (gamepad1.right_bumper) {
                YellowDetector();
            }

            if (button_a_already_pressed == false) {
                if (gamepad1.a) {
                    strip_color = strip_color + 0.01;
                    button_a_already_pressed = true;
                }
            } else {
                if (!gamepad1.a) {
                    button_a_already_pressed = false;
                }
            }

            if (button_b_already_pressed == false) {
                if (gamepad1.b) {
                    strip_color = strip_color - 0.01;
                    button_b_already_pressed = true;
                }
            } else {
                if (!gamepad1.b) {
                    button_b_already_pressed = false;
                }
            }

            if (gamepad1.x) {
                ColorStrip.setPosition(strip_color);
            }

            telemetry.addData("Colorstrip", strip_color);
            telemetry.update();

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

        if (HSV[1] <= 0.25) {
            if (HSV[2] >= 0.93) {
                telemetry.addData("Color:", "White");
                telemetry.update();
                return White;
            } else {
                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                return Unkwown;
            }
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
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

        if (HSV[1] >= 0) {
            if (HSV[2] >= 0.8) {
                telemetry.addData("Color:", "Yellow");
                telemetry.update();
                return Yellow;
            } else {
                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                return Unkwown;
            }
        } else {
            telemetry.addData("Color:", "Unknown");
            telemetry.update();
            return Unkwown;
        }
    }

}
