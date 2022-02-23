package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotserver.internal.webserver.RobotControllerWebHandlers;

import java.io.File;

//import java.io.File;

@TeleOp(name="TeleopPracticeRobot", group="MecanumDrive")
public class TeleopPracticeRobot extends LinearOpMode {

    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static RevBlinkinLedDriver ColorStrip;
    static DistanceSensor DistancesensorForward;
    static DistanceSensor DistancesensorRight;
    static ColorSensor Colorsensor;
    static MoveDirection Direction;
    static MoveDirection DiagDirection;
    static Servo CarouselServo;
    BNO055IMU IMU;
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    BNO055IMU ACCIMU;
    Orientation orientation;
    double globalangle;
    double current_value;
    double prev_value = 0;
    double final_value;
    double FRpower;
    double FLpower;
    double BRpower;
    double BLpower;
    double SteeringOutput;
    double power_x_old;
    double power_y_old;

    double movement = 1;

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
    boolean button_dpad_left_already_pressed = false;
    boolean button_dpad_right_already_pressed = false;
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
    boolean double_trigger_already_pressed = false;

    int programorder = 0;
    int programorder2 = 0;

    final int ENEMYHIT = 0;
    final int SPINATTACK = 0;
    final int ZAWARUDO = 0;
    final int TIMESTOP = 0;
    final int MECANUM = 0;
    final int OOF = 0;

    String sounds[] = {"enemyhit"};
    String spinsound[] = {"spinattack"};
    String zawarudo[] = {"zawarudo"};
    String timestop[] = {"timestop"};
    String oof[] = {"oof"};
    String mecanum[] = {"mecanum"};
    String intro[] = {"intro"};


    private String soundPath = "/FIRST/sounds";
    //private File soundAFile = new File(soundPath + "/3-AngryLoud.WAV");
    //private File soundBFile = new File(soundPath + "/2-Alert.WAV");
    //private File soundXFile = new File(soundPath + "/5-SongShort2.mp3");
    //private File soundYFile = new File(soundPath + "/1-BTBTLoud.WAV");
    //private File soundEnemyFile = new File(soundPath + "/enemyhit.WAV");

    boolean soundPlaying = false;

    @Override
    public void runOpMode () {

        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        ColorStrip = hardwareMap.get(RevBlinkinLedDriver.class, "colorstrip");


        IMU = hardwareMap.get(BNO055IMU.class, "imu");
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);

        BackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //while (!isStopRequested() && !IMU.isGyroCalibrated()) {
          //  sleep(50);
            //idle();
        //}

        //telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", IMU.getCalibrationStatus().toString());
        //telemetry.update();

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //PlaySound(mecanum[MECANUM]);

        waitForStart();

        ElapsedTime ET = new ElapsedTime();

        while (opModeIsActive()) {

            double y = -gamepad2.left_stick_y * movement;
            double x = gamepad2.left_stick_x * movement;
            double rx = gamepad2.right_stick_x * movement;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double FLPower = (y + x + rx) / denominator;
            double BLPower = (y - x + rx) / denominator;
            double FRPower = (y - x - rx) / denominator;
            double BRPower = (y + x - rx) / denominator;

            FrontLeft.setPower(FLPower);
            BackLeft.setPower(BLPower);
            FrontRight.setPower(FRPower);
            BackRight.setPower(BRPower);

            //ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

            if (gamepad2.a && !soundPlaying) {
                ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                programorder = 0;
                //programorder2 = 0;
                PlaySound(sounds[ENEMYHIT]);
            } else if (!gamepad2.a) {
                if (programorder !=0) {

                } else {
                    ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                }
            }

            if (button_b_already_pressed == false) {
                if (gamepad2.b) {
                    programorder = 1;
                    button_b_already_pressed = true;
                }
            } else if (!gamepad2.b) {
                button_b_already_pressed = false;
            }

            switch (programorder) {

                case 1:
                    ET.reset();
                    ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    programorder++;
                    break;

                case 2:
                    if (ET.milliseconds() > 500) {
                        ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
                        programorder++;
                    }
                    break;

                case 3:
                    if (ET.milliseconds() > 1000) {
                        ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                        programorder++;
                    }
                    break;
                case 4:
                    if (ET.milliseconds() > 1500) {
                        ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
                        programorder++;
                    }
                    break;
                case 5:
                    if (ET.milliseconds() > 2000) {
                        ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                        programorder++;
                    }
                    break;
                case 6:
                    if (ET.milliseconds() > 2500) {
                        ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.SKY_BLUE);
                        programorder++;
                    }
                    break;
                case 7:
                    if (ET.milliseconds() > 3000) {
                        ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
                        programorder++;
                    }
                    break;
                case 8:
                    if (ET.milliseconds() > 3500) {
                        programorder = 0;
                    }
                    break;

                default:
                    break;
            }

            if (gamepad2.y && !soundPlaying) {
                ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
                //PlaySound(zawarudo[ZAWARUDO]);
                programorder = 9;
            }
            else if (!gamepad2.y) {

            }

            if (gamepad2.x && !soundPlaying) {
                ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);
                PlaySound(timestop[TIMESTOP]);
                programorder = 10;
            }
            else if (!gamepad2.x) {

            }

            if (gamepad2.dpad_left && !soundPlaying) {
                ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                PlaySound(oof[OOF]);
                programorder = 11;
            }
            else if (!gamepad2.dpad_left) {

            }

            if (gamepad2.dpad_right && !soundPlaying) {
                ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_FOREST_PALETTE);
                //PlaySound(mecanum[MECANUM]);
                PlaySound(intro[0]);
                programorder = 12;
            }
            else if (!gamepad2.dpad_right) {

            }

        }
    }

    void PlaySound(String soundFile) {
        int SoundID = -1;

        //--- Configure our Sound Player
        SoundPlayer.PlaySoundParams params = new SoundPlayer.PlaySoundParams();
        params.loopControl = 0;
        params.waitForNonLoopingSoundsToFinish = false;

        // Start playing, when done update soundPlaying variable

        if ((SoundID = hardwareMap.appContext.getResources().getIdentifier(soundFile, "raw", hardwareMap.appContext.getPackageName())) != 0) {
            soundPlaying = true;

            SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, SoundID, params, null,
                    () -> { soundPlaying = false;
                            SoundPlayer.getInstance().stopPlayingAll();
                            ColorStrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);} );
        }

    }

        private void MotorTurn ( double FR, double FL, double BR, double BL){

            FrontRight.setPower(FR);
            FrontLeft.setPower(FL);
            BackRight.setPower(BR);
            BackLeft.setPower(BL);

        }

        private void SetMotorPower ( double x){

            FrontLeft.setPower(x);
            FrontRight.setPower(x);
            BackLeft.setPower(x);
            BackRight.setPower(x);

        }

        private int WhiteDetector () {

            Colorsensor.red();
            Colorsensor.green();
            Colorsensor.blue();

            int White = 255;
            int Unknown = 1;

            telemetry.addData("Red", Colorsensor.red());
            telemetry.addData("Green", Colorsensor.green());
            telemetry.addData("Blue", Colorsensor.blue());

            if (Colorsensor.red() >= 190 && Colorsensor.green() >= 190 && Colorsensor.blue() >= 190) {

                telemetry.addData("Color:", "White");
                telemetry.update();
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
                return White;

            } else {

                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                FrontLeft.setPower(0.8);
                FrontRight.setPower(0.8);
                BackLeft.setPower(0.8);
                BackRight.setPower(0.8);
                return Unknown;

            }
        }

        private int RedDetector () {

            Colorsensor.red();
            Colorsensor.green();
            Colorsensor.blue();

            int Red = 255;
            int Unknown = 1;

            telemetry.addData("Red", Colorsensor.red());
            telemetry.addData("Green", Colorsensor.green());
            telemetry.addData("Blue", Colorsensor.blue());

            if (Colorsensor.red() >= 190 && Colorsensor.green() <= 40 && Colorsensor.blue() <= 40) {

                telemetry.addData("Color:", "Red");
                telemetry.update();
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);
                return Red;

            } else {

                telemetry.addData("Color:", "Unknown");
                telemetry.update();
                FrontLeft.setPower(1);
                FrontRight.setPower(-1);
                BackLeft.setPower(-1);
                BackRight.setPower(1);
                return Unknown;

            }
        }

        //private void ServoPosition (double position) {

        //CarouselServo.scaleRange(0, 1);
        //CarouselServo.setPosition(position);
        //telemetry.addData("Position", CarouselServo.getPosition());

        //}

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

        private void DiagonalMovement (MoveDirection diagdirection,double Fpower, double Bpower,
        long sleep){

            DiagDirection = diagdirection;
            FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (DiagDirection == MoveDirection.TOPDIAGRIGHT) {
                FrontLeft.setPower(Fpower);
                BackRight.setPower(Bpower);
                sleep(sleep);
                FrontLeft.setPower(0);
                BackRight.setPower(0);
            } else if (DiagDirection == MoveDirection.TOPDIAGLEFT) {
                FrontRight.setPower(Fpower);
                BackLeft.setPower(Bpower);
                sleep(sleep);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
            } else if (DiagDirection == MoveDirection.BOTTOMDIAGRIGHT) {
                FrontRight.setPower(-Fpower);
                BackLeft.setPower(-Bpower);
                sleep(sleep);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
            } else if (DiagDirection == MoveDirection.BOTTOMDIAGLEFT) {
                FrontLeft.setPower(-Fpower);
                BackRight.setPower(-Bpower);
                sleep(sleep);
                FrontLeft.setPower(0);
                BackRight.setPower(0);
            } else if (DiagDirection == MoveDirection.SIDEWAYSRIGHT) {
                FrontLeft.setPower(Fpower);
                FrontRight.setPower(-Fpower);
                BackLeft.setPower(-Bpower);
                BackRight.setPower(Bpower);
                sleep(sleep);
                FrontLeft.setPower(0);
                BackRight.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
            } else if (DiagDirection == MoveDirection.SIDEWAYSLEFT) {
                FrontLeft.setPower(-Fpower);
                FrontRight.setPower(Fpower);
                BackLeft.setPower(Bpower);
                BackRight.setPower(-Bpower);
                sleep(sleep);
                FrontLeft.setPower(0);
                BackRight.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
            }
        }

        private double GyroContinuity () {

            orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            current_value = orientation.firstAngle;

            final_value = current_value - prev_value;

            if (final_value < -180)
                final_value += 360;
            else if (final_value > 180)
                final_value -= 360;

            globalangle += final_value;

            prev_value = current_value;

            return -globalangle;

        }

        private void GyroTurn ( double angledegree, double power){

            SetDirection(MoveDirection.FORWARD);

            if (angledegree > GyroContinuity()) {

                while (GyroContinuity() <= angledegree) {

                    MotorTurn(-power, power, -power, power);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }

                // If a fast turn is requested, then dual stage-turning is needed to correct overshoot
                if (power > 0.5) {
                    while (GyroContinuity() >= angledegree) {

                        MotorTurn(0.5, -0.5, 0.5, -0.5);

                        telemetry.addData("Gyro", GyroContinuity());
                        telemetry.update();

                    }
                }

            } else {

                while (GyroContinuity() >= angledegree) {

                    MotorTurn(power, -power, power, -power);

                    telemetry.addData("Gyro", GyroContinuity());
                    telemetry.update();

                }

                // If a fast turn is requested, then dual stage-turning is needed to correct overshoot
                if (power > 0.5) {
                    while (GyroContinuity() <= angledegree) {

                        MotorTurn(-0.5, 0.5, -0.5, 0.5);

                        telemetry.addData("Gyro", GyroContinuity());
                        telemetry.update();

                    }
                }
            }
            SetMotorPower(0);
            sleep(200);
        }

        private void DirectionFollower2 ( double targetdistance, double power,
        double TargetDirection,
        double kp_in, double ki_in, double kd_in){

            PID PID = new PID();
            FRpower = power;
            FLpower = power;
            BRpower = power;
            BLpower = power;

            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while (FrontRight.getCurrentPosition() < targetdistance) {

                SetDirection(MoveDirection.FORWARD);

                //SteeringOutput = PID.PID_Control(TargetDirection, 0.0001, 0.0000000000001, 0.00000003, GyroContinuity());
                //SteeringOutput = PID.PID_Control(TargetDirection, 0.003, 0.00001, 0.0003, GyroContinuity());
                SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
                FLpower = FLpower + SteeringOutput * FLpower;
                FRpower = FRpower - SteeringOutput * FRpower;
                BLpower = BLpower + SteeringOutput * BLpower;
                BRpower = BRpower - SteeringOutput * BRpower;

                FrontLeft.setPower(FLpower);
                FrontRight.setPower(FRpower);
                BackLeft.setPower(BLpower);
                BackRight.setPower(BRpower);

                telemetry.addData("Error", PID.error);
                telemetry.addData("TargetDirection", TargetDirection);
                telemetry.addData("Encoder", FrontRight.getCurrentPosition());
                telemetry.addData("Front_Left_Motor_Power", FLpower);
                telemetry.addData("Front_Right_Motor_Power", FRpower);
                telemetry.addData("Back_Left_Motor_Power", BLpower);
                telemetry.addData("Back_Right_Motor_Power", BRpower);
                telemetry.addData("Steering", SteeringOutput);
                telemetry.addData("DirectionZ", GyroContinuity());
                telemetry.addData("DirectionX", orientation.thirdAngle);
                telemetry.addData("DirectionY", orientation.secondAngle);
                telemetry.update();

            }

            while (-FrontRight.getCurrentPosition() > targetdistance) {

                SetDirection(MoveDirection.REVERSE);

                SteeringOutput = PID.PID_Control(TargetDirection, kp_in, ki_in, kd_in, GyroContinuity());
                FLpower = FLpower - SteeringOutput * FLpower;
                FRpower = FRpower + SteeringOutput * FRpower;
                BLpower = BLpower - SteeringOutput * BLpower;
                BRpower = BRpower + SteeringOutput * BRpower;

                FrontLeft.setPower(FLpower);
                FrontRight.setPower(FRpower);
                BackLeft.setPower(BLpower);
                BackRight.setPower(BRpower);

                telemetry.addData("Encoder", -FrontRight.getCurrentPosition());
                telemetry.addData("Front_Left_Motor_Power", FLpower);
                telemetry.addData("Front_Right_Motor_Power", FRpower);
                telemetry.addData("Back_Left_Motor_Power", BLpower);
                telemetry.addData("Back_Right_Motor_Power", BRpower);
                telemetry.addData("Steering", SteeringOutput);
                telemetry.addData("DirectionZ", GyroContinuity());
                telemetry.addData("DirectionX", orientation.thirdAngle);
                telemetry.addData("DirectionY", orientation.secondAngle);
                telemetry.update();

            }

            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SetMotorPower(0);
            //sleep(100);
            GyroTurn(TargetDirection, 0.3);

        }

        private void MechDrive ( double strafingangle, double power, double targetdistance){

            double power_y_new;
            double power_x_new;
            double encoder;
            double radians = Math.toRadians(strafingangle);

            power_y_old = 0;
            power_x_old = power;

            power_y_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians);
            power_x_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);

            if ((radians <= Math.toRadians(90) && radians >= 0) || (radians >= Math.toRadians(180) && radians <= Math.toRadians(270))) {
                encoder = FrontLeft.getCurrentPosition();
            } else {
                encoder = BackLeft.getCurrentPosition();
            }

            while (encoder < targetdistance) {

                double denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
                double flpower = (power_y_new + 1.1 * power_x_new) / denominator;
                double blpower = (power_y_new - 1.1 * power_x_new) / denominator;
                double frpower = (power_y_new - 1.1 * power_x_new) / denominator;
                double brpower = (power_y_new + 1.1 * power_x_new) / denominator;

                FrontLeft.setPower(flpower);
                FrontRight.setPower(frpower);
                BackLeft.setPower(blpower);
                BackRight.setPower(brpower);

                telemetry.addData("Frontleft", FrontLeft.getCurrentPosition());
                telemetry.addData("Backleft", -BackLeft.getCurrentPosition());
                telemetry.addData("ActualDistance", encoder);
                telemetry.update();

            }

            FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SetMotorPower(0);
            sleep(100);

        }

        private void WallDetectorForwards ( int Distance){

            while (DistancesensorForward.getDistance(DistanceUnit.INCH) > Distance) {

                if (DistancesensorForward.getDistance(DistanceUnit.INCH) > Distance) {

                    SetMotorPower(0.5);
                    telemetry.addData("Distance", DistancesensorForward.getDistance(DistanceUnit.INCH));
                    telemetry.update();

                } else {

                    SetMotorPower(0);

                }
            }
        }

        private void WallDetectorCoordinateY ( int y){

            while (DistancesensorForward.getDistance(DistanceUnit.INCH) > y) {

                if (DistancesensorForward.getDistance(DistanceUnit.INCH) > y) {

                    SetMotorPower(0.5);
                    telemetry.addData("Distance", DistancesensorForward.getDistance(DistanceUnit.INCH));
                    telemetry.update();

                } else {

                    SetMotorPower(0);

                }
            }
        }

        private void WallDetectorCoordinateX ( int x){

            if (DistancesensorRight.getDistance(DistanceUnit.INCH) > x) {

                MotorTurn(-0.5, 0.5, 0.5, -0.5);
                telemetry.addData("Distance", DistancesensorRight.getDistance(DistanceUnit.INCH));
                telemetry.update();

            } else {

                SetMotorPower(0);

            }
        }

        private void AutoGridpoint ( int x, int y){

            WallDetectorCoordinateX(x);
            WallDetectorCoordinateY(y);

        }
}

