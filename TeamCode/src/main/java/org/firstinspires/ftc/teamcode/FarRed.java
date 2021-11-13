package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="FarRed", group="MecanumDrive")
public class FarRed extends LinearOpMode {

    static DcMotor BackLeft;
    static DcMotor BackRight;
    static DcMotor FrontLeft;
    static DcMotor FrontRight;
    static DcMotor Arm;
    static DcMotor Rail;
    static DcMotor Intake;
    static DcMotor CarouselMotor;
    static CRServo BucketServo;
    static MoveDirection Direction;
    static MoveDirection DiagDirection;
    static DistanceSensor DistancesensorForward;
    static DistanceSensor DistancesensorRight;
    static ColorSensor Colorsensor;
    static Servo BackServo;
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
    ElapsedTime ET = new ElapsedTime();
    boolean servo_power;

    public void runOpMode () {

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

        AttachmentSetDirection();
        SetDirection(MoveDirection.REVERSE);
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        IMU.initialize(parameters);

        orientation = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalangle = 0;
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rail.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Arm.setTargetPosition(0);
        Rail.setTargetPosition(0);

        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {

            MechDrive(-90, 0.4, 1000, 0.00002, 0, 0);
            MechDrive(180, 0.4, 800, 0.00002, 0, 0);
            //put pre load
            TopBucketPosition(-350);
            BucketServoLeft();
            ResetBucketPosition();
            //run intake (not meet one)
            MechDrive(0, 0.6, 2600, 0.00002, 0, 0);
            Rail.setTargetPosition(0);
            Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Rail.setPower(0.5);
            //MechDrive(180, 0.4, 2172.3, 0.00002, 0, 0);
            //MechDrive(-90, 0.4, 543.1, 0.00002, 0, 0);
            //place freight top level (not meet one)
            //MechDrive(90, 0.4, 543.1, 0.00002, 0, 0);
            //MechDrive(-90, 0.4, 1355, 0.00002, 0, 0);
            //MechDrive(0, 0.4, 1086.2, 0.00002, 0, 0);
            break;

        }
    }

    private void AttachmentSetDirection () {

        CarouselMotor.setDirection(DcMotor.Direction.REVERSE);
        BucketServo.setDirection(CRServo.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        Arm.setDirection(DcMotor.Direction.FORWARD);
        Rail.setDirection(DcMotor.Direction.FORWARD);

    }

    private void SensorInitialize (ColorSensor color_sensor, Servo back_servo) {

        Colorsensor = color_sensor;
        BackServo = back_servo;

    }

    private void MotorTurn(double FR, double FL, double BR, double BL) {

        FrontRight.setPower(FR);
        FrontLeft.setPower(FL);
        BackRight.setPower(BR);
        BackLeft.setPower(BL);

    }

    private void SetMotorPower(double x) {

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

    private void ServoPosition (double position) {

        BackServo.scaleRange(0, 1);
        BackServo.setPosition(position);
        telemetry.addData("Position", BackServo.getPosition());

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

    private void DiagonalMovement (MoveDirection diagdirection, double Fpower, double Bpower, long sleep) {

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

    private double GyroContinuity() {

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

    private void GyroTurn (double angledegree, double power) {

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

    private void DirectionFollower2(double targetdistance, double power, double TargetDirection,
                                    double kp_in, double ki_in, double kd_in) {

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

    private void MechDrive (double strafingangle, double power, double targetdistance, double kp_in, double ki_in, double kd_in) {

        PID pid = new PID();
        double power_y_new;
        double power_x_new;
        double encoder;
        double radians = Math.toRadians(-strafingangle); // negate strafing angle for left hand rule

        power_y_old = power; // make x_old 0 to make the degrees start at the front of the robot
        power_x_old = 0;

        power_x_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians); // equation for right hand rule
        power_y_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);
        SteeringOutput = pid.PID_Control(strafingangle, kp_in, ki_in, kd_in, GyroContinuity());

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //if ((radians <= Math.toRadians(90) && radians >= Math.toRadians(0)) || (radians >= Math.toRadians(180) && radians <= Math.toRadians(270))) {
            //encoder = FrontLeft.getCurrentPosition();
        //} else {
            //encoder = BackLeft.getCurrentPosition();
        //}
        encoder = FrontRight.getCurrentPosition();
        if (encoder < 0) {
            encoder = -encoder;
        }

        while (encoder < targetdistance) {

            //if ((radians <= Math.toRadians(90) && radians >= Math.toRadians(0)) || (radians >= Math.toRadians(180) && radians <= Math.toRadians(270))) {
                //encoder = FrontLeft.getCurrentPosition();
            //} else {
                //encoder = BackLeft.getCurrentPosition();
            //}
            encoder = FrontRight.getCurrentPosition();
            if (encoder < 0) {
                encoder = -encoder;
            }

            double denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
            double flpower = (power_y_new + 1.1*power_x_new + SteeringOutput) / denominator;
            double blpower = (power_y_new - 1.1*power_x_new + SteeringOutput) / denominator;
            double frpower = (power_y_new - 1.1*power_x_new - SteeringOutput) / denominator;
            double brpower = (power_y_new + 1.1*power_x_new - SteeringOutput) / denominator;

            FrontLeft.setPower(flpower);
            FrontRight.setPower(frpower);
            BackLeft.setPower(blpower);
            BackRight.setPower(brpower);

            //telemetry.addData("Frontleft", FrontLeft.getCurrentPosition());
            //telemetry.addData("Backleft", BackLeft.getCurrentPosition());
            telemetry.addData("ActualDistance", encoder);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        sleep(100);

    }

    private void MechDriveElapsedTime (double strafingangle, double power, double target_time, double kp_in, double ki_in, double kd_in) {

        PID pid = new PID();
        double power_y_new;
        double power_x_new;
        double encoder;
        double radians = Math.toRadians(-strafingangle); // negate strafing angle for left hand rule
        ET.reset();

        power_y_old = power; // make x_old 0 to make the degrees start at the front of the robot
        power_x_old = 0;

        power_x_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians); // equation for right hand rule
        power_y_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);
        SteeringOutput = pid.PID_Control(strafingangle, kp_in, ki_in, kd_in, GyroContinuity());

        if ((radians <= Math.toRadians(90) && radians >= 0) || (radians >= Math.toRadians(180) && radians <= Math.toRadians(270))) {
            encoder = FrontLeft.getCurrentPosition();
        } else {
            encoder = BackLeft.getCurrentPosition();
        }

        while (ET.milliseconds() < target_time) {

            if ((radians <= Math.toRadians(90) && radians >= Math.toRadians(0)) || (radians >= Math.toRadians(180) && radians <= Math.toRadians(270))) {
                encoder = FrontLeft.getCurrentPosition();
            } else {
                encoder = BackLeft.getCurrentPosition();
            }

            double denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
            double flpower = (power_y_new + 1.1*power_x_new + SteeringOutput) / denominator;
            double blpower = (power_y_new - 1.1*power_x_new + SteeringOutput) / denominator;
            double frpower = (power_y_new - 1.1*power_x_new - SteeringOutput) / denominator;
            double brpower = (power_y_new + 1.1*power_x_new - SteeringOutput) / denominator;

            FrontLeft.setPower(flpower);
            FrontRight.setPower(frpower);
            BackLeft.setPower(blpower);
            BackRight.setPower(brpower);

            telemetry.addData("Frontleft", FrontLeft.getCurrentPosition());
            telemetry.addData("Backleft", -BackLeft.getCurrentPosition());
            telemetry.addData("ActualDistance", encoder);
            telemetry.addData("Steering", SteeringOutput);
            telemetry.addData("DirectionZ", GyroContinuity());
            telemetry.addData("DirectionX", orientation.thirdAngle);
            telemetry.addData("DirectionY", orientation.secondAngle);
            telemetry.update();

        }

        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetMotorPower(0);
        sleep(100);
        ET.reset();

    }

    private void WallDetectorForwards (int Distance) {

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

    private void WallDetectorCoordinateY (int y) {

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

    private void WallDetectorCoordinateX (int x) {

        if (DistancesensorRight.getDistance(DistanceUnit.INCH) > x) {

            MotorTurn(-0.5, 0.5, 0.5, -0.5);
            telemetry.addData("Distance", DistancesensorRight.getDistance(DistanceUnit.INCH));
            telemetry.update();

        } else {

            SetMotorPower(0);

        }
    }

    private void AutoGridpoint (int x, int y) {

        WallDetectorCoordinateX(x);
        WallDetectorCoordinateY(y);

    }

    private void TopBucketPosition (int arm_position) {

        Rail.setTargetPosition(1000);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setPower(0.5);
        ET.reset();
        while (ET.milliseconds() < 2000) {
        }
        Arm.setTargetPosition(arm_position);
        //350 encoder
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.2);
        ET.reset();
        while (ET.milliseconds() < 1000) {
        }
    }

    private void MiddleBucketPosition (int arm_position) {

        Rail.setTargetPosition(650);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setPower(0.5);
        ET.reset();
        while (ET.milliseconds() < 2000) {
        }
        Arm.setTargetPosition(arm_position);
        //250 encoder
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.2);
        ET.reset();
        while (ET.milliseconds() < 1000) {
        }
    }

    private void LowBucketPosition (int arm_position) {

        Rail.setTargetPosition(650);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setPower(0.5);
        ET.reset();
        while (ET.milliseconds() < 2000) {
        }
        Arm.setTargetPosition(arm_position);
        //150 encoder
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.2);
        ET.reset();
        while (ET.milliseconds() < 1000) {
        }
    }

    private void BucketServoRight () {
        ET.reset();
        while (ET.milliseconds() < 850) {
            BucketServo.setPower(-0.5);
        }
        BucketServo.setPower(0);
        servo_power = false;
        ET.reset();
        while (ET.milliseconds() < 1000) {
        }
    }

    private void BucketServoLeft () {
        ET.reset();
        while (ET.milliseconds() < 850) {
            BucketServo.setPower(0.5);
        }
        BucketServo.setPower(0);
        servo_power = true;
        ET.reset();
        while (ET.milliseconds() < 1000) {
        }
    }

    private void ResetBucketPosition () {
        if (servo_power == true) {
            BucketServo.setPower(-0.5);
            sleep(855);
            BucketServo.setPower(0);
        } else if (servo_power == false) {
            BucketServo.setPower(0.5);
            sleep(855);
            BucketServo.setPower(0);
        }
        Arm.setTargetPosition(0);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(0.2);
        ET.reset();
        while (ET.milliseconds() < 2000) {
        }
        Rail.setTargetPosition(400);
        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Rail.setPower(0.5);
        ET.reset();
        while (ET.milliseconds() < 1000) {
        }
    }

}

