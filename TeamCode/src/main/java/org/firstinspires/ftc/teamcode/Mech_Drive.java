package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Mech_Drive {

    DcMotor FrontLeft, FrontRight, BackLeft, BackRight;     // DC motors for each of the Mecanum wheels
    double flpower, frpower, blpower, brpower;              // Power command for each of the DC motors

    PID pid_obj;
    Telemetry telemetry_obj;
    double target_distance;             // Target travel distance (in encoder ticks: +ve or -ve)
    double target_strafing_angle;
    double target_power;
    double SteeringOutput;              // Steering output from PID
    Task_State run_state;               // This is used by the opmode to determine when this task has completed and proceed to the next task


    // CONSTRUCTOR
    public Mech_Drive(DcMotor FL, DcMotor FR, DcMotor BL, DcMotor BR,
                      MoveDirection direction,
                      Telemetry tel_obj) {

        // Assign the motor connected to the bucket and initialize it
        FrontLeft = FL;
        FrontRight = FR;
        BackLeft = BL;
        BackRight = BR;
        SetDirection(direction);

        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create a new PID object to control the bucket
        pid_obj = new PID();
        telemetry_obj = tel_obj;
        run_state = Task_State.INIT;
    }

    // METHOD THAT A STATE MACHINE OPMODE SHOULD CALL WHEN IT IS READY TO LAUNCH THE NEXT TASK IN ITS LIST
    public void SetTarget(double target_D, double target_SA, double target_P) {

        target_distance = target_D;
        target_strafing_angle = target_SA;
        target_power = target_P;
        run_state = Task_State.RUN;
    }

    // THIS IS THE TASK THAT A STATE MACHINE OPMODE SHOULD CALL REPEATEDLY IN ITS LOOP
    public void Task (double gyro_Z_reading) {

        double clipped_cmd;
        double power_x_old, power_x_new;
        double power_y_old, power_y_new;
        double denominator;
        double encoder;
        double radians = Math.toRadians(-target_strafing_angle); // negate strafing angle for left hand rule

        encoder = FrontRight.getCurrentPosition();

        // Always run this PID control when in RUN
        if (run_state == Task_State.RUN) {

            power_x_old = 0;                // make x_old 0 to make the degrees start at the front of the robot
            power_y_old = target_power;

            power_x_new = power_x_old * Math.cos(radians) - power_y_old * Math.sin(radians); // equation for right hand rule
            power_y_new = power_x_old * Math.sin(radians) + power_y_old * Math.cos(radians);
            SteeringOutput = pid_obj.PID_Control(target_strafing_angle, 0.00002, 0, 0, gyro_Z_reading);

            if (encoder < 0) {
                encoder = -encoder;
            }

            if (encoder < target_distance) {

                //if ((radians <= Math.toRadians(90) && radians >= Math.toRadians(0)) || (radians >= Math.toRadians(180) && radians <= Math.toRadians(270))) {
                //encoder = FrontLeft.getCurrentPosition();
                //} else {
                //encoder = BackLeft.getCurrentPosition();
                //}

                denominator = Math.max(Math.abs(power_y_new) + Math.abs(power_x_new), 1);
                flpower = (power_y_new + 1.1 * power_x_new + SteeringOutput) / denominator;
                blpower = (power_y_new - 1.1 * power_x_new + SteeringOutput) / denominator;
                frpower = (power_y_new - 1.1 * power_x_new - SteeringOutput) / denominator;
                brpower = (power_y_new + 1.1 * power_x_new - SteeringOutput) / denominator;

                FrontLeft.setPower(flpower);
                FrontRight.setPower(frpower);
                BackLeft.setPower(blpower);
                BackRight.setPower(brpower);
            }
            else {
                FrontLeft.setPower(0);
                FrontRight.setPower(0);
                BackLeft.setPower(0);
                BackRight.setPower(0);

                run_state = Task_State.DONE;
            }
        }
        else if (run_state == Task_State.DONE){
            run_state = Task_State.READY;
        }

        telemetry_obj.addData("ActualDistance", encoder);
        telemetry_obj.addData("Steering", SteeringOutput);
        telemetry_obj.addData("DirectionZ", gyro_Z_reading);
        telemetry_obj.update();

    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {

        return run_state;
    }

    private void SetDirection (MoveDirection direction) {

        if (direction == MoveDirection.FORWARD) {
            FrontLeft.setDirection(DcMotor.Direction.REVERSE);
            FrontRight.setDirection(DcMotor.Direction.FORWARD);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);
            BackRight.setDirection(DcMotor.Direction.FORWARD);
        } else if (direction == MoveDirection.REVERSE) {
            FrontLeft.setDirection(DcMotor.Direction.FORWARD);
            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.FORWARD);
            BackRight.setDirection(DcMotor.Direction.REVERSE);
        }
    }
}
