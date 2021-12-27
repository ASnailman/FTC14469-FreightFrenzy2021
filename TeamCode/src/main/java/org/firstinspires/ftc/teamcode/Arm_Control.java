package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Arm_Control {

    DcMotor motor_obj;                  // the motor connected to the bucket
    PID pid_obj;
    double target_position;             // Target position for the bucket (in encoder ticks: +ve or -ve)
    double max;
    double min;
    double cmd;                         // Power command for the DC motor
    ElapsedTime et;                     // ElapsedTime object (only used during calibration)
    final double tolerance = 15;        // How close should we be within the target bucket position before saying we're done
    Task_State run_state;               // This is used by the opmode to determine when this task has completed and proceed to the next task

    // CONSTRUCTOR
    public Arm_Control(DcMotor motor) {

        // Assign the motor connected to the bucket and initialize it
        motor_obj = motor;
        motor_obj.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_obj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create ElapsedTime object (only used during calibration)
        et = new ElapsedTime();

        // Create a new PID object to control the bucket
        pid_obj = new PID();
        run_state = Task_State.INIT;
    }

    // METHOD THAT A STATE MACHINE OPMODE SHOULD CALL WHEN IT IS READY TO LAUNCH THE NEXT TASK IN ITS LIST
    public void SetTargetPosition(double target, double negpower, double pospower) {

        target_position = target;
        max = pospower;
        min = negpower;
        run_state = Task_State.RUN;
        pid_obj.Reset_PID();
    }

    // METHOD TO CALIBRATE THE BUCKET POSITION.
    // OVERRIDES THE PID BUCKET CONTROL'S OUTPUT TO ZERO TO ALLOW MANUAL ADJUSTMENT OF BUCKET POSITION
    // ONCE CALIBRATION IS DONE, THE BUCKET WILL RETURN TO ITS TARGET POSITION

    public void Calibrate() {
        et.reset();
        run_state = Task_State.CALIBRATE;
    }

    // THIS IS THE TASK THAT A STATE MACHINE OPMODE SHOULD CALL REPEATEDLY IN ITS LOOP
    public void ArmTask () {

        double clipped_cmd;

        // Always run this PID control when in RUN, DONE or READY mode
        if (run_state == Task_State.RUN || run_state == Task_State.DONE || run_state == Task_State.READY) {

            // 0.07, 0.000001, 0.000005 (these are the best gains for accurate position and few jitters
            cmd = pid_obj.PID_Control(target_position, 0.03, 0.000001, 0.000005, motor_obj.getCurrentPosition() );

            // Don't let the motor run too fast. Otherwise, it will overshoot
            clipped_cmd = Range.clip(cmd, min, max);
            motor_obj.setPower(clipped_cmd);

            // If the bucket is within range of the target position, treat the task as done so that the opmode can move on to
            // the next task in its list
            if (run_state == Task_State.DONE) {
                run_state = Task_State.READY;
            }
            else if (run_state == Task_State.RUN && motor_obj.getCurrentPosition() > (target_position - tolerance) &&
                    motor_obj.getCurrentPosition() < (target_position + tolerance)) {
                run_state = Task_State.DONE;
            }
        }
        else if (run_state == Task_State.CALIBRATE) {
            motor_obj.setPower(0);

            if (et.milliseconds() >= 1000) {

                // Reset the bucket's DC motor encoder
                motor_obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Reset the PID object (otherwise, the PID will still have leftover
                // memory of what it previously did which may cause bad commands from carrying forward)
                pid_obj.Reset_PID();
                run_state = Task_State.DONE;
                motor_obj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {

        return run_state;
    }
}
