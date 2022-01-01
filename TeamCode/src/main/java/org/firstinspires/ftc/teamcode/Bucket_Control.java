package org.firstinspires.ftc.teamcode;

import androidx.core.app.ServiceCompat;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Bucket_Control {

    DcMotor bucketmotor;                // the motor connected to the bucket
    PID pid;
    double targetposition;              // Target position for the bucket (in encoder ticks: +ve or -ve)
    double bucketpower;                 // Power command for the DC motor
    ElapsedTime et;                     // ElapsedTime object (only used during calibration)
    final double tolerance = 15;        // How close should we be within the target bucket position before saying we're done
    Task_State state;                   // This is used by the opmode to determine when this task has completed and proceed to the next task

    // CONSTRUCTOR
    public Bucket_Control(DcMotor BucketMotor) {

        // Assign the motor connected to the bucket and initialize it
        bucketmotor = BucketMotor;
        bucketmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        bucketmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create ElapsedTime object (only used during calibration)
        et = new ElapsedTime();
        // Create a new PID object to control the bucket
        pid = new PID();

        state = Task_State.INIT;
    }

    // METHOD THAT A STATE MACHINE OPMODE SHOULD CALL WHEN IT IS READY TO LAUNCH THE NEXT TASK IN ITS LIST
    public void SetTargetPosition(double TargetPosition) {

        targetposition = TargetPosition;
        state = Task_State.RUN;
        pid.Reset_PID();

    }

    // METHOD TO CALIBRATE THE BUCKET POSITION.
    // OVERRIDES THE PID BUCKET CONTROL'S OUTPUT TO ZERO TO ALLOW MANUAL ADJUSTMENT OF BUCKET POSITION
    // ONCE CALIBRATION IS DONE, THE BUCKET WILL RETURN TO ITS TARGET POSITION
    public void Calibrate() {
        et.reset();
        state = Task_State.CALIBRATE;
    }

    // METHOD TO OVERRIDE THE BUCKET MOTOR COMMAND TO ZERO FOR ONE SEC
    // USEFUL FOR ALLOWING THE BUCKET TO HANG LOOSELY WHILE IT'S TUCKED INTO THE BASE
    public void Override() {
        et.reset();
        state = Task_State.OVERRIDE;
    }

    // THIS IS THE TASK THAT A STATE MACHINE OPMODE SHOULD CALL REPEATEDLY IN ITS LOOP
    public void BucketTask () {

        double bucketpower_range;

        // Always run this PID control when in RUN, DONE or READY mode
        if (state == Task_State.RUN || state == Task_State.DONE || state == Task_State.READY) {

            // 0.07, 0.000001, 0.000005 (these are the best gains for accurate position and few jitters
            bucketpower = pid.PID_Control(targetposition, 0.07, 0.000001, 0.000005, bucketmotor.getCurrentPosition() );

            // Don't let the motor run too fast. Otherwise, it will overshoot
            bucketpower_range = Range.clip(bucketpower, -0.4, 0.4);
            bucketmotor.setPower(bucketpower_range);

            // If the bucket is within range of the target position, treat the task as done so that the opmode can move on to
            // the next task in its list
            if (state == Task_State.DONE) {
                state = Task_State.READY;
            }
            else if (state == Task_State.RUN && bucketmotor.getCurrentPosition() > (targetposition - tolerance) &&
                    bucketmotor.getCurrentPosition() < (targetposition + tolerance)) {
                state = Task_State.DONE;
            }
        }
        else if (state == Task_State.CALIBRATE) {
            bucketmotor.setPower(0);

            if (et.milliseconds() >= 2000) {

                // Reset the bucket's DC motor encoder
                bucketmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                // Reset the PID object (otherwise, the PID will still have leftover
                // memory of what it previously did which may cause bad commands from carrying forward)
                pid.Reset_PID();
                state = Task_State.DONE;
                bucketmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
        }
        else if (state == Task_State.OVERRIDE) {
            bucketmotor.setPower(0);

            if (et.milliseconds() >= 1000) {

                // Reset the PID object (otherwise, the PID will still have leftover
                // memory of what it previously did which may cause bad commands from carrying forward)
                pid.Reset_PID();
                state = Task_State.DONE;
            }
        }
    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {

        return state;
    }
}
