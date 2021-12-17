package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

public class Bucket_Control {

    DcMotor motor_obj;                  // the motor connected to the bucket
    PID pid_obj;
    double target_position;             // Target position for the bucket (in encoder ticks: +ve or -ve)
    double cmd;                         // Power command for the DC motor
    double cmd_ov;                      // Power override command for the DC motor; Useful for killing power to the motor or request continuous rotation
    final double tolerance = 15;        // How close should we be within the target bucket position before saying we're done
    Task_State run_state;               // This is used by the opmode to determine when this task has completed and proceed to the next task

    // CONSTRUCTOR
    public Bucket_Control(DcMotor motor) {

        // Assign the motor connected to the bucket and initialize it
        motor_obj = motor;
        motor_obj.setDirection(DcMotorSimple.Direction.FORWARD);
        motor_obj.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor_obj.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Create a new PID object to control the bucket
        pid_obj = new PID();
        run_state = Task_State.INIT;
    }

    // METHOD THAT A STATE MACHINE OPMODE SHOULD CALL WHEN IT IS READY TO LAUNCH THE NEXT TASK IN ITS LIST
    public void SetTargetPosition(double target) {

        target_position = target;
        run_state = Task_State.RUN;
    }

    // METHOD TO OVERRIDE THE PID BUCKET CONTROL'S OUTPUT. USEFUL ESPECIALLY FOR KILLING POWER TO BUCKET MOTOR
    // TO GET OUT OF OVERRIDE MODE, JUST CALL SetTargetPosition() TO START RUN MODE
    public void OvControl(double command_ov) {
        cmd_ov = command_ov;
        run_state = Task_State.OVERRIDE;
    }

    // THIS IS THE TASK THAT A STATE MACHINE OPMODE SHOULD CALL REPEATEDLY IN ITS LOOP
    public void Task () {

        double clipped_cmd;

        // Always run the PID control when in RUN or DONE mode
        if (run_state != Task_State.INIT && run_state != Task_State.OVERRIDE) {

            // 0.07, 0.000001, 0.000005 (these are the best gains for accurate position and few jitters
            cmd = pid_obj.PID_Control(target_position, 0.07, 0.000001, 0.000005, motor_obj.getCurrentPosition() );

            // Don't let the motor run too fast. Otherwise, it will overshoot
            clipped_cmd = Range.clip(cmd, -0.3, 0.3);
            motor_obj.setPower(clipped_cmd);

            // If the bucket is within range of the target position, treat the task as done so that the opmode can move on to
            // the next task in its list
            if (motor_obj.getCurrentPosition() > (target_position - tolerance) &&
                    motor_obj.getCurrentPosition() < (target_position + tolerance)) {
                run_state = Task_State.DONE;
            }
        }
        else if (run_state == Task_State.OVERRIDE) {
            motor_obj.setPower(cmd_ov);
        }
    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {

        return run_state;
    }
}
