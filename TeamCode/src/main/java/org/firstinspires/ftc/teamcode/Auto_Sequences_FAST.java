package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auto_Sequences_FAST {

    static DcMotor Rail;
    static Servo IntakeServo;
    static Servo GateServo;
    static Servo ElementServo;

    ElapsedTime ET = new ElapsedTime();

    Bucket_Control BucketCtrl;
    Arm_Control ArmCtrl;

    int HIGH = 1;
    int MIDDLE = 2;
    int LOW = 3;
    int RETURN = 4;

    static final int Top_Arm_Left = -420;
    static final int Top_Arm_Right = 420;
    static final int Middle_Arm_Left = -315;
    static final int Middle_Arm_Right = 315;
    static final int Low_Arm_Left = -160;
    static final int Low_Arm_Right = 160;

    static final double OriginalBucketPosition = 0;
    static final double TopBucketPosition = 140;
    static final double MirrorTopBucketPosition = -140;
    static final double MiddleBucketPosition = 120;
    static final double MirrorMiddleBucketPosition = -120;
    static final double LowBucketPosition = 95;
    static final double MirrorLowBucketPosition = -95;

    static final double OpenGatePosition = 0.8;
    static final double OpenIntakePosition = 0.6;
    static final double ClosingGatePosition = 0.2;
    static final double ClosingIntakePosition = 0.8;

    boolean mirror_event = false;
    boolean topalliancehub = false;
    boolean middlealliancehub = false;
    boolean lowalliancehub = false;
    boolean returntobase = false;

    int DeliverySequenceHIGH = 0;
    int DeliverySequenceMIDDLE = 0;
    int DeliverySequenceLOW = 0;
    int ReturnSequence = 0;

    Telemetry telemetry;
    Task_State state;

    public Auto_Sequences_FAST(Bucket_Control bucket_ctrl, Arm_Control arm_ctrl, DcMotor rail, Telemetry Telemetry) {

        // Assign the motor connected to the bucket and initialize it
        BucketCtrl = bucket_ctrl;
        ArmCtrl = arm_ctrl;
        Rail = rail;
        telemetry = Telemetry;
        state = Task_State.INIT;

    }

    public void SetSequence(int sequence, boolean mirrorevent) {

        if (mirrorevent) {
            mirror_event = true;
        }
        else {
            mirror_event = false;
        }

        if (sequence == HIGH) {
            topalliancehub = true;
            DeliverySequenceHIGH = 1;
        }
        else if (sequence == MIDDLE) {
            middlealliancehub = true;
            DeliverySequenceMIDDLE = 1;
        }
        else if (sequence == LOW) {
            lowalliancehub = true;
            DeliverySequenceLOW = 1;
        }
        else if (sequence == RETURN) {
            returntobase = true;
            ReturnSequence = 1;
        }

        state = Task_State.RUN;
    }

    public void Task () {

        if (state == Task_State.RUN) {

            if (returntobase) {
                switch (ReturnSequence) {
                    case 1:
                        ReturnSequence++;
                        break;

                    case 2:
                        if (BucketCtrl.GetTaskState() == Task_State.INIT || BucketCtrl.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                BucketCtrl.SetTargetPosition(20);
                            }
                            else {
                                BucketCtrl.SetTargetPosition(-20);
                            }
                        }
                        else if (BucketCtrl.GetTaskState() == Task_State.DONE) {
                            ReturnSequence++;
                        }
                        break;

                    case 3:
                        if (topalliancehub == true || middlealliancehub == true) {
                            if (ArmCtrl.GetTaskState() == Task_State.INIT || ArmCtrl.GetTaskState() == Task_State.READY) {
                                if (mirror_event) {
                                    ArmCtrl.SetTargetPosition(80, 0.00002, 0.00002);
                                }
                                else {
                                    ArmCtrl.SetTargetPosition(-80, -0.00002, -0.00002);
                                }

                            }
                            else if (ArmCtrl.GetTaskState() == Task_State.DONE) {
                                ReturnSequence++;
                            }
                        }
                        else if (lowalliancehub == true) {

                            if (ArmCtrl.GetTaskState() == Task_State.INIT || ArmCtrl.GetTaskState() == Task_State.READY) {
                                if (mirror_event) {
                                    ArmCtrl.SetTargetPosition(30, 0.00001, 0.00001);
                                }
                                else {
                                    ArmCtrl.SetTargetPosition(-30, -0.00001, -0.00001);
                                }

                            }
                            else if (ArmCtrl.GetTaskState() == Task_State.DONE) {
                                ReturnSequence++;
                            }
                        }
                        break;

                    case 4:
                        if (ArmCtrl.GetTaskState() == Task_State.INIT || ArmCtrl.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                ArmCtrl.SetTargetPosition(-10, -0.1, 0.1);
                            }
                            else {
                                ArmCtrl.SetTargetPosition(10, -0.1, 0.1);
                            }
                        }
                        else if (ArmCtrl.GetTaskState() == Task_State.DONE) {
                            ReturnSequence++;
                        }
                        break;

                    case 5:
                        if (ArmCtrl.GetTaskState() == Task_State.READY) {
                            //ArmCtrl.SetTargetPosition(0, -0.1, 0.1);
                            //ArmCtrl.Override();
                            //BucketCtrl.Override();
                            Rail.setTargetPosition(750);
                            Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Rail.setPower(0.5);
                        }
                        else if (ArmCtrl.GetTaskState() == Task_State.DONE) {
                            lowalliancehub = false;
                            middlealliancehub = false;
                            topalliancehub = false;
                            returntobase = false;
                            state = Task_State.DONE;
                            ReturnSequence++;
                        }
                        break;

                    default:
                        break;
                }
            }

            else if (topalliancehub) {
                switch (DeliverySequenceHIGH) {
                    case 1:
                        BucketCtrl.SetTargetPosition(1);
                        Rail.setTargetPosition(900);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.65);
                        if (Rail.getCurrentPosition() >= 870 && Rail.getCurrentPosition() <= 930) {
                            DeliverySequenceHIGH++;
                        }
                        break;

                    case 2:
                        if (ArmCtrl.GetTaskState() == Task_State.INIT || ArmCtrl.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                ArmCtrl.SetTargetPosition(Top_Arm_Right, -0.6, 0.6);
                            } else {
                                ArmCtrl.SetTargetPosition(Top_Arm_Left, -0.6, 0.6);
                            }
                        }
                        else if (ArmCtrl.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceHIGH++;
                        }
                        break;

                    case 3:
                        if (BucketCtrl.GetTaskState() == Task_State.INIT || BucketCtrl.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                BucketCtrl.SetTargetPosition(MirrorTopBucketPosition);
                            } else {
                                BucketCtrl.SetTargetPosition(TopBucketPosition);
                            }
                        }
                        else if (BucketCtrl.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceHIGH++;
                            state = Task_State.DONE;
                        }
                        break;

                    default:
                        break;
                }
            }

            else if (middlealliancehub) {
                switch (DeliverySequenceMIDDLE) {
                    case 1:
                        BucketCtrl.SetTargetPosition(1);
                        Rail.setTargetPosition(820);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.65);
                        if (Rail.getCurrentPosition() >= 790 && Rail.getCurrentPosition() <= 850) {
                            DeliverySequenceMIDDLE++;
                        }
                        break;

                    case 2:
                        if (ArmCtrl.GetTaskState() == Task_State.INIT || ArmCtrl.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                ArmCtrl.SetTargetPosition(Middle_Arm_Right, -0.6, 0.6);
                            } else {
                                ArmCtrl.SetTargetPosition(Middle_Arm_Left, -0.6, 0.6);
                            }
                        }
                        else if (ArmCtrl.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceMIDDLE++;
                        }
                        break;

                    case 3:
                        if (BucketCtrl.GetTaskState() == Task_State.INIT || BucketCtrl.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                BucketCtrl.SetTargetPosition(MirrorMiddleBucketPosition);
                            } else {
                                BucketCtrl.SetTargetPosition(MiddleBucketPosition);
                            }
                        }
                        else if (BucketCtrl.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceMIDDLE++;
                            state = Task_State.DONE;
                        }
                        break;

                    default:
                        break;
                }
            }

            else if (lowalliancehub) {
                switch (DeliverySequenceLOW) {
                    case 1:
                        BucketCtrl.SetTargetPosition(1);
                        Rail.setTargetPosition(820);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.65);
                        if (Rail.getCurrentPosition() >= 790 && Rail.getCurrentPosition() <= 850) {
                            DeliverySequenceLOW++;
                        }
                        break;

                    case 2:
                        if (ArmCtrl.GetTaskState() == Task_State.INIT || ArmCtrl.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                ArmCtrl.SetTargetPosition(Low_Arm_Right, -0.6, 0.6);
                            } else {
                                ArmCtrl.SetTargetPosition(Low_Arm_Left, -0.6, 0.6);
                            }
                        }
                        else if (ArmCtrl.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceLOW++;
                            ET.reset();
                        }
                        break;

                    case 3:
                        if (ET.milliseconds() > 100) {
                            DeliverySequenceLOW++;
                        }
                        break;

                    case 4:
                        if (BucketCtrl.GetTaskState() == Task_State.INIT || BucketCtrl.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                BucketCtrl.SetTargetPosition(MirrorLowBucketPosition);
                            } else {
                                BucketCtrl.SetTargetPosition(LowBucketPosition);
                            }
                        }
                        else if (BucketCtrl.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceLOW++;
                            state = Task_State.DONE;
                        }
                        break;

                    default:
                        break;
                }
            }
        }
        else if (state == Task_State.DONE){
            state = Task_State.READY;
        }

        telemetry.addData("HighSequence", DeliverySequenceHIGH);
        telemetry.addData("MiddleSequence", DeliverySequenceMIDDLE);
        telemetry.addData("LowSequence", DeliverySequenceLOW);
        telemetry.addData("ReturnSequence", ReturnSequence);
        telemetry.update();
    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {
        return state;
    }

}
