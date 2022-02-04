package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Auto_Sequences {

    static DcMotor Rail;
    static NormalizedColorSensor colorsensor;
    static RevBlinkinLedDriver ColorStrip;
    BNO055IMU IMU;
    static DcMotor Bucket;
    static Servo IntakeServo;
    static Servo GateServo;
    static Servo ElementServo;

    ElapsedTime ET = new ElapsedTime();
    Bucket_Control BucketMotor;
    Arm_Control ArmMotor;

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

    int DeliverySequenceHIGH = 0;
    int DeliverySequenceMIDDLE = 0;
    int DeliverySequenceLOW = 0;
    int ReturnSequence = 0;

    Telemetry telemetry;
    Task_State state;

    public void SetSequence(int sequence, boolean mirrorevent) {

        if (sequence == HIGH) {
            if (mirrorevent) {
                mirror_event = true;
                topalliancehub = true;
            } else {
                mirror_event = false;
            }
            DeliverySequenceHIGH = 1;
            state = Task_State.RUN;
        }
        if (sequence == MIDDLE) {
            if (mirrorevent) {
                mirror_event = true;
                middlealliancehub = true;
            } else {
                mirror_event = false;
            }
            DeliverySequenceMIDDLE = 1;
            state = Task_State.RUN;
        }
        if (sequence == LOW) {
            if (mirrorevent) {
                mirror_event = true;
                lowalliancehub = true;
            } else {
                mirror_event = false;
            }
            DeliverySequenceLOW = 1;
            state = Task_State.RUN;
        }
        if (sequence == RETURN) {
            if (mirrorevent) {
                mirror_event = true;
            } else {
                mirror_event = false;
            }
            ReturnSequence = 1;
            state = Task_State.RUN;
        }

    }



    public void Task () {

        if (state == Task_State.RUN) {
            if (HIGH == 1) {
                switch (DeliverySequenceHIGH) {
                    case 1:
                        HIGH++;
                        Rail.setTargetPosition(900);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.65);
                        if (Rail.getCurrentPosition() >= 870 && Rail.getCurrentPosition() <= 930) {
                            DeliverySequenceHIGH++;
                        }
                        break;

                    case 2:
                        if (ArmMotor.GetTaskState() == Task_State.INIT) {
                            if (mirror_event) {
                                ArmMotor.SetTargetPosition(Top_Arm_Right, -0.6, 0.6);
                            } else {
                                ArmMotor.SetTargetPosition(Top_Arm_Left, -0.6, 0.6);
                            }
                        }
                        else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceHIGH++;
                        }
                        break;

                    case 3:
                        if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                BucketMotor.SetTargetPosition(MirrorTopBucketPosition);
                            } else {
                                BucketMotor.SetTargetPosition(TopBucketPosition);
                            }
                        }
                        else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceHIGH++;
                        }
                        break;
                }
            }



            if (MIDDLE == 1) {
                switch (DeliverySequenceMIDDLE) {
                    case 1:
                        MIDDLE++;
                        Rail.setTargetPosition(820);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.65);
                        if (Rail.getCurrentPosition() >= 790 && Rail.getCurrentPosition() <= 850) {
                            DeliverySequenceMIDDLE++;
                        }
                        break;

                    case 2:
                        if (ArmMotor.GetTaskState() == Task_State.INIT) {
                            if (mirror_event) {
                                ArmMotor.SetTargetPosition(Middle_Arm_Right, -0.6, 0.6);
                            } else {
                                ArmMotor.SetTargetPosition(Middle_Arm_Left, -0.6, 0.6);
                            }
                        }
                        else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceMIDDLE++;
                        }
                        break;

                    case 3:
                        if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                BucketMotor.SetTargetPosition(MirrorMiddleBucketPosition);
                            } else {
                                BucketMotor.SetTargetPosition(MiddleBucketPosition);
                            }
                        }
                        else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceMIDDLE++;
                        }
                        break;
                }
            }



            if (LOW == 1) {
                switch (DeliverySequenceLOW) {
                    case 1:
                        LOW++;
                        Rail.setTargetPosition(820);
                        Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        Rail.setPower(0.65);
                        if (Rail.getCurrentPosition() >= 790 && Rail.getCurrentPosition() <= 850) {
                            DeliverySequenceLOW++;
                        }
                        break;

                    case 2:
                        if (ArmMotor.GetTaskState() == Task_State.INIT) {
                            if (mirror_event) {
                                ArmMotor.SetTargetPosition(Low_Arm_Right, -0.6, 0.6);
                            } else {
                                ArmMotor.SetTargetPosition(Low_Arm_Left, -0.6, 0.6);
                            }
                        }
                        else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceLOW++;
                        }
                        break;

                    case 3:
                        if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                BucketMotor.SetTargetPosition(MirrorLowBucketPosition);
                            } else {
                                BucketMotor.SetTargetPosition(LowBucketPosition);
                            }
                        }
                        else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                            DeliverySequenceLOW++;
                        }
                        break;
                }
            }



            if (RETURN == 1) {
                switch (ReturnSequence) {
                    case 1:
                        RETURN++;
                        ReturnSequence++;
                        break;

                    case 2:
                        if (BucketMotor.GetTaskState() == Task_State.INIT || BucketMotor.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                BucketMotor.SetTargetPosition(15);
                            }
                            else {
                                BucketMotor.SetTargetPosition(-15);
                            }
                        }
                        else if (BucketMotor.GetTaskState() == Task_State.DONE) {
                            ReturnSequence++;
                        }
                        break;

                    case 3:
                        if (topalliancehub == true || middlealliancehub == true) {
                            if (ArmMotor.GetTaskState() == Task_State.INIT ||
                                    ArmMotor.GetTaskState() == Task_State.READY) {
                                if (mirror_event) {
                                    ArmMotor.SetTargetPosition(80, 0.00002, 0.00002);
                                }
                                else {
                                    ArmMotor.SetTargetPosition(-80, -0.00002, -0.00002);
                                }

                            }
                            else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                                ReturnSequence++;
                            }
                        }
                        else if (lowalliancehub == true) {

                            if (ArmMotor.GetTaskState() == Task_State.INIT ||
                                    ArmMotor.GetTaskState() == Task_State.READY) {
                                if (mirror_event) {
                                    ArmMotor.SetTargetPosition(30, 0.00001, 0.00001);
                                }
                                else {
                                    ArmMotor.SetTargetPosition(-30, -0.00001, -0.00001);
                                }

                            }
                            else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                                ReturnSequence++;
                            }
                        }
                        break;

                    case 4:
                        if (ArmMotor.GetTaskState() == Task_State.INIT ||
                                ArmMotor.GetTaskState() == Task_State.READY) {
                            if (mirror_event) {
                                ArmMotor.SetTargetPosition(-10, -0.1, 0.1);
                            }
                            else {
                                ArmMotor.SetTargetPosition(10, -0.1, 0.1);
                            }
                        }
                        else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            ReturnSequence++;
                        }
                        break;

                    case 5:
                        if (ArmMotor.GetTaskState() == Task_State.READY) {
                            ArmMotor.Override();
                            Rail.setTargetPosition(700);
                            Rail.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Rail.setPower(0.5);
                        }
                        else if (ArmMotor.GetTaskState() == Task_State.DONE) {
                            lowalliancehub = false;
                            middlealliancehub = false;
                            topalliancehub = false;
                            ReturnSequence++;
                        }
                        break;
                }
            }

            telemetry.addData("HighSequence", DeliverySequenceHIGH);
            telemetry.addData("MiddleSequence", DeliverySequenceMIDDLE);
            telemetry.addData("LowSequence", DeliverySequenceLOW);
            telemetry.addData("ReturnSequence", ReturnSequence);
            telemetry.update();

        }
    }

    // A STATE MACHINE OPMODE SHOULD CALL THIS METHOD TO DETERMINE WHETHER THE TASK IS DONE
    public Task_State GetTaskState() {
        return state;
    }

}
