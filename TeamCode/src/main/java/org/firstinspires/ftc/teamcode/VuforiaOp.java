package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

public class VuforiaOp extends LinearOpMode {

    private VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        VuforiaLocalizer.Parameters VuforiaParameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        VuforiaParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaParameters.vuforiaLicenseKey = "AZa083X/////AAABmVxKKF/QkUlHg/0J5chdkVRUvazSQENKXAnzKcUo/WXk8nuuMiuaMq/eW71vQQN0WFoW5w0ov0p2jxr92dYn2YsJ5JnG/IT6X//GV2CTADTOrPRtyAKWctjhjgg/dcQdgKdSZy6G9yArywDxieG4+OgnOIZGveaQQJks9ldwi02hD9E953LK4jQADFyiRiENUq/7j9OCGLnyFW+WJ0BmFQP8eNmPFlHbhiEZnJVrKBWckUDkCo1rJ1qZVwUazw10TmUQr7Yju/sKnL4UZG3S/SS1VOU0WuR/2Ok1IF74B9gTok1kjIfLTms0XumnAE4BcUkXMZRWCtK2Pp3SYb+85wmohgS4POAqCIALtfvePXOq";
        VuforiaParameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(VuforiaParameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS,4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()) {

            for(VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();

                if (pose != null) {
                    VectorF translation = pose.getTranslation();

                    telemetry.addData(beac.getName() + "-Translation", translation);

                    //make translation get 0,2 if phone is mounted horizontal on robots - not using phones, so idk
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(1), translation.get(2)));

                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();
        }

    }
}
