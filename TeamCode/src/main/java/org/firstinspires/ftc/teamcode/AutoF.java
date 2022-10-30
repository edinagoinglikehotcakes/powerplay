//.=225 ..=224 ...223
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutoF extends LinearOpMode {
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    static final int DRAGON_ID = 573;
    static final int FIREBALL_ID = 136;
    static final int SKULL_ID = 127;
    static final int FORWARD_TICKS = 1000;
    static final int STRAFE_TICKS = 1000;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                telemetry.addData("Error:", errorCode);
                telemetry.update();
            }
        });

        waitForStart();

        telemetry.setMsTransmissionInterval(50);
        boolean ran = false;
        while (opModeIsActive())
        {
            // Calling getDetectionsUpdate() will only return an object if there was a new frame
            // processed since the last time we called it. Otherwise, it will return null. This
            // enables us to only run logic when there has been a new frame, as opposed to the
            // getLatestDetections() method which will always return an object.
            ArrayList<AprilTagDetection> detections = aprilTagDetectionPipeline.getDetectionsUpdate();

            // If there's been a new frame...
            if(detections != null)
            {
                for(AprilTagDetection detection : detections)
                {
                    telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
                    if (!ran) {
                        ran = true;
                        if (detection.id == DRAGON_ID) {
                            run(AutoA.Direction.Forward, FORWARD_TICKS);
                            run(AutoA.Direction.Left, STRAFE_TICKS);
                        } else if (detection.id == FIREBALL_ID) {
                            run(AutoA.Direction.Forward, FORWARD_TICKS);
                        } else if (detection.id == SKULL_ID) {
                            run(AutoA.Direction.Forward, FORWARD_TICKS);
                            run(AutoA.Direction.Right, STRAFE_TICKS);
                        }
                    }
                }

                telemetry.update();
            }
            sleep(20);

        }
    }
    enum Direction{
        Left,
        Right,
        Forward,
        Back
    }
    void run(AutoA.Direction direction, int targetPosition) throws InterruptedException {

        DcMotor motorBackLeft = hardwareMap.get(DcMotor.class, "motorBackLeft");
        DcMotor motorBackRight = hardwareMap.get(DcMotor.class, "motorBackRight");
        DcMotor motorFrontLeft = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        DcMotor motorFrontRight = hardwareMap.get(DcMotor.class, "motorFrontRight");

        if (direction == AutoA.Direction.Left){
            motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if (direction == AutoA.Direction.Forward){
            motorBackLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        else if (direction == AutoA.Direction.Right){
            motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFrontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else if (direction == AutoA.Direction.Back){
            motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else{
            throw new InterruptedException("Direction Unrecognized");
        }

        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorBackLeft.setTargetPosition(targetPosition);
        motorBackRight.setTargetPosition(targetPosition);
        motorFrontLeft.setTargetPosition(targetPosition);
        motorFrontRight.setTargetPosition(targetPosition);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorBackLeft.setPower(0.5);
        motorBackRight.setPower(0.5);
        motorFrontLeft.setPower(0.5);
        motorFrontRight.setPower(0.5);

        while(motorBackLeft.isBusy() || motorBackRight.isBusy() || motorFrontLeft.isBusy() || motorFrontRight.isBusy()) {

            telemetry.addData("target position ", targetPosition);

            telemetry.addData("Motor back left", motorBackLeft.getCurrentPosition());
            telemetry.addData("Motor back right", motorBackRight.getCurrentPosition());
            telemetry.addData("Motor front left", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Motor front right", motorFrontRight.getCurrentPosition());

            telemetry.update();

        }

        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);

        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}
