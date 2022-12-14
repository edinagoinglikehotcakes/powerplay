package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Disabled
@Autonomous
public class AutoC extends LinearOpMode {
    /*
     * Specify the source for the Tensor Flow Model.
     * If the TensorFlowLite object model is included in the Robot Controller App as an "asset",
     * the OpMode must to load it using loadModelFromAsset().  However, if a team generated model
     * has been downloaded to the Robot Controller's SD FLASH memory, it must to be loaded using loadModelFromFile()
     * Here we assume it's an Asset.    Also see method initTfod() below .
     */
    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AS9i1yr/////AAABmerPjSF7IkWrjMfP1Cy6XXBzgkLf7qPuIQp8tUxic8eoHQ6nxd4KdVzVy4bujr9gR3FydTIuiJIAcQqMfxxLDzWrTV1eCLSRm0xOuc1oj7Mm04dEq1dfuG8t/lmaP8KXcPQTW6hpTDb0sCE1XsdQC06H4zazjKRaC0O4C82cDWR+h/N2uEuKCsQvRG2tLoGT/npdBdqV3LaMBQDBI9qSvPLoBPzrXVe+e5ckJ6WR2EUHX1d+n7BirOnL5W2a5RXG52WaBG2hxtDl6m1obtWgNJGHKuoI/BpflkVTFZmjdF5qrwlIZIaf9qAjAq1ddY8MFAqSA2buky3VPDcDIHlv/BfwEhq/zcvyNpvg2NEDv2eZ";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(1.0, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        boolean ran = false;

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        if (!ran && updatedRecognitions.size() > 0) {
                            run(AutoA.Direction.Forward, 5000);
                            ran = true;
                        }

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        }
                        telemetry.update();
                    }
                }
            }
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
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

            telemetry.addData("Motor back left :)(:", motorBackLeft.getCurrentPosition());
            telemetry.addData("Motor back right :)(:", motorBackRight.getCurrentPosition());
            telemetry.addData("Motor front left :)(:", motorFrontLeft.getCurrentPosition());
            telemetry.addData("Motor front right :)(:", motorFrontRight.getCurrentPosition());

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
