package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@TeleOp
public class TeleOpA extends LinearOpMode {

    /*
    Robot Configuration

    Motors
    - Port 0 - Tetrix Motor - motorTest
    Servos
    - Port 0 - Servo - servoTest
    Digital Devices
    - Port 1 - REV Touch Sensor - touchSensor
    I2C Bus 0
    - Port 0 - REV internal IMU (BNO055) - imu
    I2C Bus 2
    - Port 0 - REV 2M Distance Sensor - distanceSensor
    I2C Bus 3
    - Port 0 - REV Color/Range Sensor - colorRangeSensor
    Webcam 1
    */

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

    private BNO055IMU imu;
    private Orientation angles;
    private DcMotor motorTest;
    private TouchSensor touchSensor;
    private ColorRangeSensor colorRangeSensor;
    private DistanceSensor distanceSensor;
    private Servo servoTest;

    @Override
    public void runOpMode() {

        //testHardware();
        //testMotorEncoder();
        //testTensorFlow();
        //testRevCoreHexMotor();
        testLimitSwitch();

    }

    private void testHardware() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        touchSensor = hardwareMap.get(TouchSensor.class, "touchSensor");
        colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorRangeSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double targetPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(targetPower);
            telemetry.addData("Target Power", targetPower);
            telemetry.addData("Motor Power", motorTest.getPower());

            if(gamepad1.y) {
                // move to 0 degrees
                servoTest.setPosition(0);
            }
            else if (gamepad1.x || gamepad1.b) {
                // move to 90 degrees
                servoTest.setPosition(0.5);
            }
            else if (gamepad1.a) {
                // move to 180 degrees
                servoTest.setPosition(1);
            }
            telemetry.addData("Servo Position", servoTest.getPosition());

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading: ", angles.firstAngle);
            telemetry.addData("Roll: ", angles.secondAngle);
            telemetry.addData("Pitch: ", angles.thirdAngle);

            telemetry.addData("Red", colorRangeSensor.red());
            telemetry.addData("Green", colorRangeSensor.green());
            telemetry.addData("Blue", colorRangeSensor.blue());
            telemetry.addData("Alpha", colorRangeSensor.alpha());
            telemetry.addData("Range (cm)", colorRangeSensor.getDistance(DistanceUnit.CM));

            telemetry.addData("Distance (cm)", distanceSensor.getDistance(DistanceUnit.CM));

            telemetry.addData("Pressed", touchSensor.isPressed());

            telemetry.addData("Status", "Running");

            telemetry.update();

        }

    }

    private void testMotorEncoder() {

        motorTest = hardwareMap.get(DcMotor.class, "motorTest");

        waitForStart();

        motorTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTest.setTargetPosition(10000);
        motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest.setPower(0.5);
        while(motorTest.isBusy()) {
            telemetry.addData("Motor position", motorTest.getCurrentPosition());
            telemetry.update();
        }
        motorTest.setPower(0);
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void testTensorFlow() {

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
            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                            double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                            double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                            double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                            telemetry.addData(""," ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                            telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
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

    private void testRevCoreHexMotor() {

        // Configuring the robot:
        //
        // Run the driver station app
        // From the menu, choose configure robot
        // Tap edit
        // Tap control hub portal
        // Tap control hub
        // Tap motors
        // For the appropriate port, select REV Robotics Core Hex Motor
        // In the name box, enter hexMotor
        // Tap the checkmark
        // Tap done three times
        // Tap save
        // Tap ok
        // Tap back

        DcMotor motor = hardwareMap.get(DcMotor.class, "hexMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        /*
        while (opModeIsActive()) {
            double targetPower = -this.gamepad1.left_stick_y;
            motor.setPower(targetPower);
            double actualPower = motor.getPower();
            telemetry.addData("Status", "Running");
            telemetry.addData("Power", actualPower);
            telemetry.update();
        }
        */

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(1000);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(0.5);
        while(motor.isBusy()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Position", motor.getCurrentPosition());
            telemetry.update();
        }
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void testLimitSwitch() {

        // REV - Magnetic Limit Switch
        // https://www.revrobotics.com/rev-31-1462/

        // REV - Magnetic Limit Switch - Application Examples
        // https://docs.revrobotics.com/magnetic-limit-switch/application-examples

        // Configuring the robot:
        //
        // Run the driver station app
        // From the menu, choose configure robot
        // Tap edit
        // Tap control hub portal
        // Tap control hub
        // Tap digital devices
        // For the appropriate port, select REV Touch Sensor
        // In the name box, enter limitSwitch
        // Tap the checkmark
        // Tap done three times
        // Tap save
        // Tap ok
        // Tap back

        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean isPressed = limitSwitch.isPressed();
            telemetry.addData("Status", "Running");
            telemetry.addData("Pressed", isPressed);
            telemetry.update();
        }

    }

}