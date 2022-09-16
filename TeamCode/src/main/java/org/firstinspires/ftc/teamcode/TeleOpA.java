package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TeleOpA extends LinearOpMode {

    private BNO055IMU imu;
    private Orientation angles;
    private DcMotor motorTest;
    private DigitalChannel digitalTouch;
    private ColorRangeSensor colorRangeSensor;
    private DistanceSensor distanceSensor;
    private Servo servoTest;

    @Override
    public void runOpMode() {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);

        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        colorRangeSensor = hardwareMap.get(ColorRangeSensor.class, "colorRangeSensor");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        servoTest = hardwareMap.get(Servo.class, "servoTest");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        /*
        telemetry.addData("Start position", motorTest.getCurrentPosition());
        telemetry.update();
        motorTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTest.setTargetPosition(100);
        motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest.setPower(0.5);
        while (opModeIsActive() && motorTest.isBusy()) {
            telemetry.addData("Current position", motorTest.getCurrentPosition());
            telemetry.update();
        }
        motorTest.setPower(0);
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
        */
        /*
        motorTest.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTest.setTargetPosition(100);
        motorTest.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTest.setPower(0.5);
        while(motorTest.isBusy()) {
            telemetry.addData("Motor position", motorTest.getCurrentPosition());
            telemetry.update();
        }
        motorTest.setPower(0);
        motorTest.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
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

            telemetry.addData("Status", "Running");

            telemetry.update();

        }

    }

}