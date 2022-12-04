package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class TeleOpC extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware Configuration
        // - Webcam 1
        // - Control Hub Portal
        //   - Control Hub
        //     - Motors
        //       - Port 0: motorFrontLeft (Tetrix Motor)
        //       - Port 1: motorFrontRight (Tetrix Motor)
        //       - Port 2: motorBackLeft (Tetrix Motor)
        //       - Port 3: motorBackRight (Tetrix Motor)
        //   - Expansion Hub 3
        //     - Motors
        //       - Port 0: motorLift (Tetrix Motor)
        //     - Servos
        //       - Port 0: servoClaw (Servo)
        //     - Digital Devices
        //       - Port 1: limitSwitch (REV Touch Sensor)

        final double WHEEL_TURTLE_FACTOR = 3;
        final double LIFT_TURTLE_FACTOR = 12;

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorLift = hardwareMap.dcMotor.get("motorLift");
        Servo servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);

        boolean hasPressed = false;
        boolean clawOpen = false;
        boolean bDown = false;

        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing.
            double rx = gamepad1.right_stick_x;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Check if lift is all the way down.
            boolean isLiftDown = limitSwitch.isPressed();

            // Get the lift position.
            int motorLiftPosition = motorLift.getCurrentPosition();

            // Check if lift is all the way up.
            boolean isLiftUp = motorLiftPosition > 1800;

            if (isLiftDown) {
                motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                hasPressed = true;
            }
            double liftPower = 0;

            // If the user is pressing y and the lift is not all the way up...
            if (gamepad1.y && !isLiftUp) {
                if (hasPressed) {
                    liftPower = 0.8;
                }
                else {
                    telemetry.addData("Warning", "Must lower lift before raising");
                }
            }

            // If the user is pressing a and the lift is not all the way down...
            else if (gamepad1.a && !isLiftDown) {
                if (motorLiftPosition < 50){
                    liftPower = -0.1;
                }
                else {
                    liftPower = -0.8;
                }
            }

            // If the user isn't pressing y or a...
            else {
                liftPower = 0;
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            if (gamepad1.right_trigger > 0.2) {
                liftPower /= LIFT_TURTLE_FACTOR;
                frontLeftPower /= WHEEL_TURTLE_FACTOR;
                backLeftPower /= WHEEL_TURTLE_FACTOR;
                frontRightPower /= WHEEL_TURTLE_FACTOR;
                backRightPower /= WHEEL_TURTLE_FACTOR;
            }
            if (gamepad1.b) {
                if (!bDown) {
                    if (clawOpen) {
                        servoClaw.setPosition(0.9);
                    } else {
                        servoClaw.setPosition(0.6);
                    }
                    clawOpen = !clawOpen;
                    bDown = true;
                }
            }
            else {
                bDown = false;
            }
            motorLift.setPower(liftPower);
            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("Actual Lift Power", motorLift.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Lift", motorLiftPosition);
            telemetry.addData("Pressed", isLiftDown);
            telemetry.addData("Servo", servoClaw.getPosition());
            telemetry.addData("Heading", botHeading);
            telemetry.update();
        }
    }
}
