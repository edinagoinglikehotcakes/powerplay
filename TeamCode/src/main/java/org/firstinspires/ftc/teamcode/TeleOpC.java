package org.firstinspires.ftc.teamcode;

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
        //     - Servos
        //       - Port 0: servoClaw (Servo)
        //   - Expansion Hub 3
        //     - Motors
        //       - Port 0: motorLift (Tetrix Motor)
        //     - Digital Devices
        //       - Port 1: limitSwitch (REV Touch Sensor)

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorLift = hardwareMap.dcMotor.get("motorLift");
        Servo servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        boolean hasPressed = false;
        boolean clawOpen = false;
        boolean bDown = false;

        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            if (hasPressed) {

                // If the user is pressing y and the lift is not all the way up...
                if (gamepad1.y && !isLiftUp) {
                    if (motorLiftPosition < 50){
                        motorLift.setPower(0.2);
                    }
                    else {
                        motorLift.setPower(0.1);
                    }
                }

                // If the user is pressing a and the lift is not all the way down...
                else if (gamepad1.a && !isLiftDown) {
                    if (motorLiftPosition < 50){
                        motorLift.setPower(-0.025);
                    }
                    else {
                        motorLift.setPower(-0.1);
                    }
                }

                // If the user isn't pressing y or a...
                else {
                    motorLift.setPower(0);
                }

            }
            else {
                telemetry.addData("Limit Switch", "Hasn't been pressed");
            }

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (gamepad1.b) {
                if (!bDown) {
                    if (clawOpen) {
                        servoClaw.setPosition(0.75);
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

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("Actual Lift Power", motorLift.getPower());
            telemetry.addData("Status", "Running");
            telemetry.addData("Lift", motorLiftPosition);
            telemetry.addData("Pressed", isLiftDown);
            telemetry.addData("Servo", servoClaw.getPosition());
            telemetry.update();
        }
    }
}
