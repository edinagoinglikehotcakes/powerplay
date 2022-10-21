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

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor motorLift = hardwareMap.dcMotor.get("motorLift");
        Servo servoClaw = hardwareMap.get(Servo.class, "servoClaw");
        TouchSensor limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        boolean hasPressed = false;

        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            boolean isPressed = limitSwitch.isPressed();
            if (isPressed) {
                hasPressed = true;
                motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            int motorLiftPosition = motorLift.getCurrentPosition();

            if (hasPressed) {
                // If lift is being raised
                if (gamepad1.a && motorLiftPosition < 1000) {
                    motorLift.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorLift.setPower(0.5);
                }
                // If lift is being lowered
                else if (gamepad1.b && motorLiftPosition > 0) {
                    motorLift.setDirection(DcMotorSimple.Direction.REVERSE);
                    motorLift.setPower(0.5);
                }
                // If lift is stopped
                else {
                    motorLift.setPower(0);
                }
            }

            else {
                telemetry.addData("Limit Switch", "Hasn't been pressed");
            }

            if (gamepad1.y) {
                servoClaw.setPosition(1);
            }
            if (gamepad1.x) {
                servoClaw.setPosition(0);
            }

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            double servoClawPosition = servoClaw.getPosition();

            telemetry.addData("Status", "Running");
            telemetry.addData("Lift", motorLiftPosition);
            telemetry.addData("Pressed", isPressed);
            telemetry.addData("Servo", servoClawPosition);
            telemetry.update();
        }
    }
}
