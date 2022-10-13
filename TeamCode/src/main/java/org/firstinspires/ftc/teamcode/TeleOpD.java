package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class TeleOpD extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("hexMotor");
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y; // Remember, this is reversed!

            motorFrontLeft.setPower(power);
        }
    }
}