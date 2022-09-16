package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class TeleOpB extends LinearOpMode {
    private DcMotor motorTest;
    private Servo servoTest;
    private BNO055IMU imu;
    private Orientation angles;
    private DistanceSensor distanceSensor;
    @Override
    public void runOpMode() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        servoTest = hardwareMap.get(Servo.class, "servoTest");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        //
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        //
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        double targetPower = 0;
        while (opModeIsActive()) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Heading",angles.firstAngle);
            telemetry.addData("Roll",angles.secondAngle);
            telemetry.addData("Pitch",angles.thirdAngle);
            telemetry.addData("Distance (cm)",distanceSensor.getDistance(DistanceUnit.CM));
            targetPower = -this.gamepad1.left_stick_y;
            motorTest.setPower(targetPower);
            if(gamepad1.y){
                servoTest.setPosition(0);
            }else if (gamepad1.x||gamepad1.b){
                servoTest.setPosition(0.5);
            }else if (gamepad1.a){
                servoTest.setPosition(1);
            }
            telemetry.addData("Servo Position", servoTest.getPosition());
            telemetry.addData("Target Power", targetPower);
            telemetry.addData("Motor Power", motorTest.getPower());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }
    }
}
