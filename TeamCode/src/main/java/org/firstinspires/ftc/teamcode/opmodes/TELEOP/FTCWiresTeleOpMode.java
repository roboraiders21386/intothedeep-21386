package org.firstinspires.ftc.teamcode.opmodes.TELEOP;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;


@TeleOp(name = "Tx-Rx TeleOp", group = "00-Teleop")
public class FTCWiresTeleOpMode extends LinearOpMode {
    private Servo intake;
    private Servo arm;
    private DcMotor lift;
    @Override
    public void runOpMode() throws InterruptedException {
        double SLOW_DOWN_FACTOR = 0.5; //TODO Adjust to driver comfort
        double LOW_BASKET_LIFT = 1000; //TODO Make sure this is accruate when we have game elements
        telemetry.addData("Initializing FTC Wires (ftcwires.org) TeleOp adopted for Team:","21386");
        telemetry.update();

        //initializing
        lift = hardwareMap.get(DcMotor.class, "lift");
        //intake = hardwareMap.get(Servo.class, "INTAKE"); JUST IN CASE
        //arm = hardwareMap.get(Servo.class, "ARM");

        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                telemetry.addData("Running FTC Wires (ftcwires.org) TeleOp Mode adopted for Team:","TEAM NUMBER");
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                                -gamepad1.left_stick_x * SLOW_DOWN_FACTOR
                        ),
                        -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
                ));

                drive.updatePoseEstimate();

                //telemetry.addData("LF Encoder", drive.leftFront.getCurrentPosition());
                //telemetry.addData("LB Encoder", drive.leftBack.getCurrentPosition());
                //telemetry.addData("RF Encoder", drive.rightFront.getCurrentPosition());
                //telemetry.addData("RB Encoder", drive.rightBack.getCurrentPosition());

                telemetry.addLine("Current Pose");
                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", Math.toDegrees(drive.pose.heading.log()));
                telemetry.update();
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            waitForStart();

            while (opModeIsActive()) {
                drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(
                                -gamepad1.left_stick_y * SLOW_DOWN_FACTOR,
                                0.0
                        ),
                        -gamepad1.right_stick_x * SLOW_DOWN_FACTOR
                ));

                drive.updatePoseEstimate();

                telemetry.addData("x", drive.pose.position.x);
                telemetry.addData("y", drive.pose.position.y);
                telemetry.addData("heading", drive.pose.heading);
                telemetry.update();
            }
        } else {
            throw new AssertionError();
        }
        if (gamepad1.circle){
            lift.setTargetPosition(0); //set to start position
            telemetry.addData("Set to", "start value");
            telemetry.update();
        }
        if (gamepad1.triangle){
            lift.setTargetPosition(1000);//set to low position
            telemetry.addData("Set to", "low value");
            telemetry.update();
        }
        if (gamepad1.cross){
            lift.setTargetPosition(700);//set to rigging/specimin position
            telemetry.addData("Set to", "specimin value");
            telemetry.update();
        }
        if (gamepad1.square){
            lift.setTargetPosition(1500);//set to high position
            telemetry.addData("Set to", "high value");
            telemetry.update();
        }
        if (gamepad1.dpad_up){
            lift.setTargetPosition(lift.getCurrentPosition()+50);
            telemetry.addData("Incrementing by", "50");
            telemetry.update();
        }
        if (gamepad1.dpad_down){
            lift.setTargetPosition(lift.getCurrentPosition()-50);
            telemetry.addData("Decrementing by", "50");
            telemetry.update();
        }
    }
}
