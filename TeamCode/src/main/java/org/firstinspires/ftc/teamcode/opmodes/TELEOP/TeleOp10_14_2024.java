package org.firstinspires.ftc.teamcode.opmodes.TELEOP;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp

public class TeleOp10_14_2024 extends LinearOpMode {
    private DcMotor Lift;
    private DcMotor RF;
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor LB;
    private CRServo Intake;
    private Servo Extension;
    private Servo Rotation;
    private Servo Claw;
    private double dp = 0.5;
    private double intPow = 0.5;
    private double liftPow = 0.25;
    private double LIFT_POWER = 0.45;
    private int LIFT_INCREMENT = 30;
    //private int MAX_LIFT_POS = 4500;
    public void runOpMode() throws InterruptedException {
        RF = hardwareMap.get(DcMotor.class, "RF");
        LF = hardwareMap.get(DcMotor.class, "LF");
        RB = hardwareMap.get(DcMotor.class, "RB");
        LB = hardwareMap.get(DcMotor.class, "LB");
        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        LB.setDirection(DcMotorSimple.Direction.FORWARD);
        RB.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift = hardwareMap.get(DcMotor.class, "lift");
        Intake = hardwareMap.get(CRServo.class, "intake");
        Extension = hardwareMap.get(Servo.class, "extend");
        Rotation = hardwareMap.get(Servo.class, "rotate");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.FORWARD);
        Claw = hardwareMap.get(Servo.class, "claw"); //Specimen Claw
        /*double ServoPosition;
        double ServoSpeed;
        int currentPos;
        ServoPosition = 1;
        ServoSpeed = 1;*/
        waitForStart();
        while (opModeIsActive()) {
            //Forward/Backward
            if (gamepad1.left_stick_y != 0) {
                LF.setPower(dp * gamepad1.left_stick_y);
                RF.setPower(dp * gamepad1.left_stick_y);
                LB.setPower(dp * gamepad1.left_stick_y);
                RB.setPower(dp * gamepad1.left_stick_y);
            }
            //Logic to turn left/right
            if (gamepad1.right_stick_x != 0) {
                LF.setPower(-dp * gamepad1.right_stick_x);
                RF.setPower(dp * gamepad1.right_stick_x);
                LB.setPower(-dp * gamepad1.right_stick_x);
                RB.setPower(dp * gamepad1.right_stick_x);
            }
            //Logic to STRAFE left/right
            if (gamepad1.left_stick_x!=0) {
              LF.setPower(-dp * gamepad1.left_stick_x);
              RF.setPower(dp * gamepad1.left_stick_x);
              LB.setPower(dp * gamepad1.left_stick_x);
              RB.setPower(-dp * gamepad1.left_stick_x);
            }
            //StartIntake
            if (!gamepad1.start && gamepad1.right_bumper) {
                Intake.setPower(intPow);
            }
            else if (!gamepad1.start && gamepad1.left_bumper) {
                Intake.setPower(-intPow);
            }
            else {
                Intake.setPower(0);
            }
            if (gamepad1.dpad_up) {
                Extension.setPosition(1);
            }
            else if (gamepad1.dpad_down) {
                Extension.setPosition(0);
            }
            if (gamepad1.dpad_right) {
                Rotation.setPosition(0);
            }
            else if (gamepad1.dpad_left) {
                Rotation.setPosition(0.2-0.005);
            }
            else if (gamepad1.back) {
                Rotation.setPosition(0.05);
            }
            if (gamepad1.right_trigger>0) {
                Lift.setPower(liftPow);
            }
            if (gamepad1.left_trigger>0) {
                Lift.setPower(-liftPow);
            }
            if (gamepad1.a) {
                Lift.setTargetPosition(0);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                while (Lift.isBusy()) {
                    telemetry.addData("Current Position", Lift.getCurrentPosition());
                    telemetry.addData("Target Position", Lift.getTargetPosition());
                    telemetry.update();
                }
            }
            if (gamepad1.y) {
                Lift.setTargetPosition(1000);//To be updated, Belt is loose
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                while (Lift.isBusy()) {
                    telemetry.addData("Current Position", Lift.getCurrentPosition());
                    telemetry.addData("Target Position", Lift.getTargetPosition());
                    telemetry.update();
                }
            }
            if (gamepad1.start && gamepad1.left_bumper) { //Open
                Claw.setPosition(0.5);
            } else if (gamepad1.start && gamepad1.right_bumper) { //Close
                Claw.setPosition(0.75);
            }
            telemetry.update();
            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);
            //Intake.setPower(0.5);
        }
    }
    
}
