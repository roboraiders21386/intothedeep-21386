package org.firstinspires.ftc.teamcode.opmodes.TELEOP;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp
public class TeleOpPS5Meet2 extends LinearOpMode {
    private DcMotor Lift;
    private DcMotor Lift2;
    private DcMotor RF;
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor LB;
    private Servo Intake;
    private Servo Rotation;
    private Servo Claw;
    private Servo Wrist;
    private double dp = 1;
    private double intPow = 0.5;
    private double liftPow = 1;
    private int LIFT_INCREMENT = 10;
    private int a = 100;
    private int b = -100;
    //ticks per rev 537.7
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
        Lift2 = hardwareMap.get(DcMotor.class, "lift2");
        Intake = hardwareMap.get(Servo.class, "sample");
        Rotation = hardwareMap.get(Servo.class, "rotate");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setDirection(DcMotorSimple.Direction.FORWARD);
        Claw = hardwareMap.get(Servo.class, "specimen"); //Specimen Claw
        Wrist = hardwareMap.get(Servo.class, "wrist");
        /*double ServoPosition;
        double ServoSpeed;
        int currentPos;
        ServoPosition = 1;
        ServoSpeed = 1;*/
        int clawUse = 0;
        waitForStart();
        Lift.setTargetPosition(0);
        Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift.setPower(liftPow);

        Lift2.setTargetPosition(0);
        Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Lift2.setPower(liftPow);

        Rotation.setPosition(0);
        Claw.setPosition(0.77);
        Intake.setPosition(0);
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
            boolean off = true;
            if (!gamepad1.start && gamepad1.right_bumper) {
                Intake.setPosition(0.3);
                off = false;
            }
            if (!gamepad1.start && gamepad1.left_bumper) {
                Intake.setPosition(0);
            }
            while (gamepad1.left_trigger>0) {
                Wrist.setPosition(Wrist.getPosition()-0.1);
            }
            while (gamepad1.right_trigger>0) {
                Wrist.setPosition(Wrist.getPosition()+0.1);
            }
            if (gamepad1.dpad_up) {
                Rotation.setPosition(0);
                telemetry.addData("Rotate", "set to 0");
            }
            else if (gamepad1.dpad_down) {
                Claw.setPosition(0.5);
                Rotation.setPosition(0.24);//0.21
                telemetry.addData("Rotate", "set to 0.24");
                // sleep(300);
                //Intake.setPosition(0.3);
            }
            /*
            else if (!gamepad1.start && gamepad1.dpad_right) {
                Rotation.setPosition(0.05);
                telemetry.addData("Rotate", "set to 0.05");
            } else if (gamepad1.start && gamepad1.dpad_left) {
                Rotation.setPosition(0.1);
                telemetry.addData("Rotate", "set to 0.1");
            }
            */
            if (gamepad1.dpad_left) {
                Rotation.setPosition(Rotation.getPosition()-0.025);
                sleep(500);
            }
            if (gamepad1.dpad_right) {
                Rotation.setPosition(Rotation.getPosition()+0.025);
                sleep(500);
            }
            if (gamepad1.cross) {
                Claw.setPosition(0.5);
                Lift.setTargetPosition(0);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                Lift2.setTargetPosition(0);
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setPower(liftPow);
                while (Lift.isBusy() && Lift2.isBusy()) {
                    telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
                    telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
                    telemetry.addData("Current Position Lift2: ", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position Lift2: ", Lift2.getTargetPosition());
                    telemetry.update();
                }

            }
            if (gamepad1.triangle) {
                Lift.setTargetPosition(2000);//To be updated, Belt is loose
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                Lift2.setTargetPosition(2000);//To be updated, Belt is loose
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setPower(liftPow);
                while (Lift.isBusy() && Lift2.isBusy()) {
                    telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
                    telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
                    telemetry.addData("Current Position Lift2: ", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position Lift2: ", Lift2.getTargetPosition());
                    telemetry.update();
                }
            }
            if (gamepad1.options && gamepad1.triangle) {
                Wrist.setPosition(0);
                Lift.setTargetPosition(3600);//To be updated, Belt is loose
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                Lift2.setTargetPosition(3600);//To be updated, Belt is loose
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setPower(liftPow);
                while (Lift.isBusy() && Lift2.isBusy()) {
                    telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
                    telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
                    telemetry.addData("Current Position Lift 2:", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position Lift 2:", Lift2.getTargetPosition());
                    telemetry.update();
                }
            }

            if (gamepad1.square){
                Lift.setTargetPosition(Lift.getCurrentPosition()-100);
                Lift2.setTargetPosition(Lift2.getCurrentPosition()-100);
                Lift.setPower(liftPow);
                Lift2.setPower(liftPow);
                telemetry.addData("Current Pos Lift:", Lift.getCurrentPosition());
                telemetry.addData("Current Pos Lift2: ", Lift2.getCurrentPosition());
            }

            if (gamepad1.circle){
                Lift.setTargetPosition(Lift.getCurrentPosition()+100);
                Lift2.setTargetPosition(Lift2.getCurrentPosition()+100);
                Lift.setPower(liftPow);
                Lift2.setPower(liftPow);
                telemetry.addData("Current Pos Lift: ", Lift.getCurrentPosition());
                telemetry.addData("Current Pos Lift2: ", Lift2.getCurrentPosition());
            }
            if (gamepad1.options && gamepad1.left_bumper) { //Close
                Claw.setPosition(0.77);
            } else if (gamepad1.options && gamepad1.right_bumper) { //Open
                Claw.setPosition(0.5);
            }
            telemetry.update();
            LF.setPower(0);
            RF.setPower(0);
            LB.setPower(0);
            RB.setPower(0);
            //Intake.setPower(0.5);
            //If we set the power to 0 for the slides, then we have to keep thr buttons pressed
            //Lift.setPower(0);
            //Lift2.setPower(0);
        }
    }

}