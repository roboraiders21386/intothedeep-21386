package org.firstinspires.ftc.teamcode.opmodes.TELEOP;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "what are you looking at do your job")
public class TeleOpPS5Meet2ANDROIDSTUDIO extends LinearOpMode {
    private DcMotor Lift;
    private DcMotor Lift2;
    private DcMotor RF;
    private DcMotor LF;
    private DcMotor RB;
    private DcMotor LB;
    private Servo Sample;
    private Servo Rotation;
    private Servo Wrist;
    private double dp = 1;
    private double intPow = 0.5;
    private double liftPow = 1;
    private int LIFT_INCREMENT = 10;
    private int a = 100;
    private int b = -100;
    private int tcnt = 0;
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
        Sample = hardwareMap.get(Servo.class, "sample");
        Rotation = hardwareMap.get(Servo.class, "rotate");
        Lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift.setDirection(DcMotorSimple.Direction.REVERSE);
        Lift2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Lift2.setDirection(DcMotorSimple.Direction.FORWARD);
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
        Sample.setPosition(0);
        while (opModeIsActive()) {
            telemetry.addData("Lift 1 position:", Lift.getCurrentPosition());
            telemetry.addData("Lift 2 position:", Lift2.getCurrentPosition());
            telemetry.addData("Rotation position:", Rotation.getPosition());
            telemetry.addData("Sample claw position:", Sample.getPosition());
            telemetry.addData("Wrist position:", Wrist.getPosition());

            //Forward/Backward
            if (gamepad1.left_stick_y > 0.25) {
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
            if (gamepad1.left_stick_x > 0.25) {
                LF.setPower(-dp * gamepad1.left_stick_x);
                RF.setPower(dp * gamepad1.left_stick_x);
                LB.setPower(dp * gamepad1.left_stick_x);
                RB.setPower(-dp * gamepad1.left_stick_x);
            }
            //StartIntake
            boolean off = true;
            if (!gamepad1.start && gamepad1.right_bumper) {
                Sample.setPosition(0.3);
                off = false;
            }
            if (!gamepad1.start && gamepad1.left_bumper) {
                Sample.setPosition(0);
            }
            while (gamepad1.left_trigger>0) {
                Wrist.setPosition(0);
            }
            while (gamepad1.right_trigger>0) {
                Wrist.setPosition(1);
            }
            if (gamepad1.options && gamepad1.dpad_down) {
                //score specimen low
                Rotation.setPosition(0.24);//0.21
                telemetry.addData("Rotate", "set to 0.24");
                // sleep(300);
                //Intake.setPosition(0.3);
            }
            if (gamepad1.dpad_up) {
                //Grab Position
                Wrist.setPosition(0);
                Sample.setPosition(0.3);
                Rotation.setPosition(0.33);
                telemetry.addData("Rotate", "set to 0.33");
            }
            else if (!gamepad1.options && gamepad1.dpad_down) {
                //All the way backwards
                Rotation.setPosition(0);//0.25
                Wrist.setPosition(0.15);
                telemetry.addData("Rotate", "set to 0");
                // sleep(300);
                //Intake.setPosition(0.3);
            }

            if (!gamepad1.options && gamepad1.dpad_right) {
                //Pick Up Specimen from Human Player
                Rotation.setPosition(0.05);
                Wrist.setPosition(0.85);
                telemetry.addData("Rotate", "set to 0.05");
            } else if (!gamepad1.options && gamepad1.dpad_left) {
                //Midway position
                Rotation.setPosition(0.3);
                Wrist.setPosition(0);
                telemetry.addData("Rotate", "set to 0.3");
            }

            if (gamepad1.options && gamepad1.dpad_left) {
                //Decremtal
                Rotation.setPosition(Rotation.getPosition()-0.025);
                sleep(500);
            }
            if (gamepad1.options && gamepad1.dpad_right) {
                //Incremental
                Rotation.setPosition(Rotation.getPosition()+0.025);
                sleep(500);
            }
            if (gamepad1.cross) {
                //Align Straight and go down
                Rotation.setPosition(0.165);
                Wrist.setPosition(0);
                Lift.setTargetPosition(0);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                Lift2.setTargetPosition(0);
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setPower(liftPow);
                //while (Lift.isBusy() && Lift2.isBusy()) {
                    telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
                    telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
                    telemetry.addData("Current Position Lift2: ", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position Lift2: ", Lift2.getTargetPosition());
                    telemetry.update();
                //}


            }
            if (gamepad1.share && gamepad1.cross) {
                //Low Basket Sample
                Lift.setTargetPosition(1300);
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                Lift2.setTargetPosition(1300);
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setPower(liftPow);
                //while (Lift.isBusy() && Lift2.isBusy()) {
                    telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
                    telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
                    telemetry.addData("Current Position Lift2: ", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position Lift2: ", Lift2.getTargetPosition());
                    telemetry.update();
                //}
                Rotation.setPosition(0.1);

            }
            if (gamepad1.triangle) {
                if (tcnt%2==0) {
                    //Prepare for drop
                    Rotation.setPosition(0.16);
                    Wrist.setPosition(0);
                    Lift.setTargetPosition(400);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift.setPower(liftPow);
                    Lift2.setTargetPosition(400);
                    Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift2.setPower(-1 * liftPow);
                    //while (Lift.isBusy()) {
                    telemetry.addData("Current Position", Lift.getCurrentPosition());
                    telemetry.addData("Target Position", Lift.getTargetPosition());
                    telemetry.addData("Current Position", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position", Lift2.getTargetPosition());
                    telemetry.update();
                } else {
                    Rotation.setPosition(0.167);
                    Lift.setTargetPosition(1250);
                    Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift.setPower(liftPow);
                    Lift2.setTargetPosition(1250);
                    Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Lift2.setPower(-1 * liftPow);
                    //while (Lift.isBusy()) {
                    telemetry.addData("Current Position", Lift.getCurrentPosition());
                    telemetry.addData("Target Position", Lift.getTargetPosition());
                    telemetry.addData("Current Position", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position", Lift2.getTargetPosition());
                    telemetry.update();
                }
                tcnt++;
                /*Rotation.setPosition(0.17);
                Lift.setTargetPosition(2000);//To be updated, Belt is loose
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                Lift2.setTargetPosition(2000);//To be updated, Belt is loose
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setPower(liftPow);
                //while (Lift.isBusy() && Lift2.isBusy()) {
                    telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
                    telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
                    telemetry.addData("Current Position Lift2: ", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position Lift2: ", Lift2.getTargetPosition());
                    telemetry.update();
                //}*/
            }
            if (gamepad1.share && gamepad1.triangle) {
                Lift.setTargetPosition(3080);//To be updated, Belt is loose
                Lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift.setPower(liftPow);
                Lift2.setTargetPosition(3080);//To be updated, Belt is loose
                Lift2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Lift2.setPower(liftPow);
                //while (Lift.isBusy() && Lift2.isBusy()) {
                    telemetry.addData("Current Position Lift: ", Lift.getCurrentPosition());
                    telemetry.addData("Target Position Lift: ", Lift.getTargetPosition());
                    telemetry.addData("Current Position Lift 2:", Lift2.getCurrentPosition());
                    telemetry.addData("Target Position Lift 2:", Lift2.getTargetPosition());
                    telemetry.update();
                //}
                Rotation.setPosition(0.1);
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