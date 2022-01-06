package org.firstinspires.ftc.teamcode;
//Imports (Below)
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.lang.reflect.Field;

 

@TeleOp

public class FieldCentricTest extends LinearOpMode {

    //declaring objects (Below)
    private BNO055IMU imu;
    DcMotor LeftFront;
    DcMotor LeftRear;
    DcMotor RightFront;
    DcMotor RightRear;
    DcMotor ArmMotor;
    DcMotor StarIntake;
    DcMotor RightCarouselSpinner;
    DcMotor LeftCarouselSpinner;


    //Functions (Below) (Get Heading)
    private double GetHeading() {
        float IMUAngle;
        double ModdedAngle;

        IMUAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (IMUAngle < 0) {
            ModdedAngle = 360 + IMUAngle;
        } else {
            ModdedAngle = IMUAngle;
        }
        return ModdedAngle;
    }


    private void MoveInches(double Inches, double Power, String Direction) {
        double TargetDistance = 0;
        //Directions and TargetDistance for each Direction
        if (Direction.equals("Forward")) {
            LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            RightFront.setDirection(DcMotorSimple.Direction.REVERSE);
            RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
            TargetDistance = Inches;
        }
        if (Direction.equals("Backward")) {
            LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
            // Multiply by -1 for opposite direction
            TargetDistance = Inches * -1;
        }
        if (Direction.equals("Left")) {
            LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            RightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            //Multiply by 1.05 for correction value
            TargetDistance = Inches * 1.05;
        }
        if (Direction.equals("Right")) {
            LeftFront.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            RightRear.setDirection(DcMotorSimple.Direction.REVERSE);
            //Multiply by both -1 for directions and 1.05 for correction
            TargetDistance = (Inches * -1) * 1.05;
        }
        //Calculations
        //Wheel diameter = 3.78 inches
        //537.7 = Encoder Counts per Revolution
        //Calculating Circumference (d * pi)
        double DriveFactor = 3.78 * 3.14;
        //(Encoder Counts per Revolution * Gear ratio (1:1)) / Circumference
        double EncoderCountsPerInch = (537.7 * 1) / DriveFactor;
        double FinalEncoderCounts = EncoderCountsPerInch * TargetDistance;
        //Modes
        LeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Target Positions
        LeftFront.setTargetPosition((int) FinalEncoderCounts);
        LeftRear.setTargetPosition((int) FinalEncoderCounts);
        RightRear.setTargetPosition((int) FinalEncoderCounts);
        RightFront.setTargetPosition((int) FinalEncoderCounts);
        //More Modes
        LeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //Power
        LeftFront.setPower(Power * 0.8);
        sleep(5000);
        LeftRear.setPower(Power * 0.8);
        sleep(5000);
        RightRear.setPower(Power * 0.8);
        sleep(5000);
        RightFront.setPower(Power * 0.8);
        while (!(!LeftFront.isBusy() && !LeftRear.isBusy() && !RightFront.isBusy() && !RightRear.isBusy())) {
            sleep(100);
        }
    }
        public void runOpMode() {
            // In init

            //Declaring Variables (Below)
            double JoystickAngle = 0;
            double Horizontal = 0;
            double Vertical = 0;
            double Pivot = 0;
            double ArmPower = 0.75;


            //Mapping Config objects to variables(Below)
            LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
            LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
            RightFront = hardwareMap.get(DcMotor.class, "RightFront");
            RightRear = hardwareMap.get(DcMotor.class, "RightRear");
            ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
            StarIntake = hardwareMap.get(DcMotor.class, "StarIntake");
            RightCarouselSpinner = hardwareMap.get(DcMotor.class, "RightCarouselSpinner");
            LeftCarouselSpinner = hardwareMap.get(DcMotor.class, "LeftCarouselSpinner");
            imu = hardwareMap.get(BNO055IMU.class, "imu");

            //IMU init (Below)
            BNO055IMU.Parameters ImuParameters = new BNO055IMU.Parameters();
            imu.initialize(ImuParameters);

            //Directions (Below)
            LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
            RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
            ArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);
            LeftCarouselSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
            RightCarouselSpinner.setDirection(DcMotorSimple.Direction.FORWARD);


            //Target Position(Below)
            ArmMotor.setTargetPosition(0);
            StarIntake.setTargetPosition(0);


            //Modes (Below)
            LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            StarIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RightCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LeftCarouselSpinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


            LeftFront.setPower(0);
            RightRear.setPower(0);
            RightFront.setPower(0);
            LeftRear.setPower(0);


            waitForStart();
            // In Play


            boolean FieldCentric = true;
            //boolean used to activate and deactivate field centric at any moment


            while (opModeIsActive()) {


                double IMUy = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle;
                double IMUz = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                double IMUx = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;

            /*telemetry.addData("IMU Y", IMUy);
            telemetry.addData("IMU Z", IMUz);
            telemetry.addData("IMU X", IMUx);
            telemetry.update();

             */

                MoveInches(5,0.25, "Forward");
                sleep(2000);
                /*
                MoveInches(5,0.5,"Backward");
                sleep(1000);
                MoveInches(5,0.5,"Left");
                sleep(1000);
                MoveInches(5,0.5,"Right");
                sleep(1000);
                */


                Vertical = -gamepad1.left_stick_y;
                Horizontal = gamepad1.left_stick_x;
                Pivot = gamepad1.right_stick_x;
                // Above, Getting Joystick Values (For both robot and field centric)

                if (!FieldCentric) {
                    //Robot Centric
                    RightFront.setPower(-Pivot + (Vertical - Horizontal));
                    RightRear.setPower(-Pivot + (Vertical + Horizontal));
                    LeftFront.setPower(Pivot + (Vertical + Horizontal));
                    LeftRear.setPower(Pivot + (Vertical - Horizontal));
                }
                if (FieldCentric) {
                    //Field Centric
                    JoystickAngle = Math.atan2(Vertical, Horizontal) / Math.PI * 180;

                    // deals with crossing O
                    if (JoystickAngle < 0) {
                        JoystickAngle = JoystickAngle + 360;
                    }


                    //manipulating Joystick Angles to be Field Centric
                    double GetHeading = GetHeading();
                    double DriveAngle = (JoystickAngle - GetHeading) + 180;
                    double Length = Math.sqrt(Math.pow(Horizontal, 2) + Math.pow(Vertical, 2));
                    double ModdedVertical = Length * Math.sin(DriveAngle / 180 * Math.PI);
                    double ModdedHorizontal = Length * Math.cos(DriveAngle / 180 * Math.PI);

                    //telemetry to test
                    telemetry.addData("IMU Y", IMUy);
                    telemetry.addData("IMU Z", IMUz);
                    telemetry.addData("IMU X", IMUx);
                    telemetry.addData("Horizontal", Horizontal);
                    telemetry.addData("Vertical", Vertical);
                    telemetry.addData("JoystickAngle", JoystickAngle);
                    telemetry.addData("GetHeading", GetHeading);
                    telemetry.addData("DriveAngle", DriveAngle);
                    telemetry.addData("ModdedVertical", ModdedVertical);
                    telemetry.update();


                    //setting powers
                    RightFront.setPower((-Pivot) - (ModdedVertical - ModdedHorizontal));
                    RightRear.setPower((-Pivot) - (ModdedVertical + ModdedHorizontal));
                    LeftFront.setPower(Pivot - (ModdedVertical + ModdedHorizontal));
                    LeftRear.setPower(Pivot - (ModdedVertical - ModdedHorizontal));



                }


                double RightTrigger = gamepad1.right_trigger;

                if ((gamepad1.dpad_down && (RightTrigger < 0.3)) || (gamepad2.dpad_down && (RightTrigger < 0.3))) {
                    //arm = Floor (Back)
                    ArmMotor.setPower(ArmPower);
                    ArmMotor.setTargetPosition(0);
                    telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());
                    telemetry.addData("down", 1);
                    telemetry.update();
                } else if ((gamepad1.dpad_right && (RightTrigger < 0.3)) || (gamepad2.dpad_right && (RightTrigger < 0.3))) {
                    //arm = 1st tier (Back)
                    ArmMotor.setPower(ArmPower);
                    ArmMotor.setTargetPosition(380);
                    telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());
                    telemetry.addData("right", 1);
                    telemetry.update();
                } else if ((gamepad1.dpad_left && (RightTrigger < 0.3)) || (gamepad2.dpad_left && (RightTrigger < 0.3))) {
                    //arm = 2nd tier (Back)
                    ArmMotor.setPower(ArmPower);
                    ArmMotor.setTargetPosition(760);
                    telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());
                    telemetry.addData("left", 1);
                    telemetry.update();
                } else if ((gamepad1.dpad_up && (RightTrigger < 0.3)) || (gamepad2.dpad_up && (RightTrigger < 0.3))) {
                    //arm= 3rd tier (Back)
                    ArmMotor.setPower(ArmPower);
                    ArmMotor.setTargetPosition(950);
                    //760-1000
                    telemetry.addData("Arm Position", ArmMotor.getCurrentPosition());
                    telemetry.addData("up", 1);
                    telemetry.update();
                } else if (gamepad1.dpad_down && (RightTrigger > 0.3) || (gamepad2.dpad_down && (RightTrigger > 0.3))) {
                    //arm = Floor (Front)
                    ArmMotor.setPower(ArmPower);
                    ArmMotor.setTargetPosition(3250);
                } else if (gamepad1.dpad_right && (RightTrigger > 0.3) || (gamepad2.dpad_right && (RightTrigger > 0.3))) {
                    //arm = 1st tier (Front)
                    ArmMotor.setPower(ArmPower);
                    ArmMotor.setTargetPosition(2862);
                } else if (gamepad1.dpad_left && (RightTrigger > 0.3) || (gamepad2.dpad_left && (RightTrigger > 0.3))) {
                    //arm = 2nd tier (Front)
                    ArmMotor.setPower(ArmPower);
                    ArmMotor.setTargetPosition(2517);
                } else if (gamepad1.dpad_up && (RightTrigger > 0.3) || (gamepad2.dpad_up && (RightTrigger > 0.3))) {
                    //arm = 3rd tier (Front)
                    ArmMotor.setPower(ArmPower);
                    ArmMotor.setTargetPosition(2100);
                }


                //telemetry.addData("Arm Position", Arm.getCurrentPosition());
                //telemetry.addData("TRIGGER VALUE", RightTrigger);


                if (gamepad1.a || gamepad2.a) {
                    //A = activates intake
                    StarIntake.setDirection(DcMotorSimple.Direction.REVERSE);
                    StarIntake.setPower(0.5);
                } else if (gamepad1.b || gamepad2.b) {
                    //B = activate outake
                    StarIntake.setDirection(DcMotorSimple.Direction.FORWARD);
                    StarIntake.setPower(0.5);
                } else if (gamepad1.x || gamepad2.x) {
                    //X = Turns off Intake
                    StarIntake.setPower(0);
                }

                if (gamepad1.y) {
                    LeftCarouselSpinner.setDirection(DcMotorSimple.Direction.REVERSE);
                    RightCarouselSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
                    RightCarouselSpinner.setPower(0.75);
                    LeftCarouselSpinner.setPower(0.75);
                } else {
                    RightCarouselSpinner.setPower(0);
                    LeftCarouselSpinner.setPower(0);
                }
            }
        }
    }


    
