package org.firstinspires.ftc.teamcode;

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
private BNO055IMU imu;
//objects above

    // todo: write your code here
    private double GetHeading() {
        float IMUAngle;
        double ModdedAngle;
        
        IMUAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        if (IMUAngle < 0) {
            ModdedAngle = 360 + IMUAngle;
        } 
        else  {
            ModdedAngle = IMUAngle;
        }
        return ModdedAngle;
    }
public void runOpMode() {
    // In init
    
     DcMotor LeftFront;
     DcMotor LeftRear; 
     DcMotor RightFront;
     DcMotor RightRear;
     double JoystickAngle = 0;
     double Horizontal = 0;
     double Vertical = 0;
     double Pivot = 0;
     // Declaring Variables 
     
     //Mapping Config objects to variables
     LeftFront = hardwareMap.get(DcMotor.class, "LeftFront");
     LeftRear = hardwareMap.get(DcMotor.class, "LeftRear");
     RightFront = hardwareMap.get(DcMotor.class, "RightFront");
     RightRear = hardwareMap.get(DcMotor.class, "RightRear");
     imu = hardwareMap.get(BNO055IMU.class, "imu");
     
     //Directions 
     LeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
     RightFront.setDirection(DcMotorSimple.Direction.FORWARD);
     RightRear.setDirection(DcMotorSimple.Direction.FORWARD);
     LeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
     //Modes
     LeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     RightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     RightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
     LeftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    waitForStart();
    // After Play
    boolean FieldCentric = true;
    
    while (opModeIsActive()) {
        Vertical =  -gamepad1.left_stick_y;
        Horizontal = gamepad1.left_stick_x;
        Pivot = gamepad1.right_stick_x;
       // Above, Getting Joystick Values
        if(FieldCentric == true) {
            JoystickAngle = Math.atan2(Vertical, Horizontal)/ Math.PI * 180;

        }
            if(JoystickAngle < 0) {
                JoystickAngle = JoystickAngle + 360;
            }
            //Above, deals with crossing O 
            double DriveAngle = (JoystickAngle - GetHeading()) + 90;
            double Length = Math.sqrt(Math.pow(Horizontal,2)+ Math.pow(Vertical,2));
            double ModdedVertical = Length * Math.sin(DriveAngle/180 * Math.PI);
            double ModdedHorizontal = Length * Math.cos(DriveAngle/180 * Math.PI);
            // Above, Changing Joystick Angles to be Field Centric
            
            telemetry.addData("Horizontal", Horizontal);
            telemetry.addData("ModdedHorizontal", ModdedHorizontal);
            telemetry.update();
            
            RightFront.setPower((-Pivot) + (ModdedVertical - ModdedHorizontal));
            RightRear.setPower((-Pivot) + (ModdedVertical + ModdedHorizontal));
            LeftFront.setPower(Pivot + (ModdedVertical + ModdedHorizontal));
            LeftRear.setPower(Pivot + (ModdedVertical - ModdedHorizontal));
            
        }
    } 
}
    
