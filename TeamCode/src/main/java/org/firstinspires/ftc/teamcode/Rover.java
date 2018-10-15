package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Rover
{
    private static final double LIFT_SPEED = 5;
    private static final int[] servoPositions = {0, 180};

    //Declaraton of the hardwaremap object
    private HardwareMap hwMap;

    //Declaration of wheel motors
    private DcMotor left_front = null;
    private DcMotor left_rear = null;
    private DcMotor right_front = null;
    private DcMotor right_rear = null;

    /*
    Declaration of power set to each wheel
    0 = front left
    1 = rear left
    2 = front right
    3 = rear right
    */
    private double[] wheelPower;

    /*private double frontLeftPower;
    private double rearLeftPower;
    private double frontRightPower;
    private double rearRightPower;*/

    //Declaration of arm motors
    private DcMotor clamp_arm1 = null;
    private DcMotor clamp_arm2 = null;

    //Declaration of arm servos
    private Servo lander_clamp = null;
    private Servo unobtanium_turbine = null;
    private Servo unobtanium_arm = null;

    public Rover()
    {}

    public void init(HardwareMap ahwMap)
    {
        //Set hwMap to parameter
        hwMap = ahwMap;

        //Set motor/servo variables to motors/servos in hwMap
        left_front = hwMap.get(DcMotor.class, "left_front");
        left_rear = hwMap.get(DcMotor.class, "left_rear");
        right_front = hwMap.get(DcMotor.class, "right_front");
        right_rear = hwMap.get(DcMotor.class, "right_rear");

        clamp_arm1 = hwMap.get(DcMotor.class, "clamp_arm_1");
        clamp_arm2 = hwMap.get(DcMotor.class, "clamp_arm_2");

        lander_clamp = hwMap.get(Servo.class, "lander_clamp");

        unobtanium_turbine = hwMap.get(Servo.class, "mineral_turbine");
        unobtanium_arm = hwMap.get(Servo.class, "mineral_arm");

        //Sets motors to the correct directions
        left_front.setDirection(DcMotor.Direction.FORWARD);
        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.REVERSE);

        clamp_arm1.setDirection(DcMotor.Direction.FORWARD);
        clamp_arm2.setDirection(DcMotor.Direction.REVERSE);

        //Sets rovers power to 0
        left_front.setPower(0);
        left_rear.setPower(0);
        right_front.setPower(0);
        right_rear.setPower(0);

        clamp_arm1.setPower(0);
        clamp_arm2.setPower(0);

        wheelPower = new double[4];

        /*frontLeftPower = 0;
        rearLeftPower = 0;
        frontRightPower = 0;
        rearRightPower = 0;*/
    }

    /**
     * Sets the speed corresponding to forward movement on the remote to each wheel motor
     *
      * @param speed = the speed each wheel motor will be set to (negative for reverse)
     */
    public void setForwardSpeed(double speed)
    {
        if(checkStop(speed))
            stop();

        wheelPower[0] = speed;
        wheelPower[1] = speed;
        wheelPower[2] = speed;
        wheelPower[3] = speed;
    }

    /**
     * Sets the speed corresponding to right rotation on the remote to each wheel motor, overwriting the forward speed if not 0
     *
     * Precondition: setForwardSpeed() (and checkStop()) has already been activated
     * Precondition: move() has been activated between uses of this method
     *
     * @param speed = the speed the wheels will rotate to rotate the robot
     */
    public void setTurnSpeed(double speed)
    {
        if(speed != 0)
        {
            wheelPower[0] = speed;
            wheelPower[1] = speed;
            wheelPower[2] = speed;
            wheelPower[3] = speed;
        }
    }

    /**
     * Adds the speed corresponding to right movement on the remote to each wheel motor
     *
     * Precondition: setForwardSpeed() has already been activated
     * Precondition: move() has been activated between uses of this method
     *
     * @param speed = the speed the wheels will turn to move the robot
     */
    public void setStrafeSpeed(double speed)
    {
        wheelPower[0] += speed;
        wheelPower[1] += speed;
        wheelPower[2] -= speed;
        wheelPower[3] -= speed;
    }

    /**
     * Activates the motors to move the speed the above methods called for then resets the power variables
     */
    public void move()
    {
        right_rear.setPower(wheelPower[0]);
        left_rear.setPower(wheelPower[1]);
        left_front.setPower(wheelPower[2]);
        right_front.setPower(wheelPower[3]);
    }

    //Stops the robot by reversing the power of the wheels then setting them to 0
    public void stop()
    {
        for(int i = 0; i < wheelPower.length; i++)
        {
            wheelPower[i] = -wheelPower[i];
        }

        move();

        for(int i = 0; i < wheelPower.length; i++)
        {
            wheelPower[i] = 0;
        }

        move();
    }

    /**
     * A method stating whether or not the robot has stopped by comparing the input speed to the prior speeds
     * @param speed = the input speed from the controllers
     * @return if speed is 0 and no other speeds are
     */
    private boolean checkStop(double speed)
    {
        if(speed == 0) {
            for (double p : wheelPower)
            {
                if(p == 0)
                {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Sets the speeds/locations of the two servos powering the unobtanium arm
     *
     * @param inOrOut = Whether the turbine is moving the unobtanium in (1), out (2), or not moving (0)
     * @param upOrDown = Whether or not the arm is up
     */
    private void moveUnobtaniumArm(int inOrOut, boolean upOrDown)
    {
         if(upOrDown)
             unobtanium_arm.setPosition(servoPositions[0]);
         else
             unobtanium_arm.setPosition(servoPositions[1]);

         if(inOrOut == 0)
         {
             //set speed to 0
         }
         else
         {
             if(inOrOut == 1)
             {
                 //set in
             }
             else
             {
                 //set out
             }
         }
    }

    /**
     * Moves the motors and servos corrosponding to the lander clamp
     *
     * @param upOrDown = The direction of the clamp arm (0 = down, 1 = stopped, 2 = up)
     * @param onOrOff = Whetner or not the claw is engaged
     */
    private void moveClampArm(int upOrDown, boolean onOrOff)
    {
        if(upOrDown != 1)
        {
            if(upOrDown > 1)
            {
                clamp_arm1.setPower(LIFT_SPEED);
                clamp_arm2.setPower(LIFT_SPEED);
            }
            else
            {
                clamp_arm1.setPower(-LIFT_SPEED);
                clamp_arm2.setPower(-LIFT_SPEED);
            }
        }
        else
        {
            clamp_arm1.setPower(0);
            clamp_arm2.setPower(0);
        }

        if(onOrOff)
        {
            lander_clamp.setPosition(servoPositions[0]);
            lander_clamp.setPosition(servoPositions[1]);
        }
    }
}