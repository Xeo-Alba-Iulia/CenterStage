package org.firstinspires.ftc.teamcode.utilities;

public class ServoSmoothing {

    public static double servoSmoothing(double currPos, double targetPos){
        double smoothedPos;
        smoothedPos = (targetPos*0.05)+(currPos*0.95);
    return smoothedPos;
    }
}
//daca curent pos<0.1
//      Pwm Range facem mai mic
//altfel daca curent pos>servo outtake-0.1
//      Pwm Range facem mai mic
//altfel
//      Pwn Range

//daca target-curent<0.1
//    pwm scade
//altfel
//    pwm default
