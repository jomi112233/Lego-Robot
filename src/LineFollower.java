
import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.hardware.lcd.LCD;
import lejos.utility.Delay;

public class LineFollower {

    public static void main(String[] args) {

        int mult = 4;

        
        Motor.A.rotate(360 * mult, true);
        Motor.B.rotate(360 * mult);

        Delay.msDelay(1000);

        LCD.drawString("Rotating.", 0, 1);


        Motor.A.rotate(340, true);                         
        Motor.B.rotate(-340);  
        
        
        Delay.msDelay(1000);

        Motor.A.rotate(360 * mult, true);
        Motor.B.rotate(360 * mult);



        Delay.msDelay(1000);

        LCD.clear();
        LCD.drawString("Motors stopped.", 0, 1);
        
        Button.waitForAnyPress();
    }
}
