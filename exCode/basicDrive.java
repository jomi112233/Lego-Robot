import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.Motor;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;   // allows the sensor to return the samples or data
                                        // e.g., for getting distance data from sonic sensor etc
import lejos.utility.Delay;

public class basicDrive {

    public static void main(String[] args) {
        // Creating an instance of US sensor at port 1
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        
        // Get the distance sample provider
        SampleProvider distance = ultrasonicSensor.getDistanceMode();
        
        // Create a sample array to hold the distance value
        // even though sonic sensor gives distance as an o/p, but since other sensors, e.g., light sensor
        // can provide multiple values, therefore to keep consistency, I'm using sampleprovider
        float[] sample = new float[distance.sampleSize()];
    
        // Keep displaying the distance, until user presses a button
        while (!Button.ESCAPE.isDown())
        {
            // Get the curRent distnce reading from the US sensor
            distance.fetchSample(sample, 0);
            
            // Display the distance on the LCD screen
            LCD.clear();
            LCD.drawString("Dist: " + sample[0] + " meters", 0, 0);
            

            Motor.A.forward();
            Motor.B.forward();

            if(sample[0] <= 1){
                Motor.A.rotate(-360, true);
                Motor.B.rotate(360);
                LCD.drawString("toimii", 0, 0);

            }
            // // Refresh display every 100 ms
            try {
                Thread.sleep(100);
           } catch (InterruptedException e) {
               e.printStackTrace();
           }

        }
        
        // Close US sensor
        ultrasonicSensor.close();
    }
}