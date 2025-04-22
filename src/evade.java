import lejos.hardware.motor.Motor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.lcd.LCD;
import lejos.hardware.Button;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class evade {
    public static void main(String[] args) {
        UltraThread ultraThread = new UltraThread(); // Shared instance

        Thread motorThread = new Thread(new MotorThread(ultraThread)); // Pass it to MotorThread
        Thread ultraSensorThread = new Thread(ultraThread);

        ultraSensorThread.start();
        motorThread.start();
    }
}

class MotorThread implements Runnable  {
    private UltraThread ultraThread;
    private int turncounter = 0;
    private int forwardcounter = 0;


    public MotorThread(UltraThread ultraThread) {
        this.ultraThread = ultraThread;
    }

    public void run() {
        while (true) {
            float distance = ultraThread.getDistance(); // Get the current distance from the sensor

            // Drive forward by default
            Motor.A.forward();
            Motor.B.forward();

            if (distance <= 0.5f) {
                // Detected an obstacle, rotate to avoid it
                Motor.A.rotate(-180, true);
                Motor.B.rotate(180);
                turncounter++; // Increment turn counter

                try {
                    Thread.sleep(100); // Pause briefly after rotating
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                // Check if the path is clear again
                if (ultraThread.getDistance() >= 0.5f) {
                    // Move forward and straighten the robot
                    Motor.A.rotate(500, true);
                    Motor.B.rotate(500);
                    Motor.A.rotate(180, true);
                    Motor.B.rotate(-180);
                    forwardcounter++; // Increment forward counter

                    // Return to the original path
                    while (turncounter > 0 || forwardcounter > 0) {
                        if (forwardcounter > 0) {
                            // Reverse one forward movement
                            Motor.A.rotate(-500, true);
                            Motor.B.rotate(-500);
                            forwardcounter--;
                        }

                        if (turncounter > 0) {
                            // Reverse one turn
                            Motor.A.rotate(180, true);
                            Motor.B.rotate(-180);
                            turncounter--;
                        }
                    }
                }
            }
        }
    }
}

class UltraThread implements Runnable  {
    private float distance = Float.MAX_VALUE; 

    public float getDistance() {
        return distance;
    }

    public void run() {
        EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
        SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
        float[] sample = new float[distanceMode.sampleSize()];

        while (true) {
            distanceMode.fetchSample(sample, 0);
            distance = sample[0]; // Update shared variable

            //LCD.clear();
            //LCD.drawString("Distance: " + distance, 0, 0);

            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }     
    }
}
