import lejos.hardware.motor.Motor;
import lejos.hardware.ev3.EV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.robotics.SampleProvider;



public class LineFollowerMulti{
    public static void main(String[] args){
        LightSensor lightData = new LightSensor();
        UltraSensor ultradata = new UltraSensor();
        calculator calculator = new calculator(lightData, ultradata);
    
        Thread lightThread = new Thread(lightData);
        Thread ultraThread = new Thread(ultradata);
        Thread calculatorThread = new Thread(calculator);
        Thread motorThread = new Thread(new Motors(calculator));
    
        lightThread.start();
        ultraThread.start();
        calculatorThread.start();
        motorThread.start();
    }
}


class calculator implements Runnable{
    private static final float KP = 1000.0f;
    private static final float KI = 0.0f;
    private static final float KD = 100.0f;
    private static final int BASE_SPEED = 100;
    private static final int MAX_SPEED = 300;
    private static final float OBSTACLE_DISTANCE = 0.5f;
    
    private static float integral = 0;
    private static float lastError = 0;
    

    private int TrueSpeedA;
    private int TrueSpeedB;

    //lightSensor Object
    private LightSensor lightSensor;
    private UltraSensor ultraSensor;
    public calculator(LightSensor lightSensor, UltraSensor ultraSensor){
        this.lightSensor = lightSensor;
        this.ultraSensor = ultraSensor;
    }

    public void run(){

    
        while (true) {
            float colorSample = lightSensor.GetLight();
            float distanceSample = ultraSensor.getDistance();

            //colorSample = colorSample[0]
            float backgroundValue = 0.8f;
            float lineValue = 0.2f;
            float targetValue = (lineValue + backgroundValue) / 2;
            

            //colorProvider.fetchSample(colorSample, 0);
            float currentValue = colorSample;
                
            float error = targetValue - currentValue;
                
            integral = integral * 0.5f + error;
            float derivative = error - lastError;
            float correction = KP * error + KI * integral + KD * derivative;
                
                
                
            int speedA = (int)(BASE_SPEED + correction);
            int speedB = (int)(BASE_SPEED - correction);
                
            speedA = Math.min(Math.max(speedA, -MAX_SPEED), MAX_SPEED);
            speedB = Math.min(Math.max(speedB, -MAX_SPEED), MAX_SPEED);
            
            TrueSpeedA = speedA;
            TrueSpeedB = speedB;

            LCD.drawString("" + speedA, 0, 0);
        }
    }
    public int GetSpeedA(){
        return TrueSpeedA;
    }
    public int GetSpeedB(){
        return TrueSpeedB;
    }
}



class Motors implements Runnable {
    private calculator Calculator;
    public Motors(calculator Calculator){
        this.Calculator = Calculator;
    }

    public void run(){

        int speedA = Calculator.GetSpeedA();
        int speedB = Calculator.GetSpeedB();

        while (true) {
        

            Motor.A.setSpeed(Math.abs(speedA));
            Motor.B.setSpeed(Math.abs(speedB));

                    //>
            if (speedA > 0) {
                Motor.A.forward();
            } else {
                Motor.A.backward();
            }
            
            if (speedB > 0) {
                Motor.B.forward();
            } else {
                Motor.B.backward();
            }

        }
    }
}

class UltraSensor implements Runnable {
    private float distance = Float.MAX_VALUE;

    EV3UltrasonicSensor ultrasonicSensor = new EV3UltrasonicSensor(SensorPort.S1);
    SampleProvider distanceMode = ultrasonicSensor.getDistanceMode();
    float[] distanceSample = new float[distanceMode.sampleSize()];

    public float getDistance(){
        return distance;
    }

    public void run(){
        while (true) {
            distanceMode.fetchSample(distanceSample, 0);
            distance = distanceSample[0];
        }
        


    }
}

class LightSensor implements Runnable {
    private float Light = Float.MAX_VALUE;

    EV3ColorSensor LightSensor = new EV3ColorSensor(SensorPort.S2);
    SampleProvider LightProvider = LightSensor.getRedMode(); //getRedMode or getambientmode??
    float[] LightSample = new float[LightProvider.sampleSize()];

    public float GetLight(){
        return Light;
    }

    public void run(){
        

        while (true) {
            LightProvider.fetchSample(LightSample, 0);
            Light = LightSample[0];
            //LCD.drawString("Light: " + Light, 0, 0);

            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}