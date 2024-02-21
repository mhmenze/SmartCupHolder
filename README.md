# Smart Cup Holder

## Balancing

-Balances a cup using 2 servo motors in 2 axes based on readings from the MPU6050 sensor.

-The servo motors are controlled using Pololu Mini Maestro Servo Controller.

## Temperature sensing and control

-It uses the second core in ESP32 for temperature sensing.

-The temperature sensor is attached to the cup holder and sends readings to ESP32.

-The ESP32 board also features Bluetooth to send temperature readings to a mobile phone via a serial monitor app.

-ESP32 controls three LEDs based on temperature readings.\
             --- Red LED turns on if it's too hot.\
             --- Blue LED turns on if it's too cold.\
             --- Green LED turns on if it's safe to drink.

Demo Video - https://youtu.be/FQLWyY6ufs4
