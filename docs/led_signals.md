# LED Indication

Our manipulator has three LEDs to indicate system status to the operator. Each sequence and LED blinking has a meaning. In this file we explain each of them and the possible errors.


| LEVEL | RED | YELLOW | GREEN | Posible situations |
|--------|-----|--------|-------|--------------------|
| WAITING |<img src="images/LED_RED_OFF.png" alt="Red LED off" width="75">|<img src="images/LED_YELLOW_BLINKING.gif" alt="Yellow LED blinking" width="75">|<img src="images/LED_GREEN_OFF.png" alt="Green LED off" width="75"> | Searching for serial communication |
| SUCESS |<img src="images/LED_RED_OFF.png" alt="Red LED off" width="75">|<img src="images/LED_YELLOW_OFF.png" alt="Yellow LED off" width="75">|<img src="images/LED_GREEN_BLINKING.gif" alt="Green LED blinking" width="75"> | Connecting to computer |
| RUNNING |<img src="images/LED_RED_OFF.png" alt="Red LED off" width="75">|<img src="images/LED_YELLOW_OFF.png" alt="Yellow LED off" width="75">|<img src="images/LED_GREEN_ON.png" alt="Green LED on" width="75"> | System running and connected to computer |
| ERROR |<img src="images/LED_RED_ON.png" alt="Red LED on" width="75">|<img src="images/LED_YELLOW_OFF.png" alt="Yellow LED off" width="75">|<img src="images/LED_GREEN_OFF.png" alt="Green LED off" width="75"> | No connection found |
| CRITICAL FAILURE |<img src="images/LED_RED_BLINKING.gif" alt="Red LED blinking" width="75">|<img src="images/LED_YELLOW_OFF.png" alt="Yellow LED off" width="75">|<img src="images/LED_GREEN_OFF.png" alt="Green LED off" width="75"> |  MUTEX creation failure |