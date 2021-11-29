# History

## 26 Nov 2021

Working on the stability of the code.  Infrequently, every 1 or 2 days, the system will reboot due to watchdog timeout.  While I've not yet spotted it happening live in the logs with a terminal attached I believe what is happening is that after a long delay in establishing the wifi connection if the send to the MQTT broker is also slow it could extend beyond the length of time of the watchdog.  In connect Wifi I was not resetting the watchdog timer after the system eventually connected.  Also in this section of the code I decreased the retries, as with 10 it was getting close to the watchdog timeout limit.  Additionally after timing the loop I did see instances where the time to send the message was over 2 times as long as average.  This also could get the near to the watchdog timeout limit.  Therefore I added Watchdog resets right before and after sending to MQTT.
