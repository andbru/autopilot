# autopilot

This is my project to create an autopilot for my boat. I started out with arduinomega but I had difficulties to get a compass signal that was both fast an with low noise. I used Madgwicks sensor fusion algorithm but the arduino just managed to get magnetometer samples at 20 Hz. That gave a very noisy signal.

My boat is very volatile in low speeds. In speeds above 8 knots there are no problems.

This project uses a Raspberry Pi 3 and a MPU9250 for the AHRS. More documentation will be published here in beginning of 2017.

Anders
