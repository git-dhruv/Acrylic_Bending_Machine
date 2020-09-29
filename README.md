# Acrylic_Bending_Machine
## Description
Master code for Acrylic Bending Machine. Developed by Dhruv Parikh.
There are two variants of the code.
1. Using Max 6675 Thermocouple
2. Using any standard analog temperature sensor

## Max 6675 Thermocouple
### Connections
- Pushbutton -> Pin 2
- Nichrome PWM -> Pin 3
- Red LED -> 13
- Green LED -> 12
- SCK -> 6
- CS -> 5
- SO -> 4

## Algorithm
PI temperature controller with Exponential Smoothing for temperature signal filtering. Change the coefficient alpha as per your needs.

## Disclaimer
Author takes no gaurentee for any failure of components using the code given above. This is not for a professional application. 
## Author
Dhruv Parikh
