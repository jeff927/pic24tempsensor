# pic24tempsensor
Sensor Data Acquisition System project using PIC24 microcontroller connected to Microstick II. 
This project also uses a 3x4 keypad and 2 lines x 16 characters LCD
The goal of this project is to create a system that will measure and save up to ten readings from an LM60 Temperature sensor.
The project has the following functionality:
*Power on self check. red LED blinks three times. OK message.
*Menu Display options: 1)read, 2)save, 3)recall and 4) clear 
*Press 1 to read sensor and display reading on LCD. Blink LED once.
*Press 2 to save sensor reading and acknowledge save on LCD. Blink LED twice.
*Press 3 to recall a saved reading. Ask for reading number.
*Press 4 to clear reading without saving. Blink LED once.
*Press 5 to delete a saved reading from a particular slot.
*After any selection is finished, it returns to the Display options screen.
