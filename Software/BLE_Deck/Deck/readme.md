
Deck contains the project used in the crazyflie for receiveing Beacons'RSSI.

Set up for experiment in the paper:
1. For the BLE Deck: program it with the nef52xxaa hex file, reference the readme file in the DK_board folder for how to program the nrf52832
   The hex file can be rebuilt through the ble zip project, reference the readme file in the DK_board folder for how to build nrf projects  
2. add the hello.c source file into the crazyflie driver source file folder
   follow the crazyflie_make&config document to modify the make file and config file to add the BLE deck into project
   reference this crazyflie website for more details on how to add new decks:
   https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/howto/
3. run the python file for crazyflie controlling and data collecting


