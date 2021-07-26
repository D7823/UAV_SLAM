Ble&i2c(tested, used in demonstrating the connection between nrf52 and crazyflie via I2C): receive one beacon’s rssi and put the data to I2C channel for being read by crazyflie

Ble&i2c_data(not tested): receive one beacon’s rssi and put different data(rssi or position information) to I2C channel according to crazyflie’s command

Drone1(tested): Implement the nrf52 as both central and peripheral. As central it can connect to the peripheral role of drone2. It receives the rssi from one beacon and sent the rssi to the drone2.

Drone2(tested): Implement the nrf52 as both central and peripheral as drone1. As peripheral, it can be connected to drone1, receiving the data from drone1 and sending data back to drone1. As central, it can connect to drone3 and exchange data with drone3, but in this project, the drone2 doesn’t have the ability of receiving beacon rssi data as drone 1. Ideally, each drone should behave as the drone1.

Note: Drone1/2 needs further development, including receiving all 5 beacons data, creating I2C services, sending data with certain formatting according to the crazyflie’s command,
      which is integrating all the above functionality in those projects into a single one project(all in one below)

Ble(tested, used in experiment to collect one beacon’s rssi value): receive one beacon’s rssi

Double beacon(tested): receive two beacons’ rssi

Multi beacons(tested, used in experiment to collect five beacon’s rssi values): receive five beacons’rssi and able to expand the number if knowing the peer address of the beacons

Uart-data_f(tested): implement the nrf52 as the ble peripheral, exchange data with the ble central

Uart_c-data_format(tested): implement the nrf52 as the ble central, exchange data with the ble peripheral
