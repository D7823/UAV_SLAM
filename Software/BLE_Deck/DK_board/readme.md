The projects in this folder aims at the NRF52832 development board. But they could also be deployed to the BLE DECK.

Require tools:
  KEIL uVersion4: project build and download and debug
  (optional)nRFgo studio: program nrf52862 chip
  (optional)J-Link RTT viewer: monitor the Log via RTT

All project files are under selected_project/pca10040/s132/arm4 
The ready hex files are under selected_project/pca10040/s132/arm4/_build

Rebuild the project: (please reference the usage of KEIL tool for most accurate building process)
  Prepare KEIL and nRFgo studio, download the NRF52 SDK 15.000 and we would use the s132.
  1. Build project and generate hex file in KEIL
    a. double click the keil project file in any of the projects under arm4 directory
    b. After editing the code, build the project and make sure you choose generating hex file at the project configuration
    c. The application hex file should be located at the _build directory

  2. Download the hex file to the NRF52832 chip in KEIL
  
  (optional)3. You might use the nRFgo studio to program the chip
                a. open the tool and click on the nrf52 boards at the device manage window.
                b. click “erase all” at the center window.
                c. program Softdevice before programming your application (both of them need the hex files)
                    Location of the Softdevice: NRF52 SDK 15.00/components/softdevice/s132/hex. Download the SDK15.0 in the Nordic website.
  
  
Important website, resource, application:
  Information center: http://infocenter.nordicsemi.com/index.jsp
  Document library: https://www.nordicsemi.com/DocLib
  Community: https://devzone.nordicsemi.com/
  SDK download: https://developer.nordicsemi.com/nRF5_SDK/
  Sparkfun nrf52832 breakout board: 
      https://learn.sparkfun.com/tutorials/nrf52832-breakout-board-hookup-guide?_ga=2.173242276.683581879.1533582992-836203132.1530127885




  
