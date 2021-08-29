
#include "deck.h"
#include "param.h"

#define DEBUG_MODULE "SCANNER"

#include "system.h"
#include "debug.h"
#include "log.h"
#include "i2cdev.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>
static bool isInit = false;
static bool isTested = false;


static I2C_Dev *I2CX;  //pointer to the I2C peripheral
static uint16_t length=20;
static uint8_t rssi_value[20]={0};
static int8_t rssi[20]={0};

static void scannerTask(void *param)
{
  systemWaitStart();

  TickType_t lastWakeTime = xTaskGetTickCount(); //get tick time count

  while(1) {
    vTaskDelayUntil(&lastWakeTime, M2T(1000));   //delay some time get next data

    if(i2cdevRead(I2CX,0x74,length,rssi_value))
    {
    	//DEBUG_PRINT("Data received.\n");
        for(int i=0;i<length;i++)
        {
    	  rssi[i]=rssi_value[i]-128;
    	  //DEBUG_PRINT("rssi1: %f.\n",rssi[0]);
        }
        DEBUG_PRINT("rssi2: %d.\n",rssi_value[1]);
        DEBUG_PRINT("rssi2: %d.\n",rssi[1]);
    }
    //for testing the task
    //DEBUG_PRINT("test:%d.\n",rssi_value);
  }
}

static void scannerInit()             //initialize the pins, every deck has these functions
{
  if (isInit) {
    return;
  }
  i2cdevInit(I2C1_DEV);
  I2CX=I2C1_DEV;

  isInit = true;
//create a task thread which execute the detect task function
  xTaskCreate(scannerTask, "scanner", 2*configMINIMAL_STACK_SIZE, NULL,
              /*priority*/3, NULL);
}

static bool scannerTest()
{
  bool pass = isInit;

  if (isTested) {
    DEBUG_PRINT("Cannot test scanner deck a second time\n");
    return false;
  }
  DEBUG_PRINT("ScannerTest is good.\n");
  isTested = true;
  return pass;
}

static const DeckDriver scanner_deck = {
  .vid = 0,              //deck id address and name , new name to resolve conflict
  .pid = 0,
  .name = "blescanner",

  .usedGpio = 0,  // FIXME: set the used pins, future editing needed

  .init = scannerInit,
  .test = scannerTest,
};

DECK_DRIVER(scanner_deck);

PARAM_GROUP_START(deck)
PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, blescanner, &isInit)
PARAM_GROUP_STOP(deck)

LOG_GROUP_START(target)
LOG_ADD(LOG_INT8, rssi0, &rssi[0])
LOG_ADD(LOG_INT8, rssi1, &rssi[1])
LOG_ADD(LOG_INT8, rssi2, &rssi[2])
LOG_ADD(LOG_INT8, rssi3, &rssi[3])
LOG_ADD(LOG_INT8, rssi4, &rssi[4])
LOG_GROUP_STOP(target)
LOG_GROUP_START(anchor1) //might need to separate the group
LOG_ADD(LOG_INT8, rssi5, &rssi[5])
LOG_ADD(LOG_INT8, rssi6, &rssi[6])
LOG_ADD(LOG_INT8, rssi7, &rssi[7])
LOG_ADD(LOG_INT8, rssi8, &rssi[8])
LOG_ADD(LOG_INT8, rssi9, &rssi[9])
LOG_GROUP_STOP(anchor1)
LOG_GROUP_START(anchor2)
LOG_ADD(LOG_INT8, rssi10, &rssi[10])
LOG_ADD(LOG_INT8, rssi11, &rssi[11])
LOG_ADD(LOG_INT8, rssi12, &rssi[12])
LOG_ADD(LOG_INT8, rssi13, &rssi[13])
LOG_ADD(LOG_INT8, rssi14, &rssi[14])
LOG_GROUP_STOP(anchor2)
LOG_GROUP_START(anchor3)
LOG_ADD(LOG_INT8, rssi15, &rssi[15])
LOG_ADD(LOG_INT8, rssi16, &rssi[16])
LOG_ADD(LOG_INT8, rssi17, &rssi[17])
LOG_ADD(LOG_INT8, rssi18, &rssi[18])
LOG_ADD(LOG_INT8, rssi19, &rssi[19])
LOG_GROUP_STOP(anchor3)
