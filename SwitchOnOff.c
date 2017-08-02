/**
 * @file SwitchOnOff.c
 * @copyright Copyright (c) 2001-2015
 * Sigma Designs, Inc.
 * All Rights Reserved�
 * @brief Z-Wave Switch On/Off Sample Application
 * @details This sample application is a Z-Wave slave node which has an LED (D2
 * on ZDP03A) that can be turned on or off from another Z-Wave node by sending
 * a Basic Set On or a Basic Set Off command.
 *
 * It can be included and excluded from a Z-Wave network by pressing S1 switch
 * on the ZDP03A board 3 times. S2 switch toggles LED D2. S3 switch transmits
 * a Node Information Frame (NIF).
 *
 * Last changed by: $Author: $
 * Revision:        $Revision: $
 * Last changed:    $Date: $
 *
 * @author Someone who started this sample application at some point in time.
 * @author Thomas Roll (TRO)
 * @author Christian Salmony Olsen (COLSEN)
 */

/****************************************************************************/
/*                              INCLUDE FILES                               */
/****************************************************************************/

#include "config_app.h"

#include <slave_learn.h>
#include <ZW_slave_api.h>
#ifdef ZW_SLAVE_32
#include <ZW_slave_32_api.h>
#else
#include <ZW_slave_routing_api.h>
#endif  /* ZW_SLAVE_32 */

#include <ZW_classcmd.h>
#include <ZW_mem_api.h>

#include <eeprom.h>
#include <ZW_uart_api.h>

#include <misc.h>
#ifdef BOOTLOADER_ENABLED
#include <ota_util.h>
#include <CommandClassFirmwareUpdate.h>
#endif
#include <nvm_util.h>

/*IO control*/
#include <io_zdp03a.h>
#include <ZW_task.h>
#include <ev_man.h>

#ifdef ZW_ISD51_DEBUG
#include "ISD51.h"
#endif

#include <association_plus.h>
#include <agi.h>
#include <CommandClassAssociation.h>
#include <CommandClassAssociationGroupInfo.h>
#include <CommandClassVersion.h>
#include <CommandClassZWavePlusInfo.h>
#include <CommandClassPowerLevel.h>
#include <CommandClassDeviceResetLocally.h>
#include <CommandClassBasic.h>
#include <CommandClassBinarySwitch.h>
#include <CommandClassSwitchAll.h>
#include <CommandClassSupervision.h>
#include <CommandClassMultiChan.h>
#include <CommandClassMultiChanAssociation.h>


/****************************************************************************/
/*                      PRIVATE TYPES and DEFINITIONS                       */
/****************************************************************************/
/**
 * @def ZW_DEBUG_MYPRODUCT_SEND_BYTE(data)
 * Transmits a given byte to the debug port.
 * @def ZW_DEBUG_MYPRODUCT_SEND_STR(STR)
 * Transmits a given string to the debug port.
 * @def ZW_DEBUG_MYPRODUCT_SEND_NUM(data)
 * Transmits a given number to the debug port.
 * @def ZW_DEBUG_MYPRODUCT_SEND_WORD_NUM(data)
 * Transmits a given WORD number to the debug port.
 * @def ZW_DEBUG_MYPRODUCT_SEND_NL()
 * Transmits a newline to the debug port.
 */
#ifdef ZW_DEBUG_APP
#define ZW_DEBUG_APP_SEND_BYTE(data) ZW_DEBUG_SEND_BYTE(data)
#define ZW_DEBUG_APP_SEND_STR(STR) ZW_DEBUG_SEND_STR(STR)
#define ZW_DEBUG_APP_SEND_NUM(data)  ZW_DEBUG_SEND_NUM(data)
#define ZW_DEBUG_APP_SEND_WORD_NUM(data) ZW_DEBUG_SEND_WORD_NUM(data)
#define ZW_DEBUG_APP_SEND_NL()  ZW_DEBUG_SEND_NL()
#else
#define ZW_DEBUG_APP_SEND_BYTE(data)
#define ZW_DEBUG_APP_SEND_STR(STR)
#define ZW_DEBUG_APP_SEND_NUM(data)
#define ZW_DEBUG_APP_SEND_WORD_NUM(data)
#define ZW_DEBUG_APP_SEND_NL()
#endif



/**
 * Application events for AppStateManager(..)
 */
typedef enum _EVENT_APP_
{
  EVENT_EMPTY = DEFINE_EVENT_APP_NBR,
  EVENT_APP_INIT,
  EVENT_APP_REFRESH_MMI,
  EVENT_APP_OTA_HOST_WRITE_DONE,
  EVENT_APP_OTA_HOST_STATUS,
} EVENT_APP;


/**
 * Application states. Function AppStateManager(..) includes the state
 * event machine.
 */
typedef enum _STATE_APP_
{
  STATE_APP_STARTUP,
  STATE_APP_IDLE,
  STATE_APP_LEARN_MODE,
  STATE_APP_WATCHDOG_RESET,
  STATE_APP_OTA,
  STATE_APP_OTA_HOST
} STATE_APP;


/****************************************************************************/
/*                              PRIVATE DATA                                */
/****************************************************************************/

/**
 * Unsecure node information list.
 * Be sure Command classes are not duplicated in both lists.
 * CHANGE THIS - Add all supported non-secure command classes here
 **/
static code BYTE cmdClassListNonSecureNotIncluded[] =
{
  COMMAND_CLASS_ZWAVEPLUS_INFO,
  COMMAND_CLASS_SWITCH_BINARY,
  COMMAND_CLASS_SWITCH_ALL,
  COMMAND_CLASS_ASSOCIATION,
  COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2,
  COMMAND_CLASS_ASSOCIATION_GRP_INFO,
  COMMAND_CLASS_TRANSPORT_SERVICE_V2,
  COMMAND_CLASS_VERSION,
  COMMAND_CLASS_MANUFACTURER_SPECIFIC,
  COMMAND_CLASS_DEVICE_RESET_LOCALLY,
  COMMAND_CLASS_POWERLEVEL,
  COMMAND_CLASS_SECURITY,
  COMMAND_CLASS_SECURITY_2,
  COMMAND_CLASS_SUPERVISION
#ifdef BOOTLOADER_ENABLED
  ,COMMAND_CLASS_FIRMWARE_UPDATE_MD_V2
#endif
};

/**
 * Unsecure node information list Secure included.
 * Be sure Command classes are not duplicated in both lists.
 * CHANGE THIS - Add all supported non-secure command classes here
 **/
static code BYTE cmdClassListNonSecureIncludedSecure[] =
{
  COMMAND_CLASS_ZWAVEPLUS_INFO,
  COMMAND_CLASS_TRANSPORT_SERVICE_V2,
  COMMAND_CLASS_SECURITY,
  COMMAND_CLASS_SECURITY_2
};


/**
 * Secure node inforamtion list.
 * Be sure Command classes are not duplicated in both lists.
 * CHANGE THIS - Add all supported secure command classes here
 **/
static code BYTE cmdClassListSecure[] =
{
  COMMAND_CLASS_VERSION,
  COMMAND_CLASS_SWITCH_BINARY,
  COMMAND_CLASS_SWITCH_ALL,
  COMMAND_CLASS_ASSOCIATION,
  COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2,
  COMMAND_CLASS_ASSOCIATION_GRP_INFO,
  COMMAND_CLASS_MANUFACTURER_SPECIFIC,
  COMMAND_CLASS_DEVICE_RESET_LOCALLY,
  COMMAND_CLASS_POWERLEVEL,
  COMMAND_CLASS_SUPERVISION
#ifdef BOOTLOADER_ENABLED
  ,COMMAND_CLASS_FIRMWARE_UPDATE_MD_V2
#endif
};


/**
 * Structure includes application node information list's and device type.
 */
APP_NODE_INFORMATION m_AppNIF =
{
  cmdClassListNonSecureNotIncluded, sizeof(cmdClassListNonSecureNotIncluded),
  cmdClassListNonSecureIncludedSecure, sizeof(cmdClassListNonSecureIncludedSecure),
  cmdClassListSecure, sizeof(cmdClassListSecure),
  DEVICE_OPTIONS_MASK, GENERIC_TYPE, SPECIFIC_TYPE
};

/**
 * AGI lifeline string
 */
const char GroupName[]   = "Lifeline";

/**
 * Setup AGI lifeline table from app_config.h
 */
CMD_CLASS_GRP  agiTableLifeLine[] = {AGITABLE_LIFELINE_GROUP};

/**
 * Application node ID
 */
BYTE myNodeID = 0;

/**
 * Application state-machine state.
 */
static STATE_APP currentState = STATE_APP_IDLE;

/**
 * Parameter is used to save wakeup reason after ApplicationInitHW(..)
 */
SW_WAKEUP wakeupReason;

/**
 * LED state
 */
static BYTE onOffState;

/**
 * Use to tell if the host OTA required auto rebooting or not
 */
BOOL userReboot = FALSE;

#ifdef APP_SUPPORTS_CLIENT_SIDE_AUTHENTICATION
s_SecurityS2InclusionCSAPublicDSK_t sCSAResponse = { 0, 0, 0, 0};
#endif /* APP_SUPPORTS_CLIENT_SIDE_AUTHENTICATION */

/****************************************************************************/
/*                              EXPORTED DATA                               */
/****************************************************************************/

// No exported data.

/****************************************************************************/
/*                            PRIVATE FUNCTIONS                             */
/****************************************************************************/

void ZCB_DeviceResetLocallyDone(TRANSMISSION_RESULT * pTransmissionResult);
void ZCB_Timer1Delay(void);
void ZCB_Timer2Delay(void);
void ZCB_Timer3Delay(void);
STATE_APP GetAppState();
void AppStateManager( EVENT_APP event);
void ChangeState( STATE_APP newState);

#ifdef BOOTLOADER_ENABLED
void ZCB_OTAFinish(OTA_STATUS otaStatus);
BOOL ZCB_OTAStart(void);
void ZCB_OTAWrite(BYTE *pData, BYTE len);
#endif

void LoadConfiguration(ZW_NVM_STATUS nvmStatus);
void SetDefaultConfiguration(void);

void ToggleLed(void);
void RefreshMMI(void);


/**
 * @brief See description for function prototype in ZW_basis_api.h.
 */
void
ApplicationRfNotify(BYTE rfState)
{
  UNUSED(rfState);
}

/*******************************switch************************************/
//-------------------relay----------------------
#define relay1_pin_out() SetPinOut(ZDP03A_LED_D1)
#define relay2_pin_out() SetPinOut(ZDP03A_LED_D2)
#define relay3_pin_out() SetPinOut(ZDP03A_LED_D3)

#define relay1_on()  Led(ZDP03A_LED_D1,ON)
#define relay1_off() Led(ZDP03A_LED_D1,OFF)
#define relay2_on()  Led(ZDP03A_LED_D2,ON)
#define relay2_off() Led(ZDP03A_LED_D2,OFF)
#define relay3_on()  Led(ZDP03A_LED_D3,ON)
#define relay3_off() Led(ZDP03A_LED_D3,OFF)

#define relay_init() \
	relay1_pin_out(); \
  relay2_pin_out(); \ 
  relay3_pin_out(); \
	relay1_off(); \
	relay2_off(); \
	relay3_off()

//-------------------led----------------------
#define led_nwk_pin_out() SetPinOut(ZDP03A_LED_D6)

#define led_nwk_on()  Led(ZDP03A_LED_D6,ON)
#define led_nwk_off() Led(ZDP03A_LED_D6,OFF)

#define led_init() \
   led_nwk_pin_out(); \
   led_nwk_off()

//-------------------key----------------------
#define EVENT_KEY1_DOWN            EVENT_KEY_B1_DOWN
#define EVENT_KEY1_UP              EVENT_KEY_B1_UP
#define EVENT_KEY1_HELD            EVENT_KEY_B1_HELD
#define EVENT_KEY1_PRESS           EVENT_KEY_B1_PRESS
#define EVENT_KEY1_HELD_10_SEC     EVENT_KEY_B1_HELD_10_SEC
#define EVENT_KEY1_TRIPLE_PRESS    EVENT_KEY_B1_TRIPLE_PRESS
#define EVENT_KEY2_DOWN            EVENT_KEY_B2_DOWN
#define EVENT_KEY2_UP              EVENT_KEY_B2_UP
#define EVENT_KEY2_HELD            EVENT_KEY_B2_HELD
#define EVENT_KEY2_PRESS           EVENT_KEY_B2_PRESS
#define EVENT_KEY2_HELD_10_SEC     EVENT_KEY_B2_HELD_10_SEC
#define EVENT_KEY2_TRIPLE_PRESS    EVENT_KEY_B2_TRIPLE_PRESS
#define EVENT_KEY3_DOWN            EVENT_KEY_B3_DOWN
#define EVENT_KEY3_UP              EVENT_KEY_B3_UP
#define EVENT_KEY3_HELD            EVENT_KEY_B3_HELD
#define EVENT_KEY3_PRESS           EVENT_KEY_B3_PRESS
#define EVENT_KEY3_HELD_10_SEC     EVENT_KEY_B3_HELD_10_SEC
#define EVENT_KEY3_TRIPLE_PRESS    EVENT_KEY_B3_TRIPLE_PRESS

#define key1_pin_in()    SetPinIn(ZDP03A_KEY_1,TRUE) 
#define key2_pin_in()    SetPinIn(ZDP03A_KEY_2,TRUE)
#define key3_pin_in()    SetPinIn(ZDP03A_KEY_3,TRUE)

#define key_init() \
	key1_pin_in(); \
	key2_pin_in(); \
	key3_pin_in()
	
//-------------------switch----------------------
typedef struct {
	BYTE s1:1;
	BYTE s2:1;
	BYTE s3:1;
	BYTE learn:1;
	BYTE tmr_handle;
}switch_state_t;
static switch_state_t switch_state;

#define switch_init() relay_init(); \
                      led_init(); \
                      key_init(); \
                      switch_state.s1 = 0; \
                      switch_state.s2 = 0; \
                      switch_state.s3 = 0

static void s1_state_set(BYTE sta)
{
	if(sta) { relay1_on();  switch_state.s1 = 1; }
	else    { relay1_off(); switch_state.s1 = 0; }
}

static void s2_state_set(BYTE sta)
{
	if(sta) { relay2_on();  switch_state.s2 = 1; }
	else    { relay2_off(); switch_state.s2 = 0; }
}

static void s3_state_set(BYTE sta)
{
	if(sta) { relay3_on();  switch_state.s3 = 1; }
	else    { relay3_off(); switch_state.s3 = 0; }
}

#define s1_state_get() (BYTE)switch_state.s1
#define s2_state_get() (BYTE)switch_state.s2
#define s3_state_get() (BYTE)switch_state.s3

void cb_timer_5s(void);
PCB(cb_timer_5s)(void)
{
	ZW_DEBUG_APP_SEND_STR("\ncb_timer_5s()");
	
	switch_state.learn = 1;
	if(myNodeID) {
		ZW_DEBUG_APP_SEND_STR("LEARN_MODE_EXCLUSION");
		StartLearnModeNow(LEARN_MODE_EXCLUSION_NWE);
	}
	else{
		ZW_DEBUG_APP_SEND_STR("LEARN_MODE_INCLUSION");
		StartLearnModeNow(LEARN_MODE_INCLUSION);
	}
	ChangeState(STATE_APP_LEARN_MODE);
	led_nwk_on();
}


/**
 * @brief See description for function prototype in ZW_basis_api.h.
 */
BYTE
ApplicationInitHW(SW_WAKEUP bWakeupReason)
{
  wakeupReason = bWakeupReason;
  /* hardware initialization */
  ZDP03A_InitHW(ZCB_EventSchedulerEventAdd, NULL);
	
  /*SetPinIn(ZDP03A_KEY_1,TRUE);
  SetPinIn(ZDP03A_KEY_2,TRUE);

	// Learn mode indication
  SetPinOut(ZDP03A_LED_D1); 
  Led(ZDP03A_LED_D1,ON);
  SetPinOut(ZDP03A_LED_D2);
  Led(ZDP03A_LED_D2,ON);*/
	
	switch_init();
	
  Transport_OnApplicationInitHW(bWakeupReason);

  return(TRUE);
}


/**
 * @brief See description for function prototype in ZW_basis_api.h.
 */
BYTE
ApplicationInitSW(ZW_NVM_STATUS nvmStatus)
{
  /* Init state machine */
  currentState = STATE_APP_STARTUP;
  /* Do not reinitialize the UART if already initialized for ISD51 in ApplicationInitHW() 
	*/
#ifndef ZW_ISD51_DEBUG
  ZW_DEBUG_INIT(1152);
#endif

  ZW_DEBUG_APP_SEND_STR("\nApplicationInitSW()");
  ZW_DEBUG_APP_SEND_NUM(wakeupReason);
  ZW_DEBUG_APP_SEND_NUM(nvmStatus);
  ZW_DEBUG_APP_SEND_NL();

#ifdef WATCHDOG_ENABLED
  ZW_WatchDogEnable();
#endif 

  /* Signal that the sensor is awake */
  LoadConfiguration(nvmStatus);

  /* Setup AGI group lists */
  AGI_Init();
  AGI_LifeLineGroupSetup(agiTableLifeLine, (sizeof(agiTableLifeLine)/sizeof(CMD_CLASS_GRP)), GroupName, ENDPOINT_ROOT);

#ifdef BOOTLOADER_ENABLED
  /* Initialize OTA module */
    OtaInit( ZCB_OTAStart, NULL, ZCB_OTAFinish);
#endif

  /* Initialize Event Scheduler */
  EventSchedulerInit(AppStateManager);

  Transport_OnApplicationInitSW( &m_AppNIF, NULL);

  /* Init state machine */
  ZCB_EventSchedulerEventAdd((EVENT_WAKEUP)wakeupReason);

  return(TRUE);
}


/**
 * @brief See description for function prototype in ZW_basis_api.h.
 */
void
ApplicationTestPoll(void)
{
}


/**
 * @brief See description for function prototype in ZW_basis_api.h.
 */
void
ApplicationPoll(void)
{
#ifdef WATCHDOG_ENABLED
  ZW_WatchDogKick(); 
#endif

  TaskApplicationPoll();
}


/**
 * @brief See description for function prototype in ZW_TransportEndpoint.h.
 */
received_frame_status_t
Transport_ApplicationCommandHandlerEx(
  RECEIVE_OPTIONS_TYPE_EX *rxOpt,
  ZW_APPLICATION_TX_BUFFER *pCmd,
  BYTE cmdLength)
{
  received_frame_status_t frame_status = RECEIVED_FRAME_STATUS_NO_SUPPORT;
  ZW_DEBUG_APP_SEND_NL();
  ZW_DEBUG_APP_SEND_STR("\nTransport_ApplicationCommandHandlerEx()");
  ZW_DEBUG_APP_SEND_NUM(pCmd->ZW_Common.cmdClass);

  /* Call command class handlers */
  switch (pCmd->ZW_Common.cmdClass)
  {
    case COMMAND_CLASS_VERSION:
			ZW_DEBUG_APP_SEND_STR("->VERSION");
      frame_status = handleCommandClassVersion(rxOpt, pCmd, cmdLength);
      break;

#ifdef BOOTLOADER_ENABLED
    case COMMAND_CLASS_FIRMWARE_UPDATE_MD_V2:
			ZW_DEBUG_APP_SEND_STR("\n->MD_V2");
      frame_status = handleCommandClassFWUpdate(rxOpt, pCmd, cmdLength);
      break;
#endif

    case COMMAND_CLASS_ASSOCIATION_GRP_INFO:
			ZW_DEBUG_APP_SEND_STR("\n->INFO");
      frame_status = handleCommandClassAssociationGroupInfo( rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_ASSOCIATION:
			ZW_DEBUG_APP_SEND_STR("\n->ASSOCIATION");
			frame_status = handleCommandClassAssociation(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_POWERLEVEL:
			ZW_DEBUG_APP_SEND_STR("\n->POWERLEVEL");
      frame_status = handleCommandClassPowerLevel(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_MANUFACTURER_SPECIFIC:
			ZW_DEBUG_APP_SEND_STR("\n->SPECIFIC");
      frame_status = handleCommandClassManufacturerSpecific(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_ZWAVEPLUS_INFO:
			ZW_DEBUG_APP_SEND_STR("\n->ZWAVEPLUS_INFO");
      frame_status = handleCommandClassZWavePlusInfo(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_BASIC:
			ZW_DEBUG_APP_SEND_STR("\n->BASIC");
      frame_status = handleCommandClassBasic(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_SWITCH_BINARY:
			ZW_DEBUG_APP_SEND_STR("\n->BINARY");
      frame_status = handleCommandClassBinarySwitch(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_SWITCH_ALL:
			ZW_DEBUG_APP_SEND_STR("\n->SWITCH_ALL");
      frame_status = handleCommandClassSwitchAll(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_SUPERVISION:
			ZW_DEBUG_APP_SEND_STR("\n->SUPERVISION");
      frame_status = handleCommandClassSupervision(rxOpt, pCmd, cmdLength);
      break;

    case COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2:
			ZW_DEBUG_APP_SEND_STR("\n->ASSOCIATION_V2");
      frame_status = handleCommandClassMultiChannelAssociation(rxOpt, pCmd, cmdLength);
      break;
  }
  return frame_status;
}


/**
 * @brief See description for function prototype in CommandClassVersion.h.
 */
BYTE
handleCommandClassVersionAppl( BYTE cmdClass )
{
  BYTE commandClassVersion = UNKNOWN_VERSION;
  ZW_DEBUG_APP_SEND_STR("\nhandleCommandClassVersionAppl()");
  switch (cmdClass)
  {
    case COMMAND_CLASS_VERSION:
			ZW_DEBUG_APP_SEND_STR("\n->VERSION");
      commandClassVersion = CommandClassVersionVersionGet();
      break;

#ifdef BOOTLOADER_ENABLED
    case COMMAND_CLASS_FIRMWARE_UPDATE_MD:
			ZW_DEBUG_APP_SEND_STR("\n->MD");
      commandClassVersion = CommandClassFirmwareUpdateMdVersionGet();
      break;
#endif

    case COMMAND_CLASS_POWERLEVEL:
			ZW_DEBUG_APP_SEND_STR("\n->POWERLEVEL");
      commandClassVersion = CommandClassPowerLevelVersionGet();
      break;

    case COMMAND_CLASS_MANUFACTURER_SPECIFIC:
			ZW_DEBUG_APP_SEND_STR("\n->SPECIFIC");
      commandClassVersion = CommandClassManufacturerVersionGet();
      break;

    case COMMAND_CLASS_ASSOCIATION:
			ZW_DEBUG_APP_SEND_STR("\n->ASSOCIATION");
     commandClassVersion = CommandClassAssociationVersionGet();
      break;

    case COMMAND_CLASS_ASSOCIATION_GRP_INFO:
			ZW_DEBUG_APP_SEND_STR("\n->INFO");
     commandClassVersion = CommandClassAssociationGroupInfoVersionGet();
      break;

    case COMMAND_CLASS_DEVICE_RESET_LOCALLY:
			ZW_DEBUG_APP_SEND_STR("\n->LOCALLY");
     commandClassVersion = CommandClassDeviceResetLocallyVersionGet();
      break;

    case COMMAND_CLASS_ZWAVEPLUS_INFO:
			ZW_DEBUG_APP_SEND_STR("\n->ZWAVEPLUS_INFO");
     commandClassVersion = CommandClassZWavePlusVersion();
      break;
    case COMMAND_CLASS_BASIC:
			ZW_DEBUG_APP_SEND_STR("\n->BASIC");
     commandClassVersion =  CommandClassBasicVersionGet();
      break;
    case COMMAND_CLASS_SWITCH_BINARY:
			ZW_DEBUG_APP_SEND_STR("\n->BINARY");
     commandClassVersion = CommandClassBinarySwitchVersionGet();
      break;

    case COMMAND_CLASS_SWITCH_ALL:
			ZW_DEBUG_APP_SEND_STR("\n->SWITCH_ALL");
     commandClassVersion = CommandClassSwitchAllVersionGet();
      break;

    case COMMAND_CLASS_MULTI_CHANNEL_ASSOCIATION_V2:
			ZW_DEBUG_APP_SEND_STR("\n->ASSOCIATION_V2");
      commandClassVersion = CmdClassMultiChannelAssociationVersion();
      break;

    case COMMAND_CLASS_SUPERVISION:
			ZW_DEBUG_APP_SEND_STR("\n->SUPERVISION");
      commandClassVersion = CommandClassSupervisionVersionGet();
      break;

    default:
			ZW_DEBUG_APP_SEND_STR("\n->default");
     commandClassVersion = ZW_Transport_CommandClassVersionGet(cmdClass);
  }
  return commandClassVersion;
}


/**
 * @brief See description for function prototype in ZW_slave_api.h.
 */
void
ApplicationSlaveUpdate(
  BYTE bStatus,
  BYTE bNodeID,
  BYTE* pCmd,
  BYTE bLen)
{
  UNUSED(bStatus);
  UNUSED(bNodeID);
  UNUSED(pCmd);
  UNUSED(bLen);
}


/**
 * @brief See description for function prototype in slave_learn.h.
 */
void
LearnCompleted(BYTE bNodeID)
{
  ZW_DEBUG_APP_SEND_NL();
  ZW_DEBUG_APP_SEND_STR("LearnCompleted()");
  ZW_DEBUG_APP_SEND_NUM(bNodeID);
  /*If bNodeID= 0xff.. learn mode failed*/
  if(bNodeID != NODE_BROADCAST)
  {
    /*Success*/
    myNodeID = bNodeID;
    if (0 == myNodeID)
    {
      /*Clear association*/
      AssociationInit(TRUE);
      SetDefaultConfiguration();
    }
  }
  ZCB_EventSchedulerEventAdd((EVENT_APP) EVENT_SYSTEM_LEARNMODE_FINISH);
  Transport_OnLearnCompleted(bNodeID);
}


/**
 * @brief Returns the Z-Wave node ID.
 * @return BYTE NodeID
 */
BYTE
GetMyNodeID(void)
{
	return myNodeID;
}


/**
 * @brief Returns the current state of the application state machine.
 * @return Current state
 */
STATE_APP
GetAppState(void)
{
  return currentState;
}

/**
 * @brief The core state machine of this sample application.
 * @param event The event that triggered the call of AppStateManager.
 */
void
AppStateManager(EVENT_APP event)
{
  ZW_DEBUG_APP_SEND_NL();
  ZW_DEBUG_APP_SEND_STR("AppStateManager()");
  ZW_DEBUG_APP_SEND_NUM(event);
  ZW_DEBUG_APP_SEND_STR("s");
  ZW_DEBUG_APP_SEND_NUM(currentState);

  if(EVENT_SYSTEM_WATCHDOG_RESET == event)
  {
    /*Force state change to activate watchdog-reset without taking care of current
      state.*/
    ChangeState(STATE_APP_WATCHDOG_RESET);
  }

  switch(currentState)
  {
    case STATE_APP_STARTUP:
			ZW_DEBUG_APP_SEND_STR("\nSTATE_APP_STARTUP");
      ChangeState(STATE_APP_IDLE);
      break;

    case STATE_APP_IDLE:
			ZW_DEBUG_APP_SEND_STR("\nSTATE_APP_IDLE");
		  if(event == EVENT_SYSTEM_LEARNMODE_START) {
					if(myNodeID) {
						ZW_DEBUG_APP_SEND_STR("LEARN_MODE_EXCLUSION");
						StartLearnModeNow(LEARN_MODE_EXCLUSION_NWE);
					}
					else{
						ZW_DEBUG_APP_SEND_STR("LEARN_MODE_INCLUSION");
						StartLearnModeNow(LEARN_MODE_INCLUSION);
					}
					ChangeState(STATE_APP_LEARN_MODE);
					led_nwk_on();
					return;
      }
			else if(event == EVENT_KEY1_DOWN || 
							event == EVENT_KEY2_DOWN || 
						  event == EVENT_KEY3_DOWN) { 
				ZW_DEBUG_APP_SEND_STR("\nEVENT_KEY_DOWN"); 
				switch_state.tmr_handle = ZW_TIMER_START(cb_timer_5s,500,1); //500*10=5s
				switch_state.learn = 0;
			}
			else if(event == EVENT_KEY1_UP) { 
				ZW_DEBUG_APP_SEND_STR("\nEVENT_KEY1_UP"); 
				if(switch_state.learn == 0) {
					ZW_TIMER_CANCEL(switch_state.tmr_handle);
					s1_state_set(!switch_state.s1);
				}
			}
			else if(event == EVENT_KEY2_UP) { 
				ZW_DEBUG_APP_SEND_STR("\nEVENT_KEY2_UP"); 
				if(switch_state.learn == 0) {
					ZW_TIMER_CANCEL(switch_state.tmr_handle);
					s2_state_set(!switch_state.s2);
				}
			}
			else if(event == EVENT_KEY3_UP) { 
				ZW_DEBUG_APP_SEND_STR("\nEVENT_KEY3_UP"); 
				if(switch_state.learn == 0) {
					ZW_TIMER_CANCEL(switch_state.tmr_handle);
					s3_state_set(!switch_state.s3);
				}
			}
      break;

    case STATE_APP_LEARN_MODE:
			ZW_DEBUG_APP_SEND_STR("\nSTATE_APP_LEARN_MODE");
		  if(event == EVENT_SYSTEM_LEARNMODE_END) {
				ZW_DEBUG_APP_SEND_STR("\nEVENT_SYSTEM_LEARNMODE_END");
				StartLearnModeNow(LEARN_MODE_DISABLE);
        ChangeState(STATE_APP_IDLE);
				led_nwk_off();
			}
      else if(event == EVENT_SYSTEM_LEARNMODE_FINISH) {
        ZW_DEBUG_APP_SEND_STR("\nEVENT_SYSTEM_LEARNMODE_FINISH");
        ChangeState(STATE_APP_IDLE);
				led_nwk_off();
      }
      break;

    case STATE_APP_WATCHDOG_RESET:
      if(EVENT_APP_REFRESH_MMI == event){}
      ZW_DEBUG_APP_SEND_STR("\nSTATE_APP_WATCHDOG_RESET");
      ZW_WatchDogEnable(); /*reset asic*/
      for (;;) {}
      break;

    case STATE_APP_OTA:
			ZW_DEBUG_APP_SEND_STR("\nSTATE_APP_OTA");
      if(EVENT_APP_REFRESH_MMI == event){}
      break;

#ifdef BOOTLOADER_ENABLED
    case STATE_APP_OTA_HOST:
		  ZW_DEBUG_APP_SEND_STR("\nSTATE_APP_OTA_HOST");
      if(EVENT_APP_REFRESH_MMI == event) { }
      if(EVENT_APP_OTA_HOST_WRITE_DONE == event)
      {
        ZW_DEBUG_SEND_STR("\nEVENT_APP_OTA_HOST_WRITE_DONE");
        OtaHostFWU_WriteFinish();
      }
      if(EVENT_APP_OTA_HOST_STATUS == event)
      {
        ZW_DEBUG_SEND_STR("\nEVENT_APP_OTA_HOST_STATUS");
        userReboot = FALSE;
        OtaHostFWU_Status(userReboot, TRUE);
      }
      break;
#endif
  }
}


/**
 * @brief Sets the current state to a new, given state.
 * @param newState New state.
 */
static void
ChangeState(STATE_APP newState)
{
  ZW_DEBUG_APP_SEND_STR("\nChangeState(");
  ZW_DEBUG_APP_SEND_NUM(newState);
	ZW_DEBUG_APP_SEND_STR(")");

  currentState = newState;
  ZCB_EventSchedulerEventAdd(EVENT_APP_REFRESH_MMI);
}

/**
 * @brief Transmission callback for Device Reset Locally call.
 * @param pTransmissionResult Result of each transmission.
 */
PCB(ZCB_DeviceResetLocallyDone)(TRANSMISSION_RESULT * pTransmissionResult)
{
	ZW_DEBUG_APP_SEND_STR("\nZCB_DeviceResetLocallyDone()");
	
  if (TRANSMISSION_RESULT_FINISHED == pTransmissionResult->isFinished)
  {
    ZW_DEBUG_APP_SEND_NL();
    ZW_DEBUG_APP_SEND_STR("DRLD");

    ZCB_EventSchedulerEventAdd((EVENT_APP) EVENT_SYSTEM_WATCHDOG_RESET);
  }
}

#ifdef BOOTLOADER_ENABLED
/**
 * @brief Called when OTA firmware upgrade is finished. Reboots node to cleanup
 * and starts on new FW.
 * @param OTA_STATUS otaStatus
 */
PCB(ZCB_OTAFinish)(OTA_STATUS otaStatus)
{
	ZW_DEBUG_APP_SEND_STR("\nZCB_OTAFinish()");
	
  UNUSED(otaStatus);
  if (STATE_APP_OTA_HOST == GetAppState())
  {
    ChangeState(STATE_APP_IDLE);
    if (userReboot)
    {
      userReboot = FALSE;
      return;
    }
  }
  if (OTA_STATUS_DONE == otaStatus)
  {
  /*Just reboot node to cleanup and start on new FW.*/
    ZW_WatchDogEnable(); /*reset asic*/
    while(1);
  }
}


/**
 * @brief Function pointer for KEIL.
 */
code const BOOL (code * ZCB_OTAStart_p)(void) = &ZCB_OTAStart;
/**
 * @brief Called before OTA firmware upgrade starts.
 * @details Checks whether the application is ready for a firmware upgrade.
 * @return FALSE if OTA should be rejected, otherwise TRUE.
 */
BOOL
ZCB_OTAStart(void)
{
  BOOL  status = FALSE;
	
	ZW_DEBUG_APP_SEND_STR("\nZCB_OTAStart()");
	
  if (STATE_APP_IDLE == GetAppState())
  {
    ZCB_EventSchedulerEventAdd((EVENT_APP) EVENT_SYSTEM_OTA_START);
    status = TRUE;
  }
  return status;
}


/**
 * @brief Called OTA firmware upgrade want to write an image
 * @param BYTE *pData pointer to the image data to write.
 * @param BYTE len length of the image data
 */
PCB(ZCB_OTAWrite)(BYTE *pData, BYTE len)
{
  static WORD adr = 0;
	
	ZW_DEBUG_APP_SEND_STR("\nZCB_OTAWrite()");
	
  UNUSED(pData);
  if (STATE_APP_IDLE == GetAppState())
  {
    ZW_DEBUG_APP_SEND_NL();
    ZW_DEBUG_APP_SEND_STR("STATE_APP_OTA_HOST");

    ChangeState(STATE_APP_OTA_HOST);
  }
  if (len)
  {
    ZW_DEBUG_APP_SEND_STR("W ADR: 0x");
    ZW_DEBUG_APP_SEND_NUM((BYTE)(adr>>8) & 0xFF);
    ZW_DEBUG_APP_SEND_NUM((BYTE)(adr & 0x00FF));
    ZW_DEBUG_APP_SEND_STR(" L: 0x");
    ZW_DEBUG_APP_SEND_NUM(len);
    ZW_DEBUG_APP_SEND_BYTE(':');

    adr += len;
    while(--len)
    {
      ZW_DEBUG_APP_SEND_NUM(*pData++);
    }
    ZW_DEBUG_APP_SEND_NL();
    // set event write finish
    ZCB_EventSchedulerEventAdd(EVENT_APP_OTA_HOST_WRITE_DONE);

  }
  else
  {
    // set event ota host status
    ZCB_EventSchedulerEventAdd(EVENT_APP_OTA_HOST_STATUS);
  }

}
#endif


/**
 * @brief Handler for basic set.
 *
 * Handles received basic set commands.
 *
 * @param val Parameter dependent of the application device class.
 */
void
handleBasicSetCommand(BYTE val, BYTE endpoint )
{
  CommandClassBinarySwitchSupportSet(val, endpoint);
}


/**
 * @brief Handler for basic get. Handles received basic get commands.
 */
BYTE
getAppBasicReport( BYTE endpoint )
{
  return handleAppltBinarySwitchGet(endpoint);
}


/**
 * @brief Report the target value
 * @return target value.
 */
BYTE
getAppBasicReportTarget( BYTE endpoint )
{
  return handleAppltBinarySwitchGet(endpoint);
}


/**
 * @brief Report transition duration time.
 * @return duration time.
 */
BYTE
getAppBasicReportDuration( BYTE endpoint )
{
  UNUSED(endpoint);
  return 0;
}

/**
 * @brief See description for function prototype in CommandClassSwitchAll.h.
 */ 
void handleSwitchAll(CMD_CLASS_SWITCHALL_SET val, BYTE endpoint)
{
	UNUSED(endpoint);
	s1_state_set(val);
	s2_state_set(val);
	s3_state_set(val);
}


/**
 * @brief See description for function prototype in CommandClassVersion.h.
 */
BYTE
handleNbrFirmwareVersions(void)
{
  return 1; /*CHANGE THIS - firmware 0 version*/
}


/**
 * @brief See description for function prototype in CommandClassVersion.h.
 */
void
handleGetFirmwareVersion(
        BYTE bFirmwareNumber,
        VG_VERSION_REPORT_V2_VG* pVariantgroup)
{
  /*firmware 0 version and sub version*/
	ZW_DEBUG_APP_SEND_STR("\nhandleGetFirmwareVersion()");
	
  if(bFirmwareNumber == 0)
  {
    pVariantgroup->firmwareVersion = APP_VERSION;
    pVariantgroup->firmwareSubVersion = APP_REVISION;
  }
  else
  {
    /*Just set it to 0 if firmware n is not present*/
    pVariantgroup->firmwareVersion = 0;
    pVariantgroup->firmwareSubVersion = 0;
  }
}


/**
 * Function return firmware Id of target n (0 => is device FW ID)
 * n read version of firmware number n (0,1..N-1)
 * @return firmaware ID.
 */
WORD
handleFirmWareIdGet( BYTE n)
{
	ZW_DEBUG_APP_SEND_STR("\nhandleFirmWareIdGet()");
	
  if(n == 0)
  {
    return APP_FIRMWARE_ID;
  }
  else if (n == 1)
  {
    return 0x1234;
  }
  return 0;
}


/**
 * @brief See description for function prototype in CommandClassBianrySwitch.h
 */
BYTE
handleAppltBinarySwitchGet(BYTE endpoint)
{
	if(endpoint == 0) {
		return s1_state_get();
	}
	else if(endpoint == 1) {
		return s2_state_get();
	}
	else if(endpoint == 2) {
		return s3_state_get();
	}
	return 0;
}


/**
 * @brief See description for function prototype in CommandClassBianrySwitch.h
 */
void
handleApplBinarySwitchSet(CMD_CLASS_BIN_SW_VAL val, BYTE endpoint )
{
	if(endpoint == 0) {
		s1_state_set(val);
	}
	else if(endpoint == 1) {
		s2_state_set(val);
	}
	else if(endpoint == 2) {
		s3_state_set(val);
	}
}


/**
 * @brief Sets the configuration to default values and saves it to EEPROM.
 */
void
SetDefaultConfiguration(void)
{
	ZW_DEBUG_APP_SEND_STR("\nSetDefaultConfiguration()");
	
  onOffState = 0;
  /* Mark stored configuration as OK */
  MemoryPutByte((WORD)&OnOffState_far, onOffState);
  MemoryPutByte((WORD)&EEOFFSET_MAGIC_far, APPL_MAGIC_VALUE);
  MemoryPutByte( (WORD)&EEOFFSET_SWITCH_ALL_MODE_far[0],
    SWITCH_ALL_REPORT_INCLUDED_IN_THE_ALL_ON_ALL_OFF_FUNCTIONALITY);
}


/**
 * @brief Loads configuration from EEPROM. If no configuration is stored,
 * default values are loaded and then saved to EEPROM.
 */
void
LoadConfiguration(ZW_NVM_STATUS nvmStatus)
{
  uint8_t magicValue;

	ZW_DEBUG_APP_SEND_STR("\nLoadConfiguration()");
	
  /* Get this sensors identification on the network */
  MemoryGetID( NULL, &myNodeID);
  ManufacturerSpecificDeviceIDInit();
#ifdef BOOTLOADER_ENABLED
  NvmInit(nvmStatus);
#else
  UNUSED(nvmStatus);
#endif
  /* Check to see, if any valid configuration is stored in the EEPROM */
  magicValue = MemoryGetByte((WORD)&EEOFFSET_MAGIC_far);
  ZW_DEBUG_APP_SEND_NL();
  ZW_DEBUG_APP_SEND_BYTE('M');
  ZW_DEBUG_APP_SEND_NUM(magicValue);
  if (APPL_MAGIC_VALUE == magicValue)
  {
    loadStatusPowerLevel(NULL,NULL);
    /* There is a configuration stored, so load it */
    onOffState = MemoryGetByte((WORD)&OnOffState_far);
    ZW_DEBUG_APP_SEND_NL();
    ZW_DEBUG_APP_SEND_BYTE('C');
    ZW_DEBUG_APP_SEND_BYTE('l');

    AssociationInit(FALSE);
  }
  else
  {
    ZW_MEM_PUT_BYTE((WORD)&EEOFFS_SECURITY_RESERVED.EEOFFS_MAGIC_BYTE_field, EEPROM_MAGIC_BYTE_VALUE);
    /* Initialize transport layer NVM */
    Transport_SetDefault();
    /* Reset protocol */
    ZW_SetDefault();

    /* Apparently there is no valid configuration in EEPROM, so load */
    /* default values and save them to EEPROM. */
    SetDefaultConfiguration();

    /*Clear association*/
    AssociationInit(TRUE);

    loadInitStatusPowerLevel(NULL, NULL);
  }
  RefreshMMI();
}


/**
 * @brief Toggles LED state variable and refreshes MMI.
 */
void
ToggleLed(void)
{
	ZW_DEBUG_APP_SEND_STR("\nToggleLed()");
  /*if (onOffState)
  {
    onOffState = 0;
  }
  else
  {
    onOffState = 0xff;
  }
  RefreshMMI();
  MemoryPutByte((WORD)&OnOffState_far, onOffState);*/
}


/**
 * @brief Refreshes MMI.
 */
void
RefreshMMI(void)
{
	ZW_DEBUG_APP_SEND_STR("\nRefreshMMI()");
	
  /*if (CMD_CLASS_BIN_OFF == onOffState)
  {
    Led(ZDP03A_LED_D2,OFF);
  }
  else if (CMD_CLASS_BIN_ON == onOffState)
  {
    Led(ZDP03A_LED_D2,ON);
  }*/
}


/*
 * @brief Called when protocol needs to inform Application about a Security Event.
 * @details If the app does not need/want the Security Event the handling can be left empty.
 *
 *    Event E_APPLICATION_SECURITY_EVENT_S2_INCLUSION_REQUEST_DSK_CSA
 *          If CSA is needed, the app must do the following when this event occures:
 *             1. Obtain user input with first 4 bytes of the S2 including node DSK
 *             2. Store the user input in a response variable of type s_SecurityS2InclusionCSAPublicDSK_t.
 *             3. Call ZW_SetSecurityS2InclusionPublicDSK_CSA(response)
 *
 */
void
ApplicationSecurityEvent(
  s_application_security_event_data_t *securityEvent)
{
	ZW_DEBUG_APP_SEND_STR("\nApplicationSecurityEvent()");
  switch (securityEvent->event)
  {
#ifdef APP_SUPPORTS_CLIENT_SIDE_AUTHENTICATION
    case E_APPLICATION_SECURITY_EVENT_S2_INCLUSION_REQUEST_DSK_CSA:
      {
				ZW_DEBUG_APP_SEND_STR("\nE_APPLICATION_SECURITY_EVENT_S2_INCLUSION_REQUEST_DSK_CSA");
        ZW_SetSecurityS2InclusionPublicDSK_CSA(&sCSAResponse);
      }
      break;
#endif /* APP_SUPPORTS_CLIENT_SIDE_AUTHENTICATION */

    default:
      break;
  }
}


/**
* Set up security keys to request when joining a network.
* These are taken from the config_app.h header file.
*/
BYTE ApplicationSecureKeysRequested(void)
{
	ZW_DEBUG_APP_SEND_STR("\nApplicationSecureKeysRequested()");
  return REQUESTED_SECURITY_KEYS;
}

/**
* Set up security S2 inclusion authentication to request when joining a network.
* These are taken from the config_app.h header file.
*/
BYTE ApplicationSecureAuthenticationRequested(void)
{
	ZW_DEBUG_APP_SEND_STR("\nApplicationSecureAuthenticationRequested()");
  return REQUESTED_SECURITY_AUTHENTICATION;
}

