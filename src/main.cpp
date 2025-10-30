/*
 * TestSimpleLMICWrapper
 *
 * leuville-lmic-easy example project
 * - subclass of LMICWrapper
 * - no protocol buffer serialization
 * - no standby
 */

#include <Arduino.h>
#include <arduino_lmic_hal_boards.h>
#include <Wire.h>

// leuville-arduino-easy-lmic
#include <LMICWrapper.h>
// leuville-arduino-utilities
#include <misc-util.h>
#include <StatusLed.h>

// LoRaWAN configuration: see OTAAId in LMICWrapper.h
// file not included in this project
#include "lora-common-defs.h"

#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
USBPrinter<Serial_> console(LMIC_PRINTF_TO);
#endif

namespace lora = leuville::lora;

using namespace lora;

/* ---------------------------------------------------------------------------------------
 * Application classes
 * ---------------------------------------------------------------------------------------
 */

/*
 * Simple LoraWan endnode 
 */
class EndNode : public LMICWrapper {

	/*
	 * Jobs for event callbacks
	 */
	osjob_t _joinJob, _txCompleteJob, _msgJob;

	int _count = 0;	// message counter

public:

	EndNode(const lmic_pinmap &pinmap): EndNode(&pinmap) {}

	EndNode(const lmic_pinmap *pinmap): LMICWrapper(pinmap, LMICWrapper::KEEP_RECENT) {}

	/*
	 * delegates begin() to each sub-component and joins
	 */
	virtual void begin(const OTAAId& id, u4_t network, bool adr = true) override {
		Wire.begin();
		LMICWrapper::begin(id, network, adr);

		startJoining(); // first message sent after JOIN, in joinJob() callback

		setNoPowerInfo();
	}

	/*
	 * Configure LoRaWan network (channels, power, adr, ...)
	 * configureNetwork() not included in this project
	 */
	virtual void initLMIC(u4_t network = 0, bool adr = true) override {
		LMICWrapper::initLMIC(network, adr);
		configureNetwork(static_cast<Network>(network), adr);
	}

	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(bool ok) override {
		if (ok) {
			setCallback(_joinJob);
		} else {
			unsetCallback(_joinJob);
			unsetCallback(_msgJob);
			unsetCallback(_txCompleteJob);
		}
	}

	#if defined(LMIC_ENABLE_DeviceTimeReq)
	/*
	 * Current time should be modified using RTCZero or ISRTimer
	 */
	virtual void updateSystemTime(uint32_t newTime) override {
    	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("network time = ",	newTime); 
		console.println("----------------------------------------------------------");
    	#endif
	}
	#endif

	/*
	 * Handle LMIC callbacks by calling registered callback
	 */
	virtual void completeJob(osjob_t* job) override {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1
		console.println("----------------------------------------------------------");
		console.println("completeJob");
		console.println("----------------------------------------------------------");
		#endif
		if (job == &_msgJob)
			msgJob();
		else if (job == &_txCompleteJob)
			txCompleteJob();
		else if (job == &_joinJob)
			joinJob();
	}

	/*
	 * Send message
	 */
	virtual void msgJob() {
		const char* format = "MSG %d";
		char msg[80];
		sprintf(msg, format, _count++);
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("msgJob -> ", msg);
		console.println("----------------------------------------------------------");
		#endif
		send(msg, (_count % 2) == 0);
	}

	/*
	 * Register callback to send next message
	 */
	virtual void txCompleteJob() {
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1
		console.println("----------------------------------------------------------");
		console.println("txCompleteJob, FIFO size: ", _messages.size());
		console.println("----------------------------------------------------------");
		#endif
		setCallback(_msgJob, DEVICE_MESSAGE_DELAY * 60 * 1000); 
	}

	/*
	 * Display Lora network keys
	 * postJoinSetup() not included in this project
	 */
	virtual void joinJob() {
		LoRaWanSessionKeys keys = getSessionKeys();
		// https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments.html
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
		console.println("----------------------------------------------------------");
		console.println("netId: ", keys._netId, HEX);
		console.println("devAddr: ", keys._devAddr, HEX);
		console.print("nwkSKey: "); console.printHex(keys._nwkSKey, arrayCapacity(keys._nwkSKey));
		console.print("appSKey: "); console.printHex(keys._appSKey, arrayCapacity(keys._appSKey));
		console.println("----------------------------------------------------------");
		#endif
		postJoinSetup(keys._netId);
		setCallback(_msgJob, 2000); // send first message with 2s delay
	}

	/*
	 * Build and send Uplink message
	 */
	void send(const char* message, bool ack = false) {
		UpstreamMessage payload((uint8_t*)message, strlen(message)+1, ack);
		LMICWrapper::send(payload);
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1
		console.println("----------------------------------------------------------");
		console.print("send ", message);
		console.println(", FIFO size: ", _messages.size());
		console.println("----------------------------------------------------------");
		#endif
	}

	/*
	 * LMIC callback called on TX_COMPLETE event
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message) override {
		setCallback(_txCompleteJob);
		#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 1
		console.println("----------------------------------------------------------");
		console.print("isTxCompleted "); 
		console.print(" / len: ", message._len);
		console.print(" / ackRequest: ", message._ackRequested);
		console.print(" / txrxFlags: ", message._txrxFlags);
		console.println(" / lmicError: ", (int32_t)message._lmicTxError);
		console.println("----------------------------------------------------------");
		#endif
		return LMICWrapper::isTxCompleted(message);
	}

};

/* ---------------------------------------------------------------------------------------
 * GLOBAL OBJECTS
 * ---------------------------------------------------------------------------------------
 */
BlinkingLed statusLed(LED_BUILTIN, 500);

#if defined(LMIC_PINS)
EndNode 	endnode(LMIC_PINS); 
#else
EndNode 	endnode(Arduino_LMIC::GetPinmap_ThisBoard());
#endif

/* ---------------------------------------------------------------------------------------
 * ARDUINO FUNCTIONS
 * ---------------------------------------------------------------------------------------
 */
void setup()
{
	#if defined(LMIC_DEBUG_LEVEL) && LMIC_DEBUG_LEVEL > 0
	console.begin(115200);
	#endif
	
	statusLed.begin();
	statusLed.on();

	endnode.begin(lora::id[DEVICE_CONFIG], DEVICE_NETWORK, ADR::ON);

	delay(5000);
	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		statusLed.off();
	} else {
		statusLed.blink();
	}
}
