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
#include <lora-common-defs.h>

USBPrinter<Serial_> console(STDOUT);

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

	EndNode(const lmic_pinmap *pinmap): LMICWrapper(pinmap) {}

	/*
	 * delegates begin() to each sub-component and send a first message
	 */
	virtual void begin(const OTAAId& id, u4_t network, bool adr = true) override {
		Wire.begin();
		LMICWrapper::begin(id, network, adr);
		setCallback(_msgJob, 2000); // send first message with 2s delay
	}

	/*
	 * Configure LoRaWan network (channels, power, adr, ...)
	 * configureNetwork() not included in this project
	 */
	virtual void initLMIC(u4_t network = 0, bool adr = true) override {
		configureNetwork(static_cast<Network>(network), adr);
	}

	/*
	 * Job done on join/unjoin
	 * see completeJob()
	 */
	virtual void joined(bool ok) override {
		if (ok) {
			setCallback(_joinJob);
		}
	}

	virtual void updateSystemTime(uint32_t newTime) override {
		console.println("network time: ", newTime);
	}

	/*
	 * Handle LMIC callbacks by calling registered callback
	 */
	virtual void completeJob(osjob_t* job) override {
		console.println("completeJob()");
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
		send(msg, (_count % 2) == 0);
	}

	/*
	 * Register callback to send next message
	 */
	virtual void txCompleteJob() {
		console.println("txCompleteJob, FIFO size: ", _messages.size());
		setCallback(_msgJob, DEVICE_MESSAGE_DELAY * 1000); 
	}

	/*
	 * Display Lora network keys
	 * postJoinSetup() not included in this project
	 */
	virtual void joinJob() {
		LoRaWanSessionKeys keys = getSessionKeys();
		// https://www.thethingsnetwork.org/docs/lorawan/prefix-assignments.html
		console.println("netId: ", keys._netId, HEX);
		console.println("devAddr: ", keys._devAddr, HEX);
		console.print("nwkSKey: "); console.printHex(keys._nwkSKey, arrayCapacity(keys._nwkSKey));
		console.print("appSKey: "); console.printHex(keys._appSKey, arrayCapacity(keys._appSKey));
		postJoinSetup(keys._netId);
	}

	/*
	 * Build and send Uplink message
	 */
	void send(const char* message, bool ack = false) {
		UpstreamMessage payload((uint8_t*)message, strlen(message)+1, ack);
		console.print("send ", message);
		LMICWrapper::send(payload);
		console.println(", FIFO size: ", _messages.size());
	}

	/*
	 * LMIC callback called on TX_COMPLETE event
	 */
	virtual bool isTxCompleted(const UpstreamMessage & message, bool ack) override {
		setCallback(_txCompleteJob);
		console.print("isTxCompleted ", (char*)message._buf); 
		console.println(" / ack: ", ack);
		return LMICWrapper::isTxCompleted(message, ack);
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
	console.begin(115200);

	statusLed.begin();
	statusLed.on();

	while(! console.available()) { delay(10); }

	endnode.begin(lora::id[DEVICE_CONFIG], DEVICE_NETWORK, ADR::ON);

	delay(5000);
	statusLed.off();
}

void loop()
{
	endnode.runLoopOnce();
	if (endnode.isReadyForStandby()) {
		statusLed.off();
	}
	else {
		statusLed.blink();
	}
}
