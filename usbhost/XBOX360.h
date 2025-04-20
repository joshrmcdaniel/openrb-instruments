#pragma once
#define _xbox360_h_

enum ButtonBits {
    btnHatRight      = 0x8000,
    btnHatLeft       = 0x4000,
    btnBack          = 0x2000,
    btnStart         = 0x1000,
    btnDigiRight     = 0x0800,
    btnDigiLeft      = 0x0400,
    btnDigiDown      = 0x0200,
    btnDigiUp        = 0x0100,
    btnY             = 0x0080,
    btnX             = 0x0040,
    btnB             = 0x0020,
    btnA             = 0x0010,
    btnReserved1     = 0x0008,  // Unused?
    btnXbox          = 0x0004,
    btnShoulderRight = 0x0002,
    btnShoulderLeft  = 0x0001
};
#ifndef _usb_h_
#include "Usb.h"
#endif
#ifndef _xboxenums_h
#include "xboxEnums.h"
#endif
// PID and VID of the different versions of the controller - see:
// https://github.com/torvalds/linux/blob/c2da8b3f914f83fb9089d26a692eb8f22146ddb9/drivers/input/joystick/xpad.c#L129
#define XBOX_360_PID1 0x028E  // Microsoft X-Box 360 pad
#define XBOX_360_PID2 0x028F  // Microsoft X-Box 360 pad v2
#define XBOX_360_PID3 0x0291  // Microsoft X-Box 360 Wireless Receiver
#define XBOX_360_PID4 0x4726  // Mad Catz Xbox 360 Controller
#define XBOX_360_PID5 0x0130  // Ion Drum Rocker
#define XBOX_360_PID6 0x0002  // Harmonix Guitar for Xbox 360
#define XBOX_360_PID7 0xF901  // GameStop Xbox 360 Controller
/* Names we give to the 3 XboxONE pipes */
#define XBOX_360_CONTROL_PIPE 0
#define XBOX_360_OUTPUT_PIPE 1
#define XBOX_360_INPUT_PIPE 2

#define XBOX_360_MAX_ENDPOINTS 3

class XBOX360 : public USBDeviceConfig, public UsbConfigXtracter {
public:
    /**
     * Constructor for the XB360AUTH class.
     * @param  pUsb   Pointer to USB class instance.
     */
    XBOX360(USB *pUsb, void (*data_cb)(const uint8_t *data, const uint8_t &ndata));

    /** @name USBDeviceConfig implementation */
    /**
     * Initialize the Xbox Controller.
     * @param  parent   Hub number.
     * @param  port     Port number on the hub.
     * @param  lowspeed Speed of the device.
     * @return          0 on success.
     */
    virtual uint8_t Init(uint8_t parent, uint8_t port, bool lowspeed);
    /**
     * Release the USB device.
     * @return 0 on success.
     */
    virtual uint8_t Release();
    /**
     * Poll the USB Input endpoins and run the state machines.
     * @return 0 on success.
     */
    virtual uint8_t Poll();

    /**
     * Get the device address.
     * @return The device address.
     */
    virtual uint8_t GetAddress() {
        return bAddress;
    };

    /**
     * Used to check if the controller has been initialized.
     * @return True if it's ready.
     */
    virtual bool isReady() {
        return bPollEnable;
    };

    /**
     * Read the poll interval taken from the endpoint descriptors.
     * @return The poll interval in ms.
     */
    uint8_t readPollInterval() {
        return pollInterval;
    };

    /**
     * Used by the USB core to check what this driver support.
     * @param  vid The device's VID.
     * @param  pid The device's PID.
     * @return     Returns true if the device's VID and PID matches this driver.
     */
    virtual bool VIDPIDOK(uint16_t vid, uint16_t pid) {
        return ((vid == XBOX_VID1 || vid == XBOX_VID2 || vid == XBOX_VID3 || vid == XBOX_VID4 ||
                 vid == XBOX_VID5 || vid == XBOX_VID6 || vid == XBOX_VID7) &&
                (pid == XBOX_360_PID1 || pid == XBOX_360_PID2 || pid == XBOX_360_PID3 ||
                 pid == XBOX_360_PID4 || pid == XBOX_360_PID5 || pid == XBOX_360_PID6 ||
                 pid == XBOX_360_PID7));
    };
    /**@}*/

    /** @name Xbox Controller functions */
    /**
     * getButtonPress(ButtonEnum b) will return true as long as the button is held down.
     *
     * While getButtonClick(ButtonEnum b) will only return it once.
     *
     * So you instance if you need to increase a variable once you would use
     * getButtonClick(ButtonEnum b), but if you need to drive a robot forward you would use
     * getButtonPress(ButtonEnum b).
     * @param  b          ::ButtonEnum to read.
     * @return            getButtonClick(ButtonEnum b) will return a bool, while
     * getButtonPress(ButtonEnum b) will return a word if reading ::L2 or ::R2.
     */
    uint16_t getButtonPress(ButtonEnum b);
    bool     getButtonClick(ButtonEnum b);

    /**
     * Return the analog value from the joysticks on the controller.
     * @param  a          Either ::LeftHatX, ::LeftHatY, ::RightHatX or ::RightHatY.
     * @return            Returns a signed 16-bit integer.
     */
    int16_t getAnalogHat(AnalogHatEnum a);

    /**
     * Used to call your own function when the controller is successfully initialized.
     * @param funcOnInit Function to call.
     */
    void attachOnInit(void (*funcOnInit)(void)) {
        pFuncOnInit = funcOnInit;
    };

    /** Used to set the rumble off. */
    void setRumbleOff();

    /**
     * Used to turn on rumble continuously.
     * @param leftTrigger  Left trigger force.
     * @param rightTrigger Right trigger force.
     * @param leftMotor    Left motor force.
     * @param rightMotor   Right motor force.
     */
    void setRumbleOn(uint8_t leftTrigger, uint8_t rightTrigger, uint8_t leftMotor,
                     uint8_t rightMotor);
    /**@}*/

    /** True if a Xbox ONE controller is connected. */
    bool Xbox360Connected;

    uint8_t XboxCommand(uint8_t *data, uint16_t nbytes);

protected:
    bool needreset;
    USB *pUsb;

    void (*dataCallback)(const uint8_t *data, const uint8_t &ndata);

    /** Pointer to USB class instance. */
    /** Device address. */
    uint8_t bAddress;
    /** Endpoint info structure. */
    EpInfo epInfo[XBOX_ONE_MAX_ENDPOINTS];

    /** Configuration number. */
    uint8_t bConfNum;
    /** Total number of endpoints in the configuration. */
    uint8_t bNumEP;
    /** Next poll time based on poll interval taken from the USB descriptor. */
    uint32_t qNextPollTime;

    /** @name UsbConfigXtracter implementation */
    /**
     * UsbConfigXtracter implementation, used to extract endpoint information.
     * @param conf  Configuration value.
     * @param iface Interface number.
     * @param alt   Alternate setting.
     * @param proto Interface Protocol.
     * @param ep    Endpoint Descriptor.
     */
    void EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto,
                        const USB_ENDPOINT_DESCRIPTOR *ep);
    /**@}*/

    /**
     * Used to print the USB Endpoint Descriptor.
     * @param ep_ptr Pointer to USB Endpoint Descriptor.
     */
    void PrintEndpointDescriptor(const USB_ENDPOINT_DESCRIPTOR *ep_ptr);

private:
    /**
     * Called when the controller is successfully initialized.
     * Use attachOnInit(void (*funcOnInit)(void)) to call your own function.
     */
    void onInit();
    void (*pFuncOnInit)(void);  // Pointer to function called in onInit()

    uint8_t pollInterval;
    bool    bPollEnable;

    /* Variables to store the buttons */
    uint16_t ButtonState;
    uint16_t OldButtonState;
    uint16_t ButtonClickState;
    int16_t  hatValue[4];
    uint16_t triggerValue[2];
    uint16_t triggerValueOld[2];

    bool L2Clicked;  // These buttons are analog, so we use we use these bools to check if they
                     // where clicked or not
    bool R2Clicked;

    bool sharePressed;  // This button doesn't fit in the bitfield
    bool shareClicked;

    uint8_t readBuf[XBOX_360_EP_MAXPKTSIZE];  // General purpose buffer for input data
    uint8_t cmdCounter;

    void readReport();  // Used to read the incoming data

    /* Private commands */
};
#endif

#endif
