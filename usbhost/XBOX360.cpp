#pragma once
#include "XBOX360.h"
#include "debug_helpers.h"
#include "usbhub.h"
#include "HardwareSerial.h"
#include "usbhost.h"

XBOX360::XBOX360(USB *pUsb, void (*data_cb)(const uint8_t *data, const uint8_t &ndata))
    : pUsb(pUsb), // pointer to USB class instance - mandatory
      dataCallback(data_cb),
      bAddress(0), // device address - mandatory
      bNumEP(1), // If config descriptor needs to be parsed
      qNextPollTime(0),  // Reset NextPollTime
      pollInterval(0),
      bPollEnable(false),
      Xbox360Connected(false),
      pFuncOnInit(nullptr),
      needreset(false), {  // don't start polling before dongle is connected
      for (uint8_t i = 0; i < XBOX_360_MAX_ENDPOINTS; i++) {
        epInfo[i].epAddr      = 0;
        epInfo[i].maxPktSize  = (i) ? 0 : 8;
        epInfo[i].bmSndToggle = 0;
        epInfo[i].bmRcvToggle = 0;
        epInfo[i].bmNakPower  = (i) ? USB_NAK_NOWAIT : USB_NAK_MAX_POWER;
    }
}

uint8_t XBOX360::Init(uint8_t parent, uint8_t port, bool lowspeed) {
    uint8_t                buf[sizeof(USB_DEVICE_DESCRIPTOR)];
    USB_DEVICE_DESCRIPTOR *udd = reinterpret_cast<USB_DEVICE_DESCRIPTOR *>(buf);
    uint8_t                rcode;
    UsbDevice             *p         = NULL;
    EpInfo                *oldep_ptr = NULL;
    uint16_t               PID, VID;
    uint8_t                num_of_conf;  // Number of configurations
    AddressPool &addrPool = pUsb->GetAddressPool();

#ifdef EXTRADEBUG
    Notify(PSTR("\r\nXBOXONE Init"), 0x80);
#endif
    // check if address has already been assigned to an instance
    if (bAddress) {
        // SERIAL_DEBUG.print("ADDRESS IN USE");
#ifdef DEBUG_USB_HOST
        Notify(PSTR("\r\nAddress in use"), 0x80);
#endif
        return USB_ERROR_CLASS_INSTANCE_ALREADY_IN_USE;
    }
    // Get pointer to pseudo device with address 0 assigned
    p = addrPool.GetUsbDevicePtr(0);
    if (!p) {
#ifdef DEBUG_USB_HOST
        Notify(PSTR("\r\nAddress not found"), 0x80);
#endif
        return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;
    }

    if (!p->epinfo) {
#ifdef DEBUG_USB_HOST
        Notify(PSTR("\r\nepinfo is null"), 0x80);
#endif
        return USB_ERROR_EPINFO_IS_NULL;
    }

    // Save old pointer to EP_RECORD of address 0
    oldep_ptr = p->epinfo;

    // Temporary assign new pointer to epInfo to p->epinfo in order to avoid toggle inconsistence
    p->epinfo = epInfo;

    p->lowspeed = lowspeed;

    // Get device descriptor
    rcode = pUsb->getDevDescr(0, 0, sizeof(USB_DEVICE_DESCRIPTOR),
                              (uint8_t *)buf);  // Get device descriptor - addr, ep, nbytes, data
    // Restore p->epinfo
    p->epinfo = oldep_ptr;

    if (rcode)
        goto FailGetDevDescr;

    VID = udd->idVendor;
    PID = udd->idProduct;

    if (!VIDPIDOK(VID, PID))  // Check VID
        goto FailUnknownDevice;

    // Allocate new address according to device class
    bAddress = addrPool.AllocAddress(parent, false, port);

    if (!bAddress)
        return USB_ERROR_OUT_OF_ADDRESS_SPACE_IN_POOL;

    // Extract Max Packet Size from device descriptor
    epInfo[0].maxPktSize = udd->bMaxPacketSize0;

    // Assign new address to the device
    rcode = pUsb->setAddr(0, 0, bAddress);
    if (rcode) {
        p->lowspeed = false;
        addrPool.FreeAddress(bAddress);
        bAddress = 0;
#ifdef DEBUG_USB_HOST
        Notify(PSTR("\r\nsetAddr: "), 0x80);
        D_PrintHex<uint8_t>(rcode, 0x80);
#endif
        return rcode;
    }
#ifdef EXTRADEBUG
    Notify(PSTR("\r\nAddr: "), 0x80);
    D_PrintHex<uint8_t>(bAddress, 0x80);
#endif
    // delay(300); // Spec says you should wait at least 200ms

    p->lowspeed = false;

    // get pointer to assigned address record
    p = addrPool.GetUsbDevicePtr(bAddress);
    if (!p)
        return USB_ERROR_ADDRESS_NOT_FOUND_IN_POOL;

    p->lowspeed = lowspeed;

    // Assign epInfo to epinfo pointer - only EP0 is known
    rcode = pUsb->setEpInfoEntry(bAddress, 1, epInfo);
    if (rcode)
        goto FailSetDevTblEntry;

    num_of_conf = udd->bNumConfigurations;  // Number of configurations

    USBTRACE2("NC:", num_of_conf);

    // Check if attached device is a Xbox One controller and fill endpoint data structure
    for (uint8_t i = 0; i < num_of_conf; i++) {
        ConfigDescParser<0, 0, 0, 0> confDescrParser(
            this);  // Allow all devices, as we have already verified that it is a Xbox One
                    // controller from the VID and PID
        rcode = pUsb->getConfDescr(bAddress, 0, i, &confDescrParser);
        if (rcode)  // Check error code
            goto FailGetConfDescr;
        if (bNumEP >= XBOX_360_MAX_ENDPOINTS)  // All endpoints extracted
            break;
    }

    if (bNumEP < XBOX_360_MAX_ENDPOINTS)
        goto FailUnknownDevice;

    rcode = pUsb->setEpInfoEntry(bAddress, bNumEP, epInfo);
    if (rcode)
        goto FailSetDevTblEntry;

    delay(200);  // Give time for address change

    rcode = pUsb->setConf(bAddress, epInfo[XBOX_360_CONTROL_PIPE].epAddr, bConfNum);
    if (rcode)
        goto FailSetConfDescr;

#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nXbox 360 Controller Connected\r\n"), 0x80);
#endif

    delay(200);  // let things settle

    // // Initialize the controller for input
    cmdCounter = 0;  // Reset the counter used when sending out the commands
    uint8_t writeBuf[5];
    writeBuf[0] = 0x05;
    writeBuf[1] = 0x20;
    // Byte 2 is set in "XboxCommand"
    writeBuf[3] = 0x01;
    writeBuf[4] = 0x00;
    rcode       = XboxCommand(writeBuf, 5);
    if (rcode)
        goto Fail;

    Xbox360Connected = true;
    bPollEnable      = true;
    return 0;  // Successful configuration

    /* Diagnostic messages */
FailGetDevDescr:
#ifdef DEBUG_USB_HOST
    NotifyFailGetDevDescr();
    goto Fail;
#endif

FailSetDevTblEntry:
#ifdef DEBUG_USB_HOST
    NotifyFailSetDevTblEntry();
    goto Fail;
#endif

FailGetConfDescr:
#ifdef DEBUG_USB_HOST
    NotifyFailGetConfDescr();
    goto Fail;
#endif

FailSetConfDescr:
#ifdef DEBUG_USB_HOST
    NotifyFailSetConfDescr();
#endif
    goto Fail;

FailUnknownDevice:
#ifdef DEBUG_USB_HOST
    NotifyFailUnknownDevice(VID, PID);
#endif
    rcode = USB_DEV_CONFIG_ERROR_DEVICE_NOT_SUPPORTED;

Fail:
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nXbox 360 Init Failed, error code: "), 0x80);
    NotifyFail(rcode);
#endif
    Release();
    return rcode;
}

/* Performs a cleanup after failed Init() attempt */
uint8_t XBOX360::Release() {
    Xbox360Connected = false;
    pUsb->GetAddressPool().FreeAddress(bAddress);
    bAddress      = 0;  // Clear device address
    bNumEP        = 1;  // Must have to be reset to 1
    qNextPollTime = 0;  // Reset next poll time
    pollInterval  = 0;
    bPollEnable   = false;
#ifdef DEBUG_USB_HOST
    Notify(PSTR("\r\nXBOX360 Controller Disconnected\r\n"), 0x80);
#endif
    return 0;
}

uint8_t XBOX360::Poll() {
    uint8_t rcode = 0;

    if (!bPollEnable)
        return 0;
    // Do not poll if shorter than polling interval
    if ((int32_t)((uint32_t)millis() - qNextPollTime) >= 0L) {
        qNextPollTime   = (uint32_t)millis() + pollInterval;  // Set new poll time
        uint16_t length = (uint16_t)epInfo[XBOX_360_INPUT_PIPE]
                              .maxPktSize;  // Read the maximum packet size from the endpoint
        uint8_t rcode = pUsb->inTransfer(bAddress, epInfo[XBOX_360_INPUT_PIPE].epAddr, &length,
                                         readBuf, pollInterval);
        // char buf[64];
        // snprintf(buf, 64, "rcode: %u", rcode);
        // SERIAL_DEBUG.println(buf);
        if (!rcode) {
            if (dataCallback)
                (*dataCallback)(readBuf, length);
            readReport();
        }
    }
    return rcode;
}

void XBOX360::readReport() {
    if (dataCallback)
        dataCallback(readBuf, sizeof(readBuf));

    // Parse report and update button/analog states
    OldButtonState = ButtonState;
    ButtonState = (readBuf[3] << 8) | readBuf[2];

    for (int i = 0; i < 4; i++)
        hatValue[i] = (int16_t)((readBuf[6 + i * 2 + 1] << 8) | readBuf[6 + i * 2]);

    triggerValueOld[0] = triggerValue[0];
    triggerValueOld[1] = triggerValue[1];

    triggerValue[0] = readBuf[4];
    triggerValue[1] = readBuf[5];
}

void XBOX360::EndpointXtract(uint8_t conf, uint8_t iface, uint8_t alt, uint8_t proto,
                             const USB_ENDPOINT_DESCRIPTOR *ep) {
    if (bNumEP > XBOX_360_MAX_ENDPOINTS)
        return;

    EpInfo *pep = &epInfo[bNumEP];
    pep->epAddr = ep->bEndpointAddress & 0x0F;
    pep->maxPktSize = ep->wMaxPacketSize;
    pep->epAttribs = 0;
    pep->bmNakPower = USB_NAK_NOWAIT;

    if ((ep->bmAttributes & 0x03) == 3)
        pollInterval = ep->bInterval;

    bNumEP++;
}

void XBOX360::setRumbleOff() {
    uint8_t buf[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    XboxCommand(buf, sizeof(buf));
}

void XBOX360::setRumbleOn(uint8_t leftTrigger, uint8_t rightTrigger, uint8_t leftMotor,
                          uint8_t rightMotor) {
    uint8_t buf[8] = {0x00, leftTrigger, rightTrigger, leftMotor, rightMotor, 0x00, 0x00, 0x00};
    XboxCommand(buf, sizeof(buf));
}

uint8_t XBOX360::XboxCommand(uint8_t *data, uint16_t nbytes) {
    return pUsb->outTransfer(bAddress, epInfo[2].epAddr, nbytes, data);
}

void XBOX360::onInit() {
    if (pFuncOnInit)
        pFuncOnInit();
}
