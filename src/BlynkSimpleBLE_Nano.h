/**
 * @file       BlynkSimpleBLE_Nano.h
 * @author     Daniel Lee
 * @license    This project is released under the MIT License (MIT)
 * @copyright  Copyright (c) 2016 Daniel Lee
 * @date       Nov 2016
 * @brief
 *
 */

#ifndef BlynkSimpleBLE_Nano_h
#define BlynkSimpleBLE_Nano_h

#ifndef BLYNK_INFO_CONNECTION
#define BLYNK_INFO_CONNECTION "RBL_BLE_Nano"
#endif

#define BLYNK_NO_YIELD
#define BLYNK_SEND_ATOMIC
#define BLYNK_SEND_CHUNK 20
#define BLYNK_SEND_THROTTLE 20

#include <BlynkApiArduino.h>
#include <Blynk/BlynkProtocol.h>
#include <utility/BlynkFifo2.h>
#include <BLEPeripheral.h>

/*
 * The Nordic UART Service
 */
static const uint8_t uart_base_uuid[]     = {0x71, 0x3D, 0, 0, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_base_uuid_rev[] = {0x1E, 0x94, 0x8D, 0xF1, 0x48, 0x31, 0x94, 0xBA, 0x75, 0x4C, 0x3E, 0x50, 0, 0, 0x3D, 0x71};
static const uint8_t uart_tx_uuid[]       = {0x71, 0x3D, 0, 3, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_rx_uuid[]       = {0x71, 0x3D, 0, 2, 0x50, 0x3E, 0x4C, 0x75, 0xBA, 0x94, 0x31, 0x48, 0xF1, 0x8D, 0x94, 0x1E};
static const uint8_t uart_dev_name[]      = "Blynk";

#define TXRX_BUF_LEN 20
uint8_t txPayload[TXRX_BUF_LEN] = {0,};
uint8_t rxPayload[TXRX_BUF_LEN] = {0,};

//BLEService uartService = BLEService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
BLEService uartService = BLEService("713D0000-503E-4C75-BA94-3148F18D941E");
//BLEDescriptor uartNameDescriptor = BLEDescriptor("2901", "Blynk"); //Changed from UART
    
/*GattCharacteristic  txCharacteristic (uart_tx_uuid, txPayload, 1, TXRX_BUF_LEN,
                                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE);
*/
//BLECharacteristic txCharacteristic = BLECharacteristic("6E400003-B5A3-F393-E0A9-E50E24DCCA9E", BLENotify, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
BLECharacteristic txCharacteristic = BLECharacteristic("713D0003-503E-4C75-BA94-3148F18D941E", BLEWrite | BLEWriteWithoutResponse, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
//BLEDescriptor txNameDescriptor = BLEDescriptor("2901", "TX - Transfer Data (Write)");

/*GattCharacteristic  rxCharacteristic (uart_rx_uuid, rxPayload, 1, TXRX_BUF_LEN,
                                      GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY);
*/
//BLECharacteristic rxCharacteristic = BLECharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWriteWithoutResponse, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
BLECharacteristic rxCharacteristic = BLECharacteristic("713D0002-503E-4C75-BA94-3148F18D941E", BLENotify, BLE_ATTRIBUTE_MAX_VALUE_LENGTH);
//BLEDescriptor rxNameDescriptor = BLEDescriptor("2901", "RX - Receive Data (Notify)");
       
/*GattCharacteristic *uartChars[] = {&txCharacteristic, &rxCharacteristic};
*/

//GattService         uartService(uart_base_uuid, uartChars, sizeof(uartChars) / sizeof(GattCharacteristic*));


#define BLE_REQ   -1
#define BLE_RDY   -1
#define BLE_RST   -1
BLEPeripheral  ble = BLEPeripheral(BLE_REQ, BLE_RDY, BLE_RST);

class BlynkTransportRedBearLab_BLE_Nano
{
public:
    BlynkTransportRedBearLab_BLE_Nano()
        : mConn (false)
    {}

    // IP redirect not available
    void begin(char* h, uint16_t p) {}

    void begin() {
        instance = this;

//        ble.gap().onConnection(connectCallback);
//        ble.gap().onDisconnection(disconnectCallback);
        ble.setEventHandler(BLEConnected, connectCallback);
        ble.setEventHandler(BLEDisconnected, disconnectCallback);



/*
        ble.gattServer().addService(uartService);
        ble.gattServer().onDataWritten(writeCallback);
        ble.gattServer().onDataSent(sentCallback);
*/
        ble.addAttribute(uartService);
//        ble.addAttribute(uartNameDescriptor);
        ble.setAdvertisedServiceUuid(uartService.uuid());
        ble.addAttribute(txCharacteristic);
//        ble.addAttribute(txNameDescriptor);
        ble.addAttribute(rxCharacteristic);
//        ble.addAttribute(rxNameDescriptor);
//        ble.setEventHandler(BLEWritten, BLESerial::_received);
        rxCharacteristic.setEventHandler(BLEWritten, writeCallback);
        
//        ble.setEventHandler(BLEWritten, sentCallback)
//        txCharacteristic.setEventHandler(BLEWritten, sendCallback);

        // Setup advertising
 /*       ble.gap().accumulateAdvertisingPayload(GapAdvertisingData::COMPLETE_LIST_128BIT_SERVICE_IDS,
                                              uart_base_uuid_rev, sizeof(uart_base_uuid));
*/
    }

    bool connect() {
        mBuffRX.clear();
        return mConn = true;
    }

    void disconnect() {
        mConn = false;
    }

    bool connected() {
        return mConn;
    }

    size_t read(void* buf, size_t len) {
        uint32_t start = millis();
        while (millis() - start < BLYNK_TIMEOUT_MS) {
            if (available() < len) {
 //               ble.waitForEvent();
                ble.poll();
            } else {
                break;
            }
        }
        noInterrupts();
        size_t res = mBuffRX.get((uint8_t*)buf, len);
        interrupts();
        return res;
    }

    size_t write(const void* buf, size_t len) {
//        ble.updateCharacteristicValue(rxCharacteristic.getValueAttribute().getHandle(), (uint8_t*)buf, len);
        return len;
    }

    size_t available() {
        noInterrupts();
        size_t rxSize = mBuffRX.size();
        interrupts();
        return rxSize;
    }

private:
    static BlynkTransportRedBearLab_BLE_Nano* instance;
/*
    static
    void writeCallback(const GattWriteCallbackParams *params)
    {
        if (!instance)
            return;
      noInterrupts();
      //BLYNK_DBG_DUMP(">> ", params->data, params->len);
      instance->mBuffRX.put(params->data, params->len);
      interrupts();
    }
*/
//    static
//    void writeCallback (const uint8_t* data, size_t size);
    static
    void writeCallback (BLECentral& , BLECharacteristic& rxCharacteristic)
    {        
        if (!instance)
            return;
        noInterrupts();
/*        
        for (int i = 0; i < size; i++) {
            this->_rxHead = (this->_rxHead + 1) % sizeof(this->_rxBuffer);
            this->_rxBuffer[this->_rxHead] = data[i];
        } */

        instance->mBuffRX.put(rxCharacteristic.value(), rxCharacteristic.valueLength());
        interrupts();
  /*#ifdef BLE_SERIAL_DEBUG
    Serial.print(F("BLESerial::received("));
    for (int i = 0; i < size; i++) Serial.print((char) data[i]);
    Serial.println(F(")"));
  #endif*/
    }

    static
    void sentCallback(unsigned count)
    {
      //Serial.print("SENT: ");
      //Serial.println(count);
    }

    static
    void connectCallback(BLECentral& central);

    static
    void disconnectCallback(BLECentral& central);

private:
    bool mConn;

    BlynkFifo<uint8_t, BLYNK_MAX_READBYTES*2> mBuffRX;
};

class BlynkRedBearLab_BLE_Nano
    : public BlynkProtocol<BlynkTransportRedBearLab_BLE_Nano>
{
    typedef BlynkProtocol<BlynkTransportRedBearLab_BLE_Nano> Base;
public:
    BlynkRedBearLab_BLE_Nano(BlynkTransportRedBearLab_BLE_Nano& transp)
        : Base(transp)
    {}

    void begin(const char* auth)
    {
        Base::begin(auth);
        state = DISCONNECTED;
        conn.begin();
    }
};

BlynkTransportRedBearLab_BLE_Nano* BlynkTransportRedBearLab_BLE_Nano::instance = NULL;

static BlynkTransportRedBearLab_BLE_Nano _blynkTransport;
BlynkRedBearLab_BLE_Nano Blynk(_blynkTransport);

void BlynkTransportRedBearLab_BLE_Nano::connectCallback(BLECentral& central)
{
  BLYNK_LOG1("Device connected");
  Blynk.startSession();
}

void BlynkTransportRedBearLab_BLE_Nano::disconnectCallback(BLECentral& central)
{
  BLYNK_LOG1("Device disconnected");
  //__disable_irq();
  Blynk.disconnect();
  //__enable_irq();
//  ble.startAdvertising();
}

#include <BlynkWidgets.h>

#endif
