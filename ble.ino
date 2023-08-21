#include "ble.h"
#include <BLEDevice.h>
// #include <BLEUtils.h>
// #include <BLEServer.h>

// BLEAdvertising* pAdvertising;

void ble_init() {
  // // 初始化BLE
  // BLEDevice::init("ESP32 Single Broadcast");

  // // 创建BLE广播对象
  // pAdvertising = new BLEAdvertising();

  // // 设置广播数据包
  // BLEAdvertisementData advertisementData;
  // advertisementData.setFlags(0x04);  // Non-connectable, non-scannable undirected event type

  // // 设置广播包中的UUID
  // advertisementData.setCompleteServices(BLEUUID("00001234-0000-1000-8000-00805F9B34FB"));

  // // 设置广播数据
  // pAdvertising->setAdvertisementData(advertisementData);
}

void ble_send() {
  // // 启动BLE广播
  // pAdvertising->start();
  // Serial.println("Single Broadcast started");
  // // 延迟一定时间后停止广播
  // delay(100);
  // pAdvertising->stop();
  // Serial.println("Broadcast stopped");
  // delay(1000);
}
