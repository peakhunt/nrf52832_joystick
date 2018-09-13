#ifndef __BLE_JOYSTICK_SERVICE_DEF_H__
#define __BLE_JOYSTICK_SERVICE_DEF_H__

extern void ble_joystick_service_init(void);
extern void ble_joystick_handle_ble_evt(ble_evt_t* p_ble_evt);
extern void ble_joystick_service_resume_tx(void);
extern void ble_joystick_disconnected(void);

#endif /* !__BLE_JOYSTICK_SERVICE_DEF_H__ */
