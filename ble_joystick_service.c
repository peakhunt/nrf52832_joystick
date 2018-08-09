#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "nrf_log.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "ble_hids.h"

#define BASE_USB_HID_SPEC_VERSION         0x0101    /**< Version number of base USB HID Specification implemented by this application. */

#define BLE_JOYSTICK_BTN_UP               15
#define BLE_JOYSTICK_BTN_DOWN             14
#define BLE_JOYSTICK_BTN_LEFT             17
#define BLE_JOYSTICK_BTN_RIGHT            16

#define BLE_JOYSTICK_BTN_0                13
#define BLE_JOYSTICK_BTN_1                12
#define BLE_JOYSTICK_BTN_2                11
//#define BLE_JOYSTICK_BTN_3                10    can't use this pin
#define BLE_JOYSTICK_BTN_3                20
#define BLE_JOYSTICK_BTN_4                18
#define BLE_JOYSTICK_BTN_5                19

#define INPUT_REPORT_COUNT                1

#define INPUT_REP_STICK_INDEX             0
#define INPUT_REP_STICK_LEN               3
#define INPUT_REP_STICK_ID                1

////////////////////////////////////////////////////////////////////////////////
//
// module privates
//
////////////////////////////////////////////////////////////////////////////////
extern uint16_t     m_conn_handle;

static nrf_drv_gpiote_pin_t   _joystick_pins[] =
{
  BLE_JOYSTICK_BTN_UP,
  BLE_JOYSTICK_BTN_DOWN,
  BLE_JOYSTICK_BTN_LEFT,
  BLE_JOYSTICK_BTN_RIGHT,
  BLE_JOYSTICK_BTN_0,
  BLE_JOYSTICK_BTN_1,
  BLE_JOYSTICK_BTN_2,
  BLE_JOYSTICK_BTN_3,
  BLE_JOYSTICK_BTN_4,
  BLE_JOYSTICK_BTN_5
};

static bool _joystick_pin_status[] = 
{
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
  true,
};

BLE_HIDS_DEF(m_hids,
    NRF_SDH_BLE_TOTAL_LINK_COUNT,
    INPUT_REP_STICK_LEN);

APP_TIMER_DEF(m_jstick_timer_id);

static bool   m_in_boot_mode = false;

////////////////////////////////////////////////////////////////////////////////
//
// HID event handler
//
////////////////////////////////////////////////////////////////////////////////
static void
on_joystick_hids_evt(ble_hids_t * p_hids, ble_hids_evt_t * p_evt)
{
  switch (p_evt->evt_type)
  {
  case BLE_HIDS_EVT_BOOT_MODE_ENTERED:
    NRF_LOG_INFO("entrrering boot mode\r\n");
    m_in_boot_mode = true;
    break;

  case BLE_HIDS_EVT_REPORT_MODE_ENTERED:
    NRF_LOG_INFO("entrrering report mode\r\n");
    m_in_boot_mode = false;
    break;

  case BLE_HIDS_EVT_NOTIF_ENABLED:
    NRF_LOG_INFO("BLE_HIDS_EVT_NOTIF_ENABLED\r\n");
    break;

  case BLE_HIDS_EVT_NOTIF_DISABLED:
    NRF_LOG_INFO("BLE_HIDS_EVT_NOTIF_DISABLED\r\n");
    break;

  case BLE_HIDS_EVT_REPORT_READ:
    NRF_LOG_INFO("BLE_HIDS_EVT_REPORT_READ\r\n");
    break;

  case BLE_HIDS_EVT_REP_CHAR_WRITE:
    NRF_LOG_INFO("BLE_HIDS_EVT_REP_CHAR_WRITE\r\n");
    break;

  default:
    NRF_LOG_INFO("BLE_HIDS_EVT %d\r\n", p_evt->evt_type);
    break;
  }
}

static void
joystick_service_error_handler(uint32_t nrf_error)
{
  NRF_LOG_INFO("joystick_service_error_handler : %x\r\n", nrf_error);
  APP_ERROR_HANDLER(nrf_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// GPIO init
//
////////////////////////////////////////////////////////////////////////////////
static void
ble_joystick_report(void)
{
  uint8_t     x,
              y,
              buttons;
  uint8_t     buf[3];
  static uint8_t     prev_buf[3] = { 1, 1, 0};
  ret_code_t  err_code = NRF_SUCCESS;

  x       = 128;
  y       = 128;
  buttons = 0;

  if(_joystick_pin_status[2] == false)
  {
    x = 0;
  }
  else if(_joystick_pin_status[3] == false)
  {
    x = 255;
  }

  if(_joystick_pin_status[0] == false)
  {
    y = 0;
  }
  else if(_joystick_pin_status[1] == false)
  {
    y = 255;
  }

  for(int i = 4; i <= 9; i++)
  {
    if(_joystick_pin_status[i] == false)
    {
      buttons |= (1 << (i-4));
    }
  }

  buf[0] = x;
  buf[1] = y;
  buf[2] = buttons;

  if(m_in_boot_mode)
  {
    return;
  }

  if(buf[0] != prev_buf[0] ||
     buf[1] != prev_buf[1] ||
     buf[2] != prev_buf[2])
  {
    err_code = ble_hids_inp_rep_send(&m_hids,
                                     INPUT_REP_STICK_INDEX,
                                     INPUT_REP_STICK_LEN,
                                     buf,
                                     m_conn_handle);

    prev_buf[0] = buf[0];
    prev_buf[1] = buf[1];
    prev_buf[2] = buf[2];
  }

  if(err_code == NRF_ERROR_RESOURCES)
  {
    nrf_delay_ms(1);
    err_code = ble_hids_inp_rep_send(&m_hids,
        INPUT_REP_STICK_INDEX,
        INPUT_REP_STICK_LEN,
        buf,
        m_conn_handle);

    if(err_code == NRF_ERROR_RESOURCES)
    {
      NRF_LOG_INFO("XXXXXXXXX  err : %x\r\n", err_code);
      return;
    }
  }


#if 0
  NRF_LOG_INFO("x: %d, y: %d: btn: %x\n", x, y, buttons);
  {
    static int cnt = 0;

    cnt++;
    if(cnt >= 200)
    {
      NRF_LOG_INFO("1: %d %d %d %d\n",
          _joystick_pin_status[0],
          _joystick_pin_status[1],
          _joystick_pin_status[2],
          _joystick_pin_status[3]);
      NRF_LOG_INFO("2: %d %d %d %d\n",
          _joystick_pin_status[4],
          _joystick_pin_status[5],
          _joystick_pin_status[6],
          _joystick_pin_status[7]);
      NRF_LOG_INFO("3: %d %d\n",
          _joystick_pin_status[8],
          _joystick_pin_status[9]);
      cnt = 0;
    }
  }
#endif

#if 0
  if ((err_code != NRF_SUCCESS) &&
      (err_code != NRF_ERROR_INVALID_STATE) &&
      (err_code != NRF_ERROR_RESOURCES) &&
      (err_code != NRF_ERROR_BUSY) &&
      (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
     )
  {
    NRF_LOG_INFO("ble_joystick_report err : %x\r\n", err_code);
    //APP_ERROR_HANDLER(err_code);
  }
#else
  if (err_code != NRF_SUCCESS)
  {
    NRF_LOG_INFO("ble_joystick_report err : %x\r\n", err_code);
  }
#endif
}

static void
joystick_pin_read(void)
{
  for(int i = 0; i < sizeof(_joystick_pins)/sizeof(nrf_drv_gpiote_pin_t); i++)
  {
    uint32_t level;   // 0 : low, 1 : high

    level = nrf_gpio_pin_read(_joystick_pins[i]);

    _joystick_pin_status[i] = level == 0 ? false : true;
  }
}

static void
jstick_timer_handler(void * p_context)
{
  if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
  {
    joystick_pin_read();
    ble_joystick_report();
  }
}

static void
joystick_pin_detect_init(void)
{
  for(int i = 0; i < sizeof(_joystick_pins)/sizeof(nrf_drv_gpiote_pin_t); i++)
  {
    NRF_LOG_INFO("configuring %d for input\n", i);
    nrf_gpio_cfg_input(_joystick_pins[i], NRF_GPIO_PIN_NOPULL);
  }
}

////////////////////////////////////////////////////////////////////////////////
//
// BLE HID service init
//
////////////////////////////////////////////////////////////////////////////////
static void
ble_joystick_hids_init(void)
{
  uint32_t                  err_code;
  ble_hids_init_t           hids_init_obj;
  ble_hids_inp_rep_init_t*  p_input_report;
  uint8_t                   hid_info_flags;

  static ble_hids_inp_rep_init_t   inp_rep_array[INPUT_REPORT_COUNT];
  static uint8_t rep_map_data[] =
  {
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,                    // USAGE (Game Pad)
    0xa1, 0x01,                    // COLLECTION (Application)

      0x85, 0x01,                    //   Report ID : 1
      0x09, 0x01,                    //   USAGE (Pointer)
      0xa1, 0x00,                    //   COLLECTION (Physical)
        0x75, 0x08,                    //     Report Size: 8
        0x95, 0x02,                    //     REPORT_COUNT (2)
        0x05, 0x01,                    //     USAGE_PAGE (Generic Desktop)
        0x09, 0x30,                    //     USAGE (X)
        0x09, 0x31,                    //     USAGE (Y)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x25, 0xff,                    //     LOGICAL_MAXIMUM (255)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
        0x75, 0x01,                    //     Report Size: 1
        0x95, 0x08,                    //     REPORT_COUNT (8)
        0x05, 0x09,                    //     USAGE_PAGE (Button)
        0x19, 0x01,                    //     USAGE_MINIMUM (Button 1)
        0x29, 0x08,                    //     USAGE_MAXIMUM (Button 8)
        0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
        0x25, 0x01,                    //     LOGICAL_MAXIMUM (1)
        0x81, 0x02,                    //     INPUT (Data,Var,Abs)
      0xc0,                          //   END_COLLECTION (Physical)
    0xc0,                          // END_COLLECTION (Application)
  };

  memset(inp_rep_array, 0, sizeof(inp_rep_array));

  // Initialize HID Service.
  p_input_report                      = &inp_rep_array[INPUT_REP_STICK_INDEX];
  p_input_report->max_len             = INPUT_REP_STICK_LEN;
  p_input_report->rep_ref.report_id   = INPUT_REP_STICK_ID;
  p_input_report->rep_ref.report_type = BLE_HIDS_REP_TYPE_INPUT;

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&p_input_report->security_mode.write_perm);

  hid_info_flags = HID_INFO_FLAG_REMOTE_WAKE_MSK | HID_INFO_FLAG_NORMALLY_CONNECTABLE_MSK;

  memset(&hids_init_obj, 0, sizeof(hids_init_obj));

  hids_init_obj.evt_handler                    = on_joystick_hids_evt;
  hids_init_obj.error_handler                  = joystick_service_error_handler;
  hids_init_obj.is_kb                          = false;
  hids_init_obj.is_mouse                       = false;
  hids_init_obj.inp_rep_count                  = INPUT_REPORT_COUNT;
  hids_init_obj.p_inp_rep_array                = inp_rep_array;
  hids_init_obj.outp_rep_count                 = 0;
  hids_init_obj.p_outp_rep_array               = NULL;
  hids_init_obj.feature_rep_count              = 0;
  hids_init_obj.p_feature_rep_array            = NULL;
  hids_init_obj.rep_map.data_len               = sizeof(rep_map_data);
  hids_init_obj.rep_map.p_data                 = rep_map_data;
  hids_init_obj.hid_information.bcd_hid        = BASE_USB_HID_SPEC_VERSION;
  hids_init_obj.hid_information.b_country_code = 0;
  hids_init_obj.hid_information.flags          = hid_info_flags;
  hids_init_obj.included_services_count        = 0;
  hids_init_obj.p_included_services_array      = NULL;

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.rep_map.security_mode.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.rep_map.security_mode.write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.hid_information.security_mode.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.hid_information.security_mode.write_perm);

  /*
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_boot_mouse_inp_rep.write_perm);
  */

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_protocol.write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&hids_init_obj.security_mode_ctrl_point.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&hids_init_obj.security_mode_ctrl_point.write_perm);

  err_code = ble_hids_init(&m_hids, &hids_init_obj);
  APP_ERROR_CHECK(err_code);
}

////////////////////////////////////////////////////////////////////////////////
//
// module public
//
////////////////////////////////////////////////////////////////////////////////
void
ble_joystick_service_init(void)
{
  #define APP_TIMER_PRESCALER             0   // in main actually

  uint32_t err_code;

  joystick_pin_detect_init();
  ble_joystick_hids_init();

  // Create timers
  err_code = app_timer_create(&m_jstick_timer_id, APP_TIMER_MODE_REPEATED, jstick_timer_handler);
  APP_ERROR_CHECK(err_code);

  // start
  app_timer_start(m_jstick_timer_id, APP_TIMER_TICKS(8), NULL);

  // app scheduler is initialized in main.c
}
