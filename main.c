#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_hids.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "ble_advertising.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "ble_joystick_service.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define DEVICE_NAME                     "HKJoystick"                              /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "HKim"                       /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define BATTERY_LEVEL_MEAS_INTERVAL     APP_TIMER_TICKS(2000)                       /**< Battery level measurement interval (ticks). */
#define MIN_BATTERY_LEVEL               81                                          /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL               100                                         /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT         1                                           /**< Increment between each simulated battery level measurement. */

#define PNP_ID_VENDOR_ID_SOURCE         0x02                                        /**< Vendor ID Source. */
#define PNP_ID_VENDOR_ID                0x1915                                      /**< Vendor ID. */
#define PNP_ID_PRODUCT_ID               0xEEEE                                      /**< Product ID. */
#define PNP_ID_PRODUCT_VERSION          0x0001                                      /**< Product Version. */

/*lint -emacro(524, MIN_CONN_INTERVAL) // Loss of precision */
#if 0
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15, UNIT_1_25_MS)             /**< Maximum connection interval (15 ms). */
#else
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)            /**< Minimum connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)              /**< Maximum connection interval (15 ms). */
#endif
#define SLAVE_LATENCY                   20                                          /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(3000, UNIT_10_MS)             /**< Connection supervisory timeout (3000 ms). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAM_UPDATE_COUNT     3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

#define MOVEMENT_SPEED                  5                                           /**< Number of pixels by which the cursor is moved each time a button is pushed. */
#define INPUT_REPORT_COUNT              3                                           /**< Number of input reports in this application. */
#define INPUT_REP_BUTTONS_LEN           3                                           /**< Length of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_LEN          3                                           /**< Length of Mouse Input Report containing movement data. */
#define INPUT_REP_MEDIA_PLAYER_LEN      1                                           /**< Length of Mouse Input Report containing media player data. */
#define INPUT_REP_BUTTONS_INDEX         0                                           /**< Index of Mouse Input Report containing button data. */
#define INPUT_REP_MOVEMENT_INDEX        1                                           /**< Index of Mouse Input Report containing movement data. */
#define INPUT_REP_MPLAYER_INDEX         2                                           /**< Index of Mouse Input Report containing media player data. */
#define INPUT_REP_REF_BUTTONS_ID        1                                           /**< Id of reference to Mouse Input Report containing button data. */
#define INPUT_REP_REF_MOVEMENT_ID       2                                           /**< Id of reference to Mouse Input Report containing movement data. */
#define INPUT_REP_REF_MPLAYER_ID        3                                           /**< Id of reference to Mouse Input Report containing media player data. */

#define BASE_USB_HID_SPEC_VERSION       0x0101                                      /**< Version number of base USB HID Specification implemented by this application. */

#define SCHED_MAX_EVENT_DATA_SIZE       APP_TIMER_SCHED_EVENT_DATA_SIZE             /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE                20                                          /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE                10                                          /**< Maximum number of events in the scheduler queue. */
#endif

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_ADV_FAST_INTERVAL           0x0028                                      /**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_ADV_SLOW_INTERVAL           0x0C80                                      /**< Slow advertising interval (in units of 0.625 ms. This value corrsponds to 2 seconds). */

#define APP_ADV_FAST_DURATION           3000                                        /**< The advertising duration of fast advertising in units of 10 milliseconds. */
#define APP_ADV_SLOW_DURATION           18000                                       /**< The advertising duration of slow advertising in units of 10 milliseconds. */


APP_TIMER_DEF(m_battery_timer_id);                                                  /**< Battery timer. */
BLE_BAS_DEF(m_bas);                                                                 /**< Battery service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

uint16_t          m_conn_handle  = BLE_CONN_HANDLE_INVALID;                         /**< Handle of the current connection. */
static pm_peer_id_t      m_peer_id;                                                 /**< Device reference handle to the current bonded central. */
static sensorsim_cfg_t   m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */
static pm_peer_id_t      m_whitelist_peers[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];       /**< List of peers currently in the whitelist. */
static uint32_t          m_whitelist_peer_cnt;                                      /**< Number of peers currently in the whitelist. */
static ble_uuid_t        m_adv_uuids[] =                                            /**< Universally unique service identifiers. */
{
    {BLE_UUID_HUMAN_INTERFACE_DEVICE_SERVICE, BLE_UUID_TYPE_BLE}
};



/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void
assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
  app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Fetch the list of peer manager peer IDs.
 *
 * @param[inout] p_peers   The buffer where to store the list of peer IDs.
 * @param[inout] p_size    In: The size of the @p p_peers buffer.
 *                         Out: The number of peers copied in the buffer.
 */
static void
peer_list_get(pm_peer_id_t * p_peers, uint32_t * p_size)
{
  pm_peer_id_t peer_id;
  uint32_t     peers_to_copy;

  peers_to_copy = (*p_size < BLE_GAP_WHITELIST_ADDR_MAX_COUNT) ?
    *p_size : BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

  peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);
  *p_size = 0;

  while ((peer_id != PM_PEER_ID_INVALID) && (peers_to_copy--))
  {
    p_peers[(*p_size)++] = peer_id;
    peer_id = pm_next_peer_id_get(peer_id);
  }
}


/**@brief Clear bond information from persistent storage.
 */
static void
delete_bonds(void)
{
  ret_code_t err_code;

  NRF_LOG_INFO("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void
advertising_start(bool erase_bonds)
{
  if (erase_bonds == true)
  {
    delete_bonds();
    // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
  }
  else
  {
    ret_code_t ret;

    memset(m_whitelist_peers, PM_PEER_ID_INVALID, sizeof(m_whitelist_peers));
    m_whitelist_peer_cnt = (sizeof(m_whitelist_peers) / sizeof(pm_peer_id_t));

    peer_list_get(m_whitelist_peers, &m_whitelist_peer_cnt);

    ret = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
    APP_ERROR_CHECK(ret);

    // Setup the device identies list.
    // Some SoftDevices do not support this feature.
    ret = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
    if (ret != NRF_ERROR_NOT_SUPPORTED)
    {
      APP_ERROR_CHECK(ret);
    }

    ret = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(ret);
  }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void
pm_evt_handler(pm_evt_t const * p_evt)
{
  ret_code_t err_code;

  switch (p_evt->evt_id)
  {
  case PM_EVT_BONDED_PEER_CONNECTED:
    {
      NRF_LOG_INFO("Connected to a previously bonded device.");
    } break;

  case PM_EVT_CONN_SEC_SUCCEEDED:
    {
      NRF_LOG_INFO("Connection secured: role: %d, conn_handle: 0x%x, procedure: %d.",
          ble_conn_state_role(p_evt->conn_handle),
          p_evt->conn_handle,
          p_evt->params.conn_sec_succeeded.procedure);

      m_peer_id = p_evt->peer_id;
    } break;

  case PM_EVT_CONN_SEC_FAILED:
    {
      /* Often, when securing fails, it shouldn't be restarted, for security reasons.
       * Other times, it can be restarted directly.
       * Sometimes it can be restarted, but only after changing some Security Parameters.
       * Sometimes, it cannot be restarted until the link is disconnected and reconnected.
       * Sometimes it is impossible, to secure the link, or the peer device does not support it.
       * How to handle this error is highly application dependent. */
    } break;

  case PM_EVT_CONN_SEC_CONFIG_REQ:
    {
      // Reject pairing request from an already bonded peer.
      pm_conn_sec_config_t conn_sec_config = {.allow_repairing = false};
      pm_conn_sec_config_reply(p_evt->conn_handle, &conn_sec_config);
    } break;

  case PM_EVT_STORAGE_FULL:
    {
      // Run garbage collection on the flash.
      err_code = fds_gc();
      if (err_code == FDS_ERR_NO_SPACE_IN_QUEUES)
      {
        // Retry.
      }
      else
      {
        APP_ERROR_CHECK(err_code);
      }
    } break;

  case PM_EVT_PEERS_DELETE_SUCCEEDED:
    {
      advertising_start(false);
    } break;

  case PM_EVT_PEER_DATA_UPDATE_SUCCEEDED:
    {
      if (     p_evt->params.peer_data_update_succeeded.flash_changed
          && (p_evt->params.peer_data_update_succeeded.data_id == PM_PEER_DATA_ID_BONDING))
      {
        NRF_LOG_INFO("New Bond, add the peer to the whitelist if possible");
        NRF_LOG_INFO("\tm_whitelist_peer_cnt %d, MAX_PEERS_WLIST %d",
            m_whitelist_peer_cnt + 1,
            BLE_GAP_WHITELIST_ADDR_MAX_COUNT);
        // Note: You should check on what kind of white list policy your application should use.

        if (m_whitelist_peer_cnt < BLE_GAP_WHITELIST_ADDR_MAX_COUNT)
        {
          // Bonded to a new peer, add it to the whitelist.
          m_whitelist_peers[m_whitelist_peer_cnt++] = m_peer_id;

          // The whitelist has been modified, update it in the Peer Manager.
          err_code = pm_whitelist_set(m_whitelist_peers, m_whitelist_peer_cnt);
          APP_ERROR_CHECK(err_code);

          err_code = pm_device_identities_list_set(m_whitelist_peers, m_whitelist_peer_cnt);
          if (err_code != NRF_ERROR_NOT_SUPPORTED)
          {
            APP_ERROR_CHECK(err_code);
          }
        }
      }
    } break;

  case PM_EVT_PEER_DATA_UPDATE_FAILED:
    {
      // Assert.
      APP_ERROR_CHECK(p_evt->params.peer_data_update_failed.error);
    } break;

  case PM_EVT_PEER_DELETE_FAILED:
    {
      // Assert.
      APP_ERROR_CHECK(p_evt->params.peer_delete_failed.error);
    } break;

  case PM_EVT_PEERS_DELETE_FAILED:
    {
      // Assert.
      APP_ERROR_CHECK(p_evt->params.peers_delete_failed_evt.error);
    } break;

  case PM_EVT_ERROR_UNEXPECTED:
    {
      // Assert.
      APP_ERROR_CHECK(p_evt->params.error_unexpected.error);
    } break;

  case PM_EVT_CONN_SEC_START:
  case PM_EVT_PEER_DELETE_SUCCEEDED:
  case PM_EVT_LOCAL_DB_CACHE_APPLIED:
  case PM_EVT_LOCAL_DB_CACHE_APPLY_FAILED:
    // This can happen when the local DB has changed.
  case PM_EVT_SERVICE_CHANGED_IND_SENT:
  case PM_EVT_SERVICE_CHANGED_IND_CONFIRMED:
  default:
    break;
  }
}


/**@brief Function for handling advertising errors.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void
ble_advertising_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for performing a battery measurement, and update the Battery Level characteristic in the Battery Service.
 */
static void
battery_level_update(void)
{
  ret_code_t err_code;
  uint8_t  battery_level;

  battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

  err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
  if ((err_code != NRF_SUCCESS) &&
      (err_code != NRF_ERROR_BUSY) &&
      (err_code != NRF_ERROR_RESOURCES) &&
      (err_code != NRF_ERROR_FORBIDDEN) &&
      (err_code != NRF_ERROR_INVALID_STATE) &&
      (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
     )
  {
    APP_ERROR_HANDLER(err_code);
  }
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
static void
battery_level_meas_timeout_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  battery_level_update();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module.
 */
static void
timers_init(void)
{
  ret_code_t err_code;

  err_code = app_timer_init();
  APP_ERROR_CHECK(err_code);

  // Create battery timer.
  err_code = app_timer_create(&m_battery_timer_id,
      APP_TIMER_MODE_REPEATED,
      battery_level_meas_timeout_handler);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void
gap_params_init(void)
{
  ret_code_t              err_code;
  ble_gap_conn_params_t   gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode,
      (const uint8_t *)DEVICE_NAME,
      strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_HID_JOYSTICK);
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency     = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void
gatt_init(void)
{
  ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void
nrf_qwr_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Queued Write Module.
 */
static void
qwr_init(void)
{
  ret_code_t         err_code;
  nrf_ble_qwr_init_t qwr_init_obj = {0};

  qwr_init_obj.error_handler = nrf_qwr_error_handler;

  err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init_obj);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Device Information Service.
 */
static void
dis_init(void)
{
  ret_code_t       err_code;
  ble_dis_init_t   dis_init_obj;
  ble_dis_pnp_id_t pnp_id;

  pnp_id.vendor_id_source = PNP_ID_VENDOR_ID_SOURCE;
  pnp_id.vendor_id        = PNP_ID_VENDOR_ID;
  pnp_id.product_id       = PNP_ID_PRODUCT_ID;
  pnp_id.product_version  = PNP_ID_PRODUCT_VERSION;

  memset(&dis_init_obj, 0, sizeof(dis_init_obj));

  ble_srv_ascii_to_utf8(&dis_init_obj.manufact_name_str, MANUFACTURER_NAME);
  dis_init_obj.p_pnp_id = &pnp_id;

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&dis_init_obj.dis_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init_obj.dis_attr_md.write_perm);

  err_code = ble_dis_init(&dis_init_obj);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing Battery Service.
 */
static void
bas_init(void)
{
  ret_code_t     err_code;
  ble_bas_init_t bas_init_obj;

  memset(&bas_init_obj, 0, sizeof(bas_init_obj));

  bas_init_obj.evt_handler          = NULL;
  bas_init_obj.support_notification = true;
  bas_init_obj.p_report_ref         = NULL;
  bas_init_obj.initial_batt_level   = 100;

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.cccd_write_perm);
  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_char_attr_md.read_perm);
  BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init_obj.battery_level_char_attr_md.write_perm);

  BLE_GAP_CONN_SEC_MODE_SET_ENC_NO_MITM(&bas_init_obj.battery_level_report_read_perm);

  err_code = ble_bas_init(&m_bas, &bas_init_obj);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
  qwr_init();
  dis_init();
  bas_init();
}


/**@brief Function for initializing the battery sensor simulator.
 */
static void
sensor_simulator_init(void)
{
  m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
  m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
  m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
  m_battery_sim_cfg.start_at_max = true;

  sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void
conn_params_error_handler(uint32_t nrf_error)
{
  APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void
conn_params_init(void)
{
  ret_code_t             err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params                  = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count   = MAX_CONN_PARAM_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
  cp_init.disconnect_on_fail             = false;
  cp_init.evt_handler                    = NULL;
  cp_init.error_handler                  = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting timers.
 */
static void
timers_start(void)
{
  ret_code_t err_code;

  err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void
sleep_mode_enter(void)
{
  ret_code_t err_code;

  err_code = bsp_indication_set(BSP_INDICATE_IDLE);
  APP_ERROR_CHECK(err_code);

  // Prepare wakeup buttons.
  err_code = bsp_btn_ble_sleep_mode_prepare();
  APP_ERROR_CHECK(err_code);

  // Go to system-off mode (this function will not return; wakeup will cause a reset).
  err_code = sd_power_system_off();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void
on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
  ret_code_t err_code;

  switch (ble_adv_evt)
  {
  case BLE_ADV_EVT_DIRECTED_HIGH_DUTY:
    NRF_LOG_INFO("Directed advertising.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_DIRECTED);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_ADV_EVT_FAST:
    NRF_LOG_INFO("Fast advertising.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_ADV_EVT_SLOW:
    NRF_LOG_INFO("Slow advertising.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_ADV_EVT_FAST_WHITELIST:
    NRF_LOG_INFO("Fast advertising with whitelist.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_ADV_EVT_SLOW_WHITELIST:
    NRF_LOG_INFO("Slow advertising with whitelist.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_WHITELIST);
    APP_ERROR_CHECK(err_code);
    err_code = ble_advertising_restart_without_whitelist(&m_advertising);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_ADV_EVT_IDLE:
    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
    sleep_mode_enter();
    break;

  case BLE_ADV_EVT_WHITELIST_REQUEST:
    {
      ble_gap_addr_t whitelist_addrs[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
      ble_gap_irk_t  whitelist_irks[BLE_GAP_WHITELIST_ADDR_MAX_COUNT];
      uint32_t       addr_cnt = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;
      uint32_t       irk_cnt  = BLE_GAP_WHITELIST_ADDR_MAX_COUNT;

      err_code = pm_whitelist_get(whitelist_addrs, &addr_cnt,
          whitelist_irks,  &irk_cnt);
      APP_ERROR_CHECK(err_code);
      NRF_LOG_DEBUG("pm_whitelist_get returns %d addr in whitelist and %d irk whitelist",
          addr_cnt,
          irk_cnt);

      // Apply the whitelist.
      err_code = ble_advertising_whitelist_reply(&m_advertising,
          whitelist_addrs,
          addr_cnt,
          whitelist_irks,
          irk_cnt);
      APP_ERROR_CHECK(err_code);
    }
    break;

  case BLE_ADV_EVT_PEER_ADDR_REQUEST:
    {
      pm_peer_data_bonding_t peer_bonding_data;

      // Only Give peer address if we have a handle to the bonded peer.
      if (m_peer_id != PM_PEER_ID_INVALID)
      {

        err_code = pm_peer_data_bonding_load(m_peer_id, &peer_bonding_data);
        if (err_code != NRF_ERROR_NOT_FOUND)
        {
          APP_ERROR_CHECK(err_code);

          ble_gap_addr_t * p_peer_addr = &(peer_bonding_data.peer_ble_id.id_addr_info);
          err_code = ble_advertising_peer_addr_reply(&m_advertising, p_peer_addr);
          APP_ERROR_CHECK(err_code);
        }

      }
      break;
    }

  default:
    break;
  }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void
ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
  ret_code_t err_code;

  switch (p_ble_evt->header.evt_id)
  {
  case BLE_GAP_EVT_CONNECTED:
    NRF_LOG_INFO("Connected");
    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);

    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected");
    // LED indication will be changed when advertising starts.

    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    ble_joystick_disconnected();
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
      NRF_LOG_DEBUG("PHY update request.");
      ble_gap_phys_t const phys =
      {
        .rx_phys = BLE_GAP_PHY_AUTO,
        .tx_phys = BLE_GAP_PHY_AUTO,
      };
      err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
      APP_ERROR_CHECK(err_code);
    } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_HVN_TX_COMPLETE:
    ble_joystick_service_resume_tx();
    break;

  default:
    NRF_LOG_DEBUG("hit the default case.");
    // No implementation needed.
    break;
  }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void
ble_stack_init(void)
{
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for the Peer Manager initialization.
 */
static void
peer_manager_init(void)
{
  ble_gap_sec_params_t sec_param;
  ret_code_t           err_code;

  err_code = pm_init();
  APP_ERROR_CHECK(err_code);

  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_param.bond           = SEC_PARAM_BOND;
  sec_param.mitm           = SEC_PARAM_MITM;
  sec_param.lesc           = SEC_PARAM_LESC;
  sec_param.keypress       = SEC_PARAM_KEYPRESS;
  sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob            = SEC_PARAM_OOB;
  sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc  = 1;
  sec_param.kdist_own.id   = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id  = 1;

  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality.
 */
static void
advertising_init(void)
{
  ret_code_t             err_code;
  uint8_t                adv_flags;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  adv_flags                            = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
  init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance      = true;
  init.advdata.flags                   = adv_flags;
  init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

  init.config.ble_adv_whitelist_enabled          = true;
  init.config.ble_adv_directed_high_duty_enabled = true;
  init.config.ble_adv_directed_enabled           = false;
  init.config.ble_adv_directed_interval          = 0;
  init.config.ble_adv_directed_timeout           = 0;
  init.config.ble_adv_fast_enabled               = true;
  init.config.ble_adv_fast_interval              = APP_ADV_FAST_INTERVAL;
  init.config.ble_adv_fast_timeout               = APP_ADV_FAST_DURATION;
  init.config.ble_adv_slow_enabled               = true;
  init.config.ble_adv_slow_interval              = APP_ADV_SLOW_INTERVAL;
  init.config.ble_adv_slow_timeout               = APP_ADV_SLOW_DURATION;

  init.evt_handler   = on_adv_evt;
  init.error_handler = ble_advertising_error_handler;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for the Event Scheduler initialization.
 */
static void
scheduler_init(void)
{
  APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void
bsp_event_handler(bsp_event_t event)
{
  ret_code_t err_code;

  switch (event)
  {
  case BSP_EVENT_SLEEP:
    sleep_mode_enter();
    break;

  case BSP_EVENT_DISCONNECT:
    err_code = sd_ble_gap_disconnect(m_conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
      APP_ERROR_CHECK(err_code);
    }
    break;

  case BSP_EVENT_WHITELIST_OFF:
    if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
    {
      err_code = ble_advertising_restart_without_whitelist(&m_advertising);
      if (err_code != NRF_ERROR_INVALID_STATE)
      {
        APP_ERROR_CHECK(err_code);
      }
    }
    break;

  case BSP_EVENT_KEY_0:
    break;

  case BSP_EVENT_KEY_1:
    break;

  case BSP_EVENT_KEY_2:
    break;

  case BSP_EVENT_KEY_3:
    break;

  default:
    break;
  }
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void
buttons_leds_init(bool * p_erase_bonds)
{
  ret_code_t err_code;
  bsp_event_t startup_event;

  err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);

  APP_ERROR_CHECK(err_code);

  err_code = bsp_btn_ble_init(NULL, &startup_event);
  APP_ERROR_CHECK(err_code);

  *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void
log_init(void)
{
  ret_code_t err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void
power_management_init(void)
{
  ret_code_t err_code;
  err_code = nrf_pwr_mgmt_init();
  APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void
idle_state_handle(void)
{
  app_sched_execute();
  if (NRF_LOG_PROCESS() == false)
  {
    nrf_pwr_mgmt_run();
  }
}


/**@brief Function for application main entry.
 */
int
main(void)
{
  bool erase_bonds;

  // Initialize.
  log_init();
  timers_init();
  buttons_leds_init(&erase_bonds);
  power_management_init();
  ble_stack_init();
  scheduler_init();
  gap_params_init();
  gatt_init();
  advertising_init();

  services_init();
  ble_joystick_service_init();

  sensor_simulator_init();
  conn_params_init();
  peer_manager_init();

  // Start execution.
  NRF_LOG_INFO("HID Joystick started.");
  timers_start();
  advertising_start(erase_bonds);

  // Enter main loop.
  for (;;)
  {
    idle_state_handle();
  }
}
