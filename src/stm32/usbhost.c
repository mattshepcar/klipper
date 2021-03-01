#include "usbh_cdc.h"
#include "sched.h"
#include "command.h" // DECL_CONSTANT_STR
#include "board/internal.h" // GPIO
#include "board/armcm_boot.h" // armcm_enable_irq
#include "board/armcm_timer.h" // udelay
#include "board/misc.h" // crc16_ccitt

USBH_HandleTypeDef hUsbHostFS;

DECL_CONSTANT_STR("RESERVE_PINS_USB", "PA11,PA12");

extern HCD_HandleTypeDef hhcd_USB_OTG_FS;

static uint8_t next_sequence = 0;
static uint8_t transmit_buf[128], tx_read_pos, tx_write_pos;
static uint8_t receive_buf[64], input_pos;
static uint8_t usb_connected = 0;

void MX_DriverVbusFS(uint8_t state)
{
}
void Error_Handler(void)
{
}
void HAL_Delay(uint32_t ms)
{
    udelay(ms * 1000);
}
void OTG_FS_IRQHandler(void)
{
    HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
    sched_wake_tasks();
}
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
  switch(id)
  {
  case HOST_USER_DISCONNECTION:
    usb_connected = 0;
    break;
  case HOST_USER_CLASS_ACTIVE:
    usb_connected = 1;
    input_pos = 0;
    tx_read_pos = 0;
    tx_write_pos = 0;
    next_sequence = 0;
    // start listening
    USBH_CDC_ReceiveCallback(&hUsbHostFS, 0);
    break;
  default:
    break;
  }
}
void usb_host_init(void)
{
    armcm_enable_irq(OTG_FS_IRQHandler, OTG_FS_IRQn, 1);
    gpio_peripheral(GPIO('A', 11), GPIO_FUNCTION(10), 0);
    gpio_peripheral(GPIO('A', 12), GPIO_FUNCTION(10), 0);
    USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS);
    USBH_RegisterClass(&hUsbHostFS, USBH_CDC_CLASS);
    USBH_Start(&hUsbHostFS);
}
DECL_INIT(usb_host_init);

void usb_host_task(void)
{
    USBH_Process(&hUsbHostFS);
}
DECL_TASK(usb_host_task);

void command_redirect(uint8_t *buf, uint8_t msglen)
{
  if (!usb_connected)
      return;
  if (msglen < MESSAGE_MIN)
      return;
  if (tx_write_pos + msglen > sizeof(transmit_buf))
      return;
  buf[MESSAGE_POS_LEN] = msglen;
  buf[MESSAGE_POS_SEQ] = next_sequence;
  uint16_t crc = crc16_ccitt(buf, msglen - MESSAGE_TRAILER_SIZE);
  buf[msglen - MESSAGE_TRAILER_CRC + 0] = crc >> 8;
  buf[msglen - MESSAGE_TRAILER_CRC + 1] = crc;
  buf[msglen - MESSAGE_TRAILER_SYNC] = MESSAGE_SYNC;
  next_sequence = ((next_sequence + 1) & MESSAGE_SEQ_MASK) | MESSAGE_DEST;
  memcpy(&transmit_buf[tx_write_pos], buf, msglen);
  tx_write_pos += msglen;
  // if no transfer is in progress then kick one off
  if (tx_read_pos == 0)
      USBH_CDC_TransmitCallback(&hUsbHostFS);
}

void USBH_CDC_TransmitCallback(USBH_HandleTypeDef *phost)
{
    int bytes_to_send = tx_write_pos - tx_read_pos;
    if (tx_read_pos != 0)
    {
        // move any remaining data back to the beginning of the buffer
        memmove(transmit_buf, transmit_buf + tx_read_pos, bytes_to_send);
        tx_write_pos = bytes_to_send;
        tx_read_pos = 0;
    }
    if (bytes_to_send) {
        if (USBH_CDC_Transmit(&hUsbHostFS, transmit_buf,
                              bytes_to_send) == USBH_OK) {
            tx_read_pos = tx_write_pos;
            sched_wake_tasks();
        }
    }
}

enum { CF_NEED_SYNC=1<<0, CF_NEED_VALID=1<<1 };
static int_fast8_t
usb_host_find_block(uint8_t *buf, uint_fast8_t buf_len, uint_fast8_t *pop_count)
{
    static uint8_t sync_state;
    if (buf_len && sync_state & CF_NEED_SYNC)
        goto need_sync;
    if (buf_len < MESSAGE_MIN)
        goto need_more_data;
    uint_fast8_t msglen = buf[MESSAGE_POS_LEN];
    if (msglen < MESSAGE_MIN || msglen > MESSAGE_MAX)
        goto error;
    if (buf_len < msglen)
        goto need_more_data;
    if (buf[msglen-MESSAGE_TRAILER_SYNC] != MESSAGE_SYNC)
        goto error;
    uint16_t msgcrc = ((buf[msglen-MESSAGE_TRAILER_CRC] << 8)
                       | buf[msglen-MESSAGE_TRAILER_CRC+1]);
    uint16_t crc = crc16_ccitt(buf, msglen-MESSAGE_TRAILER_SIZE);
    if (crc != msgcrc)
        goto error;
    sync_state &= ~CF_NEED_VALID;
    *pop_count = msglen;
    return 1;

need_more_data:
    *pop_count = 0;
    return 0;
error:
    if (buf[0] == MESSAGE_SYNC) {
        *pop_count = 1;
        return -1;
    }
    sync_state |= CF_NEED_SYNC;
need_sync: ;
    // Discard bytes until next SYNC found
    uint8_t *next_sync = memchr(buf, MESSAGE_SYNC, buf_len);
    if (next_sync) {
        sync_state &= ~CF_NEED_SYNC;
        *pop_count = next_sync - buf + 1;
    } else {
        *pop_count = buf_len;
    }
    sync_state |= CF_NEED_VALID;
    return -1;
}

void USBH_CDC_ReceiveCallback(USBH_HandleTypeDef *phost, int length)
{
    input_pos += length;
    uint_fast8_t pos = 0;
    for(;;) {
      uint_fast8_t pop_count;
      int_fast8_t ret = usb_host_find_block(&receive_buf[pos],
                                      input_pos, &pop_count);
      if (ret == 0)
        break;
      if (ret > 0) {
        if (next_sequence == 0)
          next_sequence = receive_buf[MESSAGE_POS_SEQ];
        // forward packet to klippy
        console_send_raw(0x20, &receive_buf[pos + MESSAGE_HEADER_SIZE],
            pop_count - MESSAGE_HEADER_SIZE - MESSAGE_TRAILER_SIZE);
      }
      pos += pop_count;
      input_pos -= pop_count;
    }
    if (input_pos > 0) {
      // move remaining data back to beginning of buffer
      memmove(&receive_buf[0], &receive_buf[pos], input_pos);
    }
    // request more data
    int space_left = sizeof(receive_buf) - input_pos;
    if (space_left > 0) {
        USBH_CDC_Receive(&hUsbHostFS, &receive_buf[input_pos],
                        space_left);
        sched_wake_tasks();
    }
}
