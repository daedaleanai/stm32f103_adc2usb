/*
 */
#include "stm32f103_md.h"

#include "clock.h"
#include "gpio2.h"
#include "serial.h"
#include "stb_sprintf.h"
#include "usb/cdc.h"
#include "usb/usbd.h"

/*
    STM32F013CB (LQFP48/LQFP48) Pin Assignments:

    Pin   Function     DIR   Electrical     Connected to
    ---   ---------    ---   ----------- ---------------------------------------
    PA0:7 ADC 0:7      in    Analog         Analog input 0-7
    PB0:1 ADC 8:9      in    Analog         Analog input 8,9

    PA9   USART1 TX    out   AF_PP 10MHz
    PA10  USART1 RX    in    PullUp
    PA11  USB D-       automatic
    PA12  USB D+       automatic
    PA13  SWDIO        in/out               ST-Link programmer
    PA14  SWCLK        in/out               ST-Link programmer

    PC13  LED0         out   OUT_OD 2MHz    On-board yellow LED
*/

/* clang-format off */
enum {
  ADC0_7_PIN    = PA0 | PA1 | PA2 | PA3 | PA4 | PA5 | PA6 | PA7,
  ADC8_9_PIN    = PB0 | PB1,
	USART1_TX_PIN = PA9,
	USART1_RX_PIN = PA10,
	LED0_PIN      = PC13,
};

static struct gpio_config_t {
    enum GPIO_Pin  pins;
    enum GPIO_Conf mode;
} pin_cfgs[] = {
    {PAAll, Mode_IN}, // reset
    {PBAll, Mode_IN}, // reset
    {PCAll, Mode_IN}, // reset
    {ADC0_7_PIN, Mode_INA},
    {ADC8_9_PIN, Mode_INA},    
    {USART1_TX_PIN, Mode_AF_PP_50MHz},
    {USART1_RX_PIN, Mode_IPU},
    {LED0_PIN, Mode_Out_OD_2MHz},
    {0, 0}, // sentinel
};

enum { IRQ_PRIORITY_GROUPING = 5 }; // prio[7:6] : 4 groups,  prio[5:4] : 4 subgroups
struct {
    enum IRQn_Type irq;
    uint8_t        group, sub;
} irqprios[] = {
    {SysTick_IRQn,         0, 0},
    {DMA1_Channel1_IRQn,   1, 0},
    {ADC1_2_IRQn,          1, 1},
    {TIM3_IRQn,            1, 2},
    {USART1_IRQn,          2, 0},
    {USB_LP_CAN1_RX0_IRQn, 3, 0},
    {None_IRQn, 0xff, 0xff},
};
/* clang-format on */

static inline void led0_on(void) { digitalLo(LED0_PIN); }
static inline void led0_off(void) { digitalHi(LED0_PIN); }
static inline void led0_toggle(void) { digitalToggle(LED0_PIN); }

static struct Ringbuffer usart1tx;

static volatile uint64_t adctrig = 0; // time last ADC trigger
static volatile uint64_t adcdone = 0; // time of last DMA transfer completion
static volatile uint64_t adccount = 0;
static volatile uint32_t adcdata[16]; // up to 16 samples from scan
static volatile uint16_t jadcdata[4]; // up to 4 injected channels

// ADC1 DMA Transfer Complete
void DMA1_Channel1_IRQ_Handler(void) {
  if ((DMA1.ISR & DMA_ISR_TCIF1) == 0)
    return;
  adcdone = cycleCount();
  ++adccount;
  DMA1.IFCR = DMA_ISR_TCIF1;
}

// injected conversion done
void ADC1_2_IRQ_Handler(void) {
  if ((ADC1.SR & ADC_SR_JEOC) == 0)
    return;
  jadcdata[3] = ADC1.JDR4;
  jadcdata[2] = ADC1.JDR3;
  jadcdata[1] = ADC1.JDR2;
  jadcdata[0] = ADC1.JDR1;
  ADC1.SR &= ~ADC_SR_JEOC;
}

// Trigger of ADC1 scan
void TIM3_IRQ_Handler(void) {
  if ((TIM3.SR & TIM_SR_UIF) == 0)
    return;
  adctrig = cycleCount();
  uint32_t corr = TIM3.CNT;
  corr *= TIM3.PSC;
  adctrig -= corr; // correct for if the irq was delayed
  TIM3.SR &= ~TIM_SR_UIF;
}

static const struct usb_device_descriptor dev = {
    .bLength = USB_DT_DEVICE_SIZE,
    .bDescriptorType = USB_DT_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = USB_CLASS_CDC,
    .bDeviceSubClass = 0,
    .bDeviceProtocol = 0,
    .bMaxPacketSize0 = 64,
    .idVendor = 0x0483,
    .idProduct = 0x5740,
    .bcdDevice = 0x0200,
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1,
};

#if 0
/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x83,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 16,
    .bInterval = 255,
}};
#endif

static const struct usb_endpoint_descriptor data_endp[] = {
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x01,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 64,
        .bInterval = 1,
    },
    {
        .bLength = USB_DT_ENDPOINT_SIZE,
        .bDescriptorType = USB_DT_ENDPOINT,
        .bEndpointAddress = 0x82,
        .bmAttributes = USB_ENDPOINT_ATTR_BULK,
        .wMaxPacketSize = 64,
        .bInterval = 1,
    }};

static const struct {
  struct usb_cdc_header_descriptor header;
//  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
    .header =
        {
            .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
            .bcdCDC = 0x0110,
        },
    // .call_mgmt =
    //     {
    //         .bFunctionLength =
    //             sizeof(struct usb_cdc_call_management_descriptor),
    //         .bDescriptorType = CS_INTERFACE,
    //         .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
    //         .bmCapabilities = 0,
    //         .bDataInterface = 1,
    //     },
    .acm =
        {
            .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
            .bDescriptorType = CS_INTERFACE,
            .bDescriptorSubtype = USB_CDC_TYPE_ACM,
            .bmCapabilities = 0,
        },
    .cdc_union = {
        .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
        .bDescriptorType = CS_INTERFACE,
        .bDescriptorSubtype = USB_CDC_TYPE_UNION,
        .bControlInterface = 0,
        .bSubordinateInterface0 = 1,
    }};

static const struct usb_interface_descriptor comm_iface[] = {
    {.bLength = USB_DT_INTERFACE_SIZE,
     .bDescriptorType = USB_DT_INTERFACE,
     .bInterfaceNumber = 0,
     .bAlternateSetting = 0,
//     .bNumEndpoints = 1,
     .bNumEndpoints = 0,
     .bInterfaceClass = USB_CLASS_CDC,
     .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
//     .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
     .bInterfaceProtocol = USB_CDC_PROTOCOL_NONE,
     .iInterface = 0,
//     .endpoint = comm_endp,
     .extra = &cdcacm_functional_descriptors,
     .extralen = sizeof(cdcacm_functional_descriptors)}};

static const struct usb_interface_descriptor data_iface[] = {{
    .bLength = USB_DT_INTERFACE_SIZE,
    .bDescriptorType = USB_DT_INTERFACE,
    .bInterfaceNumber = 1,
    .bAlternateSetting = 0,
    .bNumEndpoints = 2,
    .bInterfaceClass = USB_CLASS_DATA,
    .bInterfaceSubClass = 0,
    .bInterfaceProtocol = 0,
    .iInterface = 0,
    .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
                                                  .num_altsetting = 1,
                                                  .altsetting = comm_iface,
                                              },
                                              {
                                                  .num_altsetting = 1,
                                                  .altsetting = data_iface,
                                              }};

static const struct usb_config_descriptor config = {
    .bLength = USB_DT_CONFIGURATION_SIZE,
    .bDescriptorType = USB_DT_CONFIGURATION,
    .wTotalLength = 0,
    .bNumInterfaces = 2,
    .bConfigurationValue = 1,
    .iConfiguration = 0,
    .bmAttributes = 0x80,
    .bMaxPower = 0x32,
    .interface = ifaces,
};

static const char *usb_strings[] = {
    "Daedalean",
    "ADC2USB",
    "ADC",
};

static usbd_device *usbdev;
uint8_t usbd_control_buffer[128]; // Buffer to be used for control requests.
uint16_t usb_active = 0;

void USB_LP_CAN1_RX0_IRQ_Handler(void) { usbd_poll(usbdev); }

static int cdcacm_control_request(
    usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len,
    void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) {
  (void)complete;
  (void)buf;
  (void)usbd_dev;

  switch (req->bRequest) {
  case USB_CDC_REQ_SET_LINE_CODING:
    if (*len < sizeof(struct usb_cdc_line_coding))
      break;
    // fallthrough
  case USB_CDC_REQ_SET_CONTROL_LINE_STATE:
    return USBD_REQ_HANDLED;
  }
  return USBD_REQ_NOTSUPP;;
}

// echo incoming string to console, TODO: do sth more useful
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep) {
  (void)ep;
  char buf[64];
  int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);
  if (len) {
    serial_printf(&USART1, "ep%d: %*s\n", ep, len, buf);
//  usbd_ep_write_packet(usbdev, 0x82, buf, len);
  }
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue) {
  usb_active = wValue;

  usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(
      usbd_dev, USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
      USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT, cdcacm_control_request);
}

int main(void) {

  uint8_t rf = (RCC.CSR >> 24) & 0xfc;
  RCC.CSR |= RCC_CSR_RMVF; // Set RMVF bit to clear the reset flags

  SysTick_Config(1U << 24); // tick at 72Mhz/2^24 = 4.2915 HZ

  NVIC_SetPriorityGrouping(IRQ_PRIORITY_GROUPING);
  for (int i = 0; irqprios[i].irq != None_IRQn; i++) {
    NVIC_SetPriority(irqprios[i].irq,
                     NVIC_EncodePriority(IRQ_PRIORITY_GROUPING,
                                         irqprios[i].group, irqprios[i].sub));
  }

  RCC.APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC.APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN;
  RCC.APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_ADC2EN;
  RCC.APB1ENR |= RCC_APB1ENR_TIM3EN | RCC_APB1ENR_USBEN;
  RCC.AHBENR  |= RCC_AHBENR_DMA1EN;
  RCC.CFGR    |= RCC_CFGR_ADCPRE_DIV6; // ADC clock 72/6 MHz = 12 Mhz, must be < 14MHz

  delay(10); // let all clocks and peripherals start up

  for (const struct gpio_config_t *p = pin_cfgs; p->pins; ++p) {
    gpioConfig(p->pins, p->mode);
  }

  gpioLock(PAAll);
  gpioLock(PBAll);
  gpioLock(PCAll);

  led0_off();

  serial_init(&USART1, 921600, &usart1tx);

  serial_printf(&USART1, "ADC2USB:%s\n", __REVISION__);
  serial_printf(&USART1, "CPUID:%08lx\n", SCB.CPUID);
  serial_printf(&USART1, "DEVID:%08lx:%08lx:%08lx\n", UNIQUE_DEVICE_ID[2],
                UNIQUE_DEVICE_ID[1], UNIQUE_DEVICE_ID[0]);
  serial_printf(&USART1, "RESET:%02x%s%s%s%s%s%s\n", rf,
                rf & 0x80 ? " LPWR" : "", rf & 0x40 ? " WWDG" : "",
                rf & 0x20 ? " IWDG" : "", rf & 0x10 ? " SFT" : "",
                rf & 0x08 ? " POR" : "", rf & 0x04 ? " PIN" : "");
  serial_wait(&USART1);

  usbdev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings,
                     sizeof(usb_strings) / sizeof(usb_strings[0]),
                     usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbdev, cdcacm_set_config);

  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);

  ADC1.CR2 |= ADC_CR2_ADON; // first wake up
  ADC2.CR2 |= ADC_CR2_ADON;
  delay(10);
  ADC1.CR2 |= ADC_CR2_CAL; // start calibrate
  ADC2.CR2 |= ADC_CR2_CAL;
  while (ADC1.CR2 & ADC_CR2_CAL) // wait until done
    __NOP();
  while (ADC2.CR2 & ADC_CR2_CAL)
    __NOP();

  // scan adc0..9 triggered by timer 3, using dma.
  // use 13.5 cycles sample time +12.5 = 26 cycles at 12Mhz = 2.16667us per
  // sample so 21.6667 us per cycle, triggered every 10ms
  ADC1.CR1 |= ADC_CR1_SCAN | (0b0110 << 16); // simultaneous regular dual mode
  ADC1.CR2 |= ADC_CR2_EXTTRIG | (4 << 17) | ADC_CR2_DMA; // Trigger from Timer 3 TRGO event.
  ADC1.SMPR2 = 0x3fffffff;   // SMPR0..9 to  111: 239.5 cycles
  ADC1.SQR1 = (5 - 1) << 20; // 5 conversions
  ADC1.SQR2 = 0;
  ADC1.SQR3 = 0 | (2 << 5) | (4 << 10) | (6 << 15) | (8 << 20); // channels 0 2 4 6 8 in that order

  // ADC2 is slave
  ADC2.CR1 |= ADC_CR1_SCAN;
  ADC2.CR2 |= ADC_CR2_EXTTRIG | (7 << 17); // Trigger must be set to sw.
  ADC2.SMPR2 = ADC1.SMPR2;                 // must be set to same
  ADC2.SQR1 = ADC1.SQR1 & (0xf << 20);
  ADC2.SQR2 = 0;
  ADC2.SQR3 = 1 | (3 << 5) | (5 << 10) | (7 << 15) | (9 << 20); // channels 1 3 5 7 9 in that order

  // injected sequence: Vref and Temp, automatically after regular scan
  ADC1.CR1 |= ADC_CR1_JAUTO | ADC_CR1_JEOCIE; // also generate irq when done
  ADC1.CR2 |= ADC_CR2_TSVREFE;                // enable Vref and Temp
  ADC1.SMPR1 = (7 << 18) | (7 << 21); // SMPR16,17 to 0b111 ->  239.5 cycles @ 12MHz = >20us
  ADC1.JSQR  = (1 << 20) | (17 << 15) | (16 << 10); // sample chan 16, 17 into jdata 4,3
  NVIC_EnableIRQ(ADC1_2_IRQn);

  ADC1.CR2 |= ADC_CR2_ADON; // up & go.
  ADC2.CR2 |= ADC_CR2_ADON; // up & go.

  DMA1_Channel1.CCR   = (2 << 10) | (2 << 8) | DMA_CCR1_MINC | DMA_CCR2_CIRC | DMA_CCR1_TCIE;
  DMA1_Channel1.CPAR  = (uint32_t)&ADC1.DR;
  DMA1_Channel1.CMAR  = (uint32_t)&adcdata[0];
  DMA1_Channel1.CNDTR = ((ADC1.SQR1 >> 20) & 0xf) + 1; // number of conversions in scan
  DMA1_Channel1.CCR |= DMA_CCR1_EN;
  NVIC_EnableIRQ(DMA1_Channel1_IRQn);

  // enable 10Hz TIM3 to trigger ADC
  TIM3.DIER |= TIM_DIER_UIE;
  TIM3.PSC = 720 - 1;   // 72MHz / 720 = 100KHz
  TIM3.ARR = 10000 - 1; // 100KHz/10000 = 10Hz
  TIM3.CR2 |= (2 << 4); // TRGO is update event
  TIM3.CR1 |= TIM_CR1_CEN;
  NVIC_EnableIRQ(TIM3_IRQn);

  // ADC/DMA errors will cause the watchdog to cease being triggered
  // Initialize the independent watchdog
  while (IWDG.SR != 0)
   __NOP();

  IWDG.KR  = 0x5555; // enable watchdog config
  IWDG.PR  = 0;      // prescaler /4 -> 10kHz
  IWDG.RLR = 0xFFF;  // count to 4096 -> 409.6ms timeout
  IWDG.KR  = 0xcccc; // start watchdog countdown

  uint64_t lastreport = 0;
  int nconv = ((ADC1.SQR1 >> 20) & 0xf) + 1; // number of conversions

  for (;;) {
    __enable_irq();

    serial_wait(&USART1);

    __WFI(); // wait for interrupt to change the state of any of the subsystems
    __disable_irq();

    if (lastreport == adccount)
      continue;

    int64_t skip = adccount - lastreport;
    lastreport = adccount;

    // if this loop ran too slowly compared to TIM3 and the ADC, we'll notice here
    if (skip > 1) {
      serial_printf(&USART1, "### skipped %lld\n", skip);
      led0_on();
    }

    // convert to mV with vref

    if (usb_active) {
      char buf[64];
      size_t len = 0;
      len += stbsp_snprintf(buf + len, sizeof(buf) - len, "%lli", adccount);
      uint32_t vref = jadcdata[1];
      if (vref == 0) vref = 1200 * 4096 / 3300;
      for (int i = 0; i < nconv; i++) {
        uint32_t v0 = (adcdata[i] & 0xfff) * 1200 / vref;
        uint32_t v1 = (adcdata[i] >> 16) * 1200 / vref;

        len += stbsp_snprintf(buf + len, sizeof(buf) - len, " %4d %4d", v0, v1);
      }
      len += stbsp_snprintf(buf + len, sizeof(buf) - len, "\n");

      usbd_ep_write_packet(usbdev, 0x82, buf, len);
    }

    if ((adccount % 10) == 0) {
      // The JADC will probably be one cycle old, but we can live with that.
      // Vref should be around 1.2V, full scale 4096 is about 3.3V or 0.8mV/lsb
      // Vsense = 1.43V + (T-25C) * 4.3mV * T / C  or 5.3lsb/degree
      int t1 = 2500 + 1000 * (jadcdata[0] - 1775) / 53;
      int t2 = t1 / 100;
      t1 %= 100;
      serial_printf(&USART1, "# Temp: %d.%02d ℃ Vref: %d [lsb]\n", t2, t1, jadcdata[1]);
    }

    IWDG.KR = 0xAAAA; // kick the watchdog

  } // forever

  return 0;
}
