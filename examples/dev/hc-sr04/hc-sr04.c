/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/gpio-hal.h"
#include "sys/etimer.h"
#include "lib/sensors.h"
#include "dev/button-hal.h"
#include "dev/leds.h"
#include "dev/ioc.h"

#include <stdio.h>
#include <inttypes.h>

/*---------------------------------------------------------------------------*/
static struct etimer et;
static uint8_t counter;

/*---------------------------------------------------------------------------*/
#define TRIG_PORT         GPIO_C_NUM
#define TRIG_PIN          3
#define TRIG_HAL_PIN      GPIO_PORT_PIN_TO_GPIO_HAL_PIN(TRIG_PORT, TRIG_PIN)

#define ECHO_PORT         GPIO_B_NUM
#define ECHO_PIN          2
#define ECHO_HAL_PIN      GPIO_PORT_PIN_TO_GPIO_HAL_PIN(ECHO_PORT, ECHO_PIN)


/*---------------------------------------------------------------------------*/
struct echo_pulse_timer_t
{
  rtimer_clock_t start_count;
  rtimer_clock_t end_count;
  uint16_t calc_dist;
};

static struct echo_pulse_timer_t echo_pulse_timer;

/*---------------------------------------------------------------------------*/
static uint16_t
get_distance_from_pulse_duration(long duration)
{
  float speed_of_sound = 0.034; /* cm / us */

  /* Divide by two because of pulse round trip */
  return (speed_of_sound / 2.0) * duration;
}

/*---------------------------------------------------------------------------*/
uint8_t get_echo_pin_value()
{
  return gpio_hal_arch_read_pin(GPIO_HAL_NULL_PORT, ECHO_HAL_PIN);
}

/*---------------------------------------------------------------------------*/
void test_pin_configs()
{
  /* TRIG pin */
  gpio_hal_pin_cfg_t pin_cfg = gpio_hal_arch_pin_cfg_get(GPIO_HAL_NULL_PORT, TRIG_HAL_PIN);
  printf("TRIG pin cfg PULL DOWN: %s\n", 0 != (pin_cfg & GPIO_HAL_PIN_CFG_PULL_DOWN) ? "YES" : "NO");
  printf("TRIG pin cfg PULL UP: %s\n", 0 != (pin_cfg & GPIO_HAL_PIN_CFG_PULL_UP) ? "YES" : "NO");

  /* ECHO pin */
  pin_cfg = gpio_hal_arch_pin_cfg_get(GPIO_HAL_NULL_PORT, ECHO_HAL_PIN);
  printf("ECHO pin cfg PULL DOWN: %s\n", 0 != (pin_cfg & GPIO_HAL_PIN_CFG_PULL_DOWN) ? "YES" : "NO");
  printf("ECHO pin cfg PULL UP: %s\n", 0 != (pin_cfg & GPIO_HAL_PIN_CFG_PULL_UP) ? "YES" : "NO");
}

/*---------------------------------------------------------------------------*/
static void
echo_pulse_interrupt_handler(gpio_hal_pin_mask_t pin_mask)
{
  rtimer_clock_t timer_diff;
  long pulse_duration;
  float distance;

  if(pin_mask == gpio_hal_pin_to_mask(ECHO_HAL_PIN)) {
  
    /* Start measuring pulse duration on rising edge, stop on falling edge */
    if(1 == get_echo_pin_value()) {
      /* Echo pulse is high */
      
      /* TODO:
	 Register timer that handles case where the echo pulse is not
	 returning. Some docs talk about an ok timeout value to be 28
	 or 32 ms.
      */
      
      echo_pulse_timer.start_count = RTIMER_NOW();
      leds_off(LEDS_ALL);
      
    } else {
      /* Echo pulse is high */
      
      echo_pulse_timer.end_count = RTIMER_NOW();
      timer_diff = echo_pulse_timer.end_count - echo_pulse_timer.start_count;

      /* Calculate duration of echo pulse, in usec */
      pulse_duration = timer_diff * (1000000 / RTIMER_SECOND);
      //printf("Pulse duration [usec]: %ld\n", pulse_duration);

      distance = get_distance_from_pulse_duration(pulse_duration);
      echo_pulse_timer.calc_dist = distance;
      //printf("Distance [m]: %f\n", distance);
      
      if (distance > 100) {
	leds_single_on(LEDS_LED3); /* Green */
      } else if (distance > 50) {
	leds_single_on(LEDS_LED2); /* Yellow */
      } else {
	leds_single_on(LEDS_LED1); /* Red */
      }
    }

  }
}

/*---------------------------------------------------------------------------*/
static gpio_hal_event_handler_t echo_pulse_handler = {
  .next = NULL,
  .handler = echo_pulse_interrupt_handler,
  .pin_mask = gpio_hal_pin_to_mask(ECHO_HAL_PIN),
};

/*---------------------------------------------------------------------------*/
PROCESS(hc-sr04, "HC-SR04");
AUTOSTART_PROCESSES(&hc-sr04);

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(hc-sr04, ev, data)
{
  PROCESS_BEGIN();

  counter = 0;

  etimer_set(&et, CLOCK_SECOND / 5);

  /* Configure TRIG pin (output) */
  gpio_hal_arch_pin_set_output(GPIO_HAL_NULL_PORT, TRIG_HAL_PIN);  
  gpio_hal_pin_cfg_t trig_pin_cfg;
  trig_pin_cfg = gpio_hal_arch_pin_cfg_get(GPIO_HAL_NULL_PORT, TRIG_HAL_PIN);
  trig_pin_cfg |= GPIO_HAL_PIN_CFG_PULL_DOWN;
  trig_pin_cfg &= ~GPIO_HAL_PIN_CFG_PULL_UP;
  gpio_hal_arch_pin_cfg_set(GPIO_HAL_NULL_PORT, TRIG_HAL_PIN, trig_pin_cfg);  

  /* Configure ECHO pin (input) */
  gpio_hal_arch_pin_set_input(GPIO_HAL_NULL_PORT, ECHO_HAL_PIN);
  gpio_hal_pin_cfg_t echo_pin_cfg;
  echo_pin_cfg = gpio_hal_arch_pin_cfg_get(GPIO_HAL_NULL_PORT, ECHO_HAL_PIN);
  echo_pin_cfg |= GPIO_HAL_PIN_CFG_PULL_DOWN;
  echo_pin_cfg &= ~GPIO_HAL_PIN_CFG_PULL_UP;
  echo_pin_cfg |= GPIO_HAL_PIN_CFG_EDGE_BOTH;
  echo_pin_cfg |= GPIO_HAL_PIN_CFG_INT_ENABLE;
  gpio_hal_arch_pin_cfg_set(GPIO_HAL_NULL_PORT, ECHO_HAL_PIN, echo_pin_cfg);

  //test_pin_configs();
    
  gpio_hal_register_handler(&echo_pulse_handler);
  gpio_hal_arch_interrupt_enable(GPIO_HAL_NULL_PORT, ECHO_HAL_PIN);
  
  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER && data == &et) {

      //printf("Triggering\n");
      //leds_single_toggle(LEDS_LED4);

      /* Trigger HC-SR04 */
      gpio_hal_arch_set_pin(GPIO_HAL_NULL_PORT, TRIG_HAL_PIN);
      clock_delay_usec(50);
      gpio_hal_arch_clear_pin(GPIO_HAL_NULL_PORT, TRIG_HAL_PIN);

#if 0
      //printf("ECHO pin is: %d\n", get_echo_pin_value());
      uint16_t dist = echo_pulse_timer.calc_dist;
      printf("Distance is: %d cm\n", dist);
#endif
      
      counter++;
      etimer_set(&et, CLOCK_SECOND);
    } else if(ev == button_hal_release_event) {
      printf("Button release event %s\n",
             BUTTON_HAL_GET_DESCRIPTION((button_hal_button_t *)data));
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
