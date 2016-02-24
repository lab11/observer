
#include "contiki.h"
#include "sys/etimer.h"
#include "dev/leds.h"
#include "mpu9250.h"
#include "lps331ap.h"

static struct etimer periodic_timer_red;
static struct etimer periodic_timer_green;
static struct etimer periodic_timer_blue;

void accel_irq(uint8_t port, uint8_t pin){
    leds_toggle(LEDS_GREEN);
}

void print_float(float val) {
    int32_t val_int = (int32_t)val;
    uint32_t val_dec = (uint32_t)((val - (float)val_int)*1000);

    printf("%d.%d", val_int, val_dec);
}

/*---------------------------------------------------------------------------*/
PROCESS(blink_process, "Blink");
AUTOSTART_PROCESSES(&blink_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(blink_process, ev, data) {

	PROCESS_BEGIN();

	etimer_set(&periodic_timer_red, CLOCK_SECOND);
	//etimer_set(&periodic_timer_green, CLOCK_SECOND/2);
	//etimer_set(&periodic_timer_blue, CLOCK_SECOND/4);
    lps331ap_init();
    mpu9250_init();
    clock_delay_usec(50000);
    clock_delay_usec(50000);
    ak8963_init(0x06);
    //mpu9250_motion_interrupt_init(20, 7, accel_irq);
    
    //mpu9250_writeByte(MPU9250_INT_PIN_CFG, 0x22);
    int16_t x, y, z;
    uint8_t wia;

    int8_t ret_val;
    uint8_t regs[6];
    float x_mag;
    float y_mag;
    float z_mag;
    while(1) {
		PROCESS_YIELD();
        
        //int16_t value;

        x = mpu9250_readSensor(MPU9250_ACCEL_XOUT_L, MPU9250_ACCEL_XOUT_H);
        y = mpu9250_readSensor(MPU9250_ACCEL_YOUT_L, MPU9250_ACCEL_YOUT_H);
        z = mpu9250_readSensor(MPU9250_ACCEL_ZOUT_L, MPU9250_ACCEL_ZOUT_H);
        //printf("Z val %d\n", z);
        //wia = ak8963_readWIA();
        //printf("wia: %d\n", wia);
        
        //ak8963_readMultiple(AK8963_WIA, 2, regs);
        //printf("[0]: %d\n[1]: %d\n", *(regs), *(regs+1));

        ret_val = ak8963_read_Mag(regs);
        if (ret_val == -1) {
            printf("Data not ready\n");
        } else if (ret_val == -2) {
            printf("Data overflow\n");
        } else {
            x_mag = (float)(int16_t)((regs[1] << 8)|regs[0]) * AK8963_ADJUST_X;
            y_mag = (float)(int16_t)((regs[3] << 8)|regs[2]) * AK8963_ADJUST_Y;
            z_mag = (float)(int16_t)((regs[5] << 8)|regs[4]) * AK8963_ADJUST_Z;
            //printf("ADJUST X: %f, Y: %f, Z: %f\n", AK8963_ADJUST_X, AK8963_ADJUST_Y, AK8963_ADJUST_Z);
            //printf("X: %a\nY: %a\nZ: %a\n\n", x_mag, y_mag, z_mag);
            //printf("ADJ X: "); print_float(AK8963_ADJUST_X); printf("\n");
            //printf("ADJ Y: "); print_float(AK8963_ADJUST_Y); printf("\n");
            //printf("ADJ Z: "); print_float(AK8963_ADJUST_Z); printf("\n");

            printf("X: "); print_float(x_mag); printf("\n");
            printf("Y: "); print_float(y_mag); printf("\n");
            printf("Z: "); print_float(z_mag); printf("\n");
            
        }

        leds_toggle(LEDS_RED);
        etimer_restart(&periodic_timer_red);

		/*if (etimer_expired(&periodic_timer_red)) {
			leds_toggle(LEDS_RED);
			etimer_restart(&periodic_timer_red);
		} else if (etimer_expired(&periodic_timer_green)) {
			leds_toggle(LEDS_GREEN);
			etimer_restart(&periodic_timer_green);
		} else if (etimer_expired(&periodic_timer_blue)) {
			leds_toggle(LEDS_BLUE);
			etimer_restart(&periodic_timer_blue);
		}*/
	}

	PROCESS_END();
}
