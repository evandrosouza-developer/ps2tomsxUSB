#ifndef msxmap_h
#define msxmap_h

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>

//Use Tab width=2


class msxmap
{
private:


public:
	void msx_interface_setup(void);
	void msxqueuekeys(void);
	void convert2msx(void);
	//void msx_dispatch(volatile uint16_t linhavarrida, volatile bool shiftstate);
	void msx_dispatch(void);
	bool available_msx_disp_keys_queue_buffer(void);
	uint8_t put_msx_disp_keys_queue_buffer(uint8_t);
	uint8_t get_msx_disp_keys_queue_buffer(void);
	void compute_x_bits_and_check_interrupt_stuck ( 
					volatile uint8_t y_local, volatile uint8_t x_local, volatile bool x_local_setb);
};


#endif
