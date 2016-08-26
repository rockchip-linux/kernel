#ifndef __GPIO_DETECTION_H
#define __GPIO_DETECTION_H

#define GPIO_EVENT		1

/*
 * gpio event
 *  @val: 0 event active, 1 event over
 *  @name: event name
*/
struct gpio_event {
	int val;
	const char *name;
};

int gpio_det_register_notifier(struct notifier_block *nb);
int gpio_det_unregister_notifier(struct notifier_block *nb);
int gpio_det_notifier_call_chain(unsigned long val, void *v);

#endif
