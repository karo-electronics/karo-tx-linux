#ifndef LINUX_GPIO_SWITCH_H
#define LINUX_GPIO_SWITCH_H

enum gpio_sw_flags {
	GPIO_SW_ACTIVE_LOW	= (1 << 0),
	GPIO_SW_SHARED		= (1 << 1),
	GPIO_SW_SUSPEND_ON	= (1 << 2),
	GPIO_SW_SUSPEND_OFF	= (1 << 3),
	GPIO_SW_RESUME_ON	= (1 << 4),
	GPIO_SW_RESUME_OFF	= (1 << 5),
};

struct gpio_sw {
	struct list_head list;
	struct device *parent;
	const char *id;
	const char *label;
	int gpio;
	unsigned flags;
	int use_count;
	int enable_count;
	unsigned state:1;
};

enum gpio_sw_initstate {
	GPIO_SW_INIT_ACTIVE = 1,
	GPIO_SW_INIT_INACTIVE = 2,
};

struct gpio_sw_platform_data {
	const char *label;
	int gpio;
	enum gpio_sw_flags flags;
	int init_state;
};

extern int gpio_switch_register(struct device *parent, const char *id, int gpio,
			 enum gpio_sw_flags flags);

extern int gpio_switch_unregister(struct gpio_sw *sw);
extern struct gpio_sw *request_gpio_switch(struct device *dev, u32 id);
extern void free_gpio_switch(struct gpio_sw *sw);
extern void __gpio_switch_set(struct gpio_sw *sw, int on);

static inline void gpio_switch_set(struct gpio_sw *sw, int on)
{
	if (!sw)
		return;

	if ((on && (sw->enable_count++ == 0)) ||
		(!on && (--sw->enable_count == 0)))
		__gpio_switch_set(sw, on);
}

extern void gpio_switch_set_suspend_state(struct gpio_sw *sw, int suspend_state);
extern void gpio_switch_set_resume_state(struct gpio_sw *sw, int resume_state);

#endif
