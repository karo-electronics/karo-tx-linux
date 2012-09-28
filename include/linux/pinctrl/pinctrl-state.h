/*
 * Standard pin control state definitions
 */

/**
 * @PINCTRL_STATE_DEFAULT: the state the pinctrl handle shall be put
 *	into as default, usually this means the pins are up and ready to
 *	be used by the device driver. This state is commonly used by
 *	hogs to configure muxing and pins at boot, and also as a state
 *	to go into when returning from sleep and idle in
 *	.pm_runtime_resume() or ordinary .resume() for example.
 * @PINCTRL_STATE_IDLE: the state the pinctrl handle shall be put into
 *	when the pins are idle. Could typically be set from a
 *	pm_runtime_idle() operation.
 * @PINCTRL_STATE_SLEEP: the state the pinctrl handle shall be put into
 *	when the pins are sleeping. Could typically be set from a
 *	common pm_runtime_suspend() or ordinary .suspend() function.
 */
#define PINCTRL_STATE_DEFAULT "default"
#define PINCTRL_STATE_IDLE "idle"
#define PINCTRL_STATE_SLEEP "sleep"
