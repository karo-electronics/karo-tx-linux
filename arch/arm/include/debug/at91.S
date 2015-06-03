/*
 *  Copyright (C) 2003-2005 SAN People
 *
 * Debugging macro include header
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

#define SAMA5D4_UART_BASE(p)	((p) == 0 ? 0xf8004000 /* UART0 */ : \
				 ((p) == 1 ? 0cfc004000 /* UART1 */ : \
				0xf8004000))
#define SAMA5D4_USART_BASE(p)	((p) == 0 ? 0xf802c000 /* USART0 */ : \
				 (p) == 1 ? 0xf8030000 /* USART1 */ : \
				 (p) == 2 ? 0xfc008000 /* USART2 */ : \
				 (p) == 3 ? 0xfc00c000 /* USART3 */ : \
				 (p) == 4 ? 0xfc010000 /* USART4 */ : \
				0xfc00c000)

#if defined(CONFIG_AT91_DEBUG_LL_DBGU0)
#define AT91_DBGU 0xfffff200 /* AT91_BASE_DBGU0 */
#elif defined(CONFIG_AT91_DEBUG_LL_DBGU1)
#define AT91_DBGU 0xffffee00 /* AT91_BASE_DBGU1 */
#elif defined(CONFIG_DEBUG_SAMA5D4_USART)
#if CONFIG_DEBUG_SAMA5D4_UART_PORT == 0
#define AT91_DBGU 0xf802c000 /* USART0 */
#elif CONFIG_DEBUG_SAMA5D4_UART_PORT == 1
#define AT91_DBGU 0xf8030000 /* USART1 */
#elif CONFIG_DEBUG_SAMA5D4_UART_PORT == 2
#define AT91_DBGU 0xfc008000 /* USART2 */
#elif CONFIG_DEBUG_SAMA5D4_UART_PORT == 3
#define AT91_DBGU 0xfc00c000 /* USART3 */
#elif CONFIG_DEBUG_SAMA5D4_UART_PORT == 4
#define AT91_DBGU 0xfc010000 /* USART4 */
#else
#error Invalid SAMA5D4 Debug USART port selection
#endif
#elif defined(CONFIG_DEBUG_SAMA5D4_UART)
#if CONFIG_DEBUG_SAMA5D4_UART_PORT == 0
#define AT91_DBGU 0xf8004000 /* UART0 */
#elif CONFIG_DEBUG_SAMA5D4_UART_PORT == 1
#define AT91_DBGU 0cfc004000 /* UART1 */
#else
#error Invalid SAMA5D4 Debug UART port selection
#endif
#else
#error Unsupported AT91 Debug UART selection
#endif

#ifdef CONFIG_MMU
#define AT91_IO_P2V(x) ((x) - 0x01000000)
#else
#define AT91_IO_P2V(x) (x)
#endif

#define AT91_DBGU_SR		(0x14)	/* Status Register */
#define AT91_DBGU_THR		(0x1c)	/* Transmitter Holding Register */
#define AT91_DBGU_TXRDY		(1 << 1)	/* Transmitter Ready */
#define AT91_DBGU_TXEMPTY	(1 << 9)	/* Transmitter Empty */

	.macro	addruart, rp, rv, tmp
	ldr	\rp, =AT91_DBGU				@ System peripherals (phys address)
	ldr	\rv, =AT91_IO_P2V(AT91_DBGU)		@ System peripherals (virt address)
	.endm

	.macro	senduart,rd,rx
	strb	\rd, [\rx, #(AT91_DBGU_THR)]		@ Write to Transmitter Holding Register
	.endm

	.macro	waituart,rd,rx
1001:	ldr	\rd, [\rx, #(AT91_DBGU_SR)]		@ Read Status Register
	tst	\rd, #AT91_DBGU_TXRDY			@ DBGU_TXRDY = 1 when ready to transmit
	beq	1001b
	.endm

	.macro	busyuart,rd,rx
1001:	ldr	\rd, [\rx, #(AT91_DBGU_SR)]		@ Read Status Register
	tst	\rd, #AT91_DBGU_TXEMPTY			@ DBGU_TXEMPTY = 1 when transmission complete
	beq	1001b
	.endm

