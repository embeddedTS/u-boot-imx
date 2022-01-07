/*
 * Copyright (C) 2017-2022 Technologic Systems, Inc. dba embeddedTS
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>

#include <asm/arch/mx6-pins.h>
#include <asm/gpio.h>
#include <asm/imx-common/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <cli.h>
#include <command.h>
#include <console.h>
#include <i2c.h>
#include <spi.h>
#include <status_led.h>
#include <usb.h>
#include <div64.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/clock.h>

#include "tsfpga.h"

#include <miiphy.h>

void leds_test(void)
{
	int i;
	red_led_on();
	green_led_on();

	for(i = 0; i < 24; i++){
		if(i % 4 == 0) red_led_on();
		else red_led_off();
		if(i % 4 == 1) green_led_on();
		else green_led_off();
		mdelay(100);
	}

	red_led_on();
	green_led_on();
}

int weim_simple_test(void)
{
	static uint32_t wbuf[128] __attribute__((aligned(32)));
	static uint32_t rbuf[128] __attribute__((aligned(32)));
	unsigned long long start, now;
	uint32_t i;

	start = get_ticks();
	for (i = 0; i < 8; i++)
		wbuf[i] = ~i;

	/* Test burst modes */
	memcpy((uint32_t *)FPGA_BLOCKRAM, wbuf, 128*4);
	memcpy(rbuf, (uint32_t *)FPGA_BLOCKRAM, 128*4);

	for (i = 0; i < 128; i++) {
		if(wbuf[i] != rbuf[i]) {
			printf("Simple WEIM Burst mode failure, at addr %d Wrote 0x%X, got 0x%X\n", i, wbuf[i], rbuf[i]);
			return 1;
		}
	}

	/* Test 32-bit write/read */
	writel(0x99c0ffee, FPGA_SCRATCH_REG);
	if(readl(FPGA_SCRATCH_REG) != 0x99c0ffee){
		printf("1st 32-bit write/read failed\n");
		return 1;
	}

	writel(0xdeadbeef, FPGA_SCRATCH_REG);
	if(readl(FPGA_SCRATCH_REG ) != 0xdeadbeef){
		printf("2nd 32-bit write/read failed\n");
		return 1;
	}

	/* Test 16-bit write/read */
	writew(0xaaaa, FPGA_SCRATCH_REG);
	if(readw(FPGA_SCRATCH_REG ) != 0xaaaa){
		printf("1st 16-bit write/read failed\n");
		return 1;
	}
	writew(0x5555, FPGA_SCRATCH_REG);
	if(readw(FPGA_SCRATCH_REG) != 0x5555){
		printf("2nd 16-bit write/read failed\n");
		return 1;
	}

	now = get_ticks();
	if(now - start > 1000){ /* Normally ~550us */
		printf("Simple EIM test took %lluus instead of < 1ms!\n", now - start);
		return 1;
	}

	printf("Simple test takes %lluus\n", now - start);
	return 0;
}

int weim_burst_test(void)
{
	uint32_t wbuf[4096];
	static uint32_t rbuf[4096] __attribute__((aligned(32)));
	unsigned long long start, writeck, readck;
	int i;

	for (i = 0; i < 4096; i++)	{
		wbuf[i] = ~i;
	}

	start = get_ticks();
	memcpy((uint32_t *)FPGA_BLOCKRAM, wbuf, 4096*4);
	writeck = get_ticks();
	memcpy(rbuf, (uint32_t *)FPGA_BLOCKRAM, 4096*4);
	readck = get_ticks();

	readck = readck - writeck;
	writeck = writeck - start;

	for (i = 0; i < 4096; i++) {
		if(wbuf[i] != rbuf[i]) {
			printf("WEIM Burst mode failure, at addr %d Wrote 0x%X, got 0x%X\n", i, wbuf[i], rbuf[i]);
			return 1;
		}
	}

	return 0;
}

int weim_16bit_test(void)
{
	int i;

	for (i = 0; i < 8192; i += 2)
		writew(0, FPGA_BLOCKRAM + i);

	/* set value and verify it took, and only affected that location */
	writew(0x55aa, FPGA_BLOCKRAM + 100);;
	for (i = 0; i < 8192; i += 2) {
		if(i == 100) {
			if(readw(FPGA_BLOCKRAM + i) != 0x55aa) {
				printf("WEIM 16-bit test value failed\n");
				return 1;
			}
		} else {
			if(readw(FPGA_BLOCKRAM + i) != 0) {
				printf("WEIM 16-bit write failed at addr %d\n", i);
				return 1;
			}
		}
	}

	return 0;
}

int weim_32bit_test(void)
{
	uint32_t i;

	for (i = 0; i < 4096; i += 4)
		writel(~i, FPGA_BLOCKRAM + i);

	for (i = 0; i < 4096; i += 4) {
		uint32_t dat = readl(FPGA_BLOCKRAM + i);
		if(~dat != i) {
			printf("WEIM 32-bit test failed at %d, wrote 0x%X, read 0x%X\n", i, ~i, dat);
			return 1;
		}
	}

	return 0;
}

int weim_64bit_test(void)
{
	uint64_t i;

	for (i = 0; i < 2048; i += 8)
		writeq(~i, (uint32_t)(FPGA_BLOCKRAM + i));

	for (i = 0; i < 2048; i += 8) {
		uint64_t dat = readq((uint32_t)(FPGA_BLOCKRAM + i));
		if(~dat != i) {
			printf("WEIM 64-bit test failed at %lld, wrote 0x%llX, read 0x%llX\n", i, ~i, dat);
			return 1;
		}
	}

	return 0;
}
#define UART_TEST_SIZE (32768)

/* Test scratch register & 16550 loopback */
int uart_16550_test(uint32_t addr)
{
	unsigned long long start, end;
	int ret = 0;
	int i;
	int rd = 0;
	int sz = 0;
	uint8_t reg;
	uint8_t rbuf[UART_TEST_SIZE], wbuf[UART_TEST_SIZE];

	/* Test scratch register */
	for (i = 0; i < 0xff; i++) {
		//printf("Addr 0x%X - i %d\n", addr, i);
		writeb(i, addr + TS16550_SR);
		if(i != readb(addr + TS16550_SR)) {
			printf("16550 0x%X scratch register did not retain value\n", addr);
			return 1;
		}
	}

	/* Enable loopback */
	writeb(0x10, addr + TS16550_MCR);

	/* Enable DLAB, select 8n1 */
	writeb(0x83, addr + TS16550_LCR);

	/* Set divisor to 1 */
	writeb(0x1, addr + TS16550_DLL);
	writeb(0x0, addr + TS16550_DLH);

	/* Disable DLAB */
	writeb(0x3, addr + TS16550_LCR);

	/* Enable FIFO */
	writeb(0x1, addr + TS16550_FCR);

	/* clear tx/rx fifos, enable 8-byte trigger level */
	writeb(0x83, addr + TS16550_FCR);

	/* Enable RX/TX IRQs */
	writeb(0x3, addr + TS16550_IER);

	/* Write back 0-UART_TEST_SIZE and read it back */
	for (i = 0; i < UART_TEST_SIZE; i++)
		wbuf[i] = i;

	start = get_ticks();

	while(rd != UART_TEST_SIZE) {
		reg = readb(addr + TS16550_IIR);

		if(ctrlc()) return 1;

		switch((reg >> 1) & 0x7){
		case 0x0:
			readb(addr + TS16550_MSR);
			break;
		case 0x1:
			if(sz < UART_TEST_SIZE) {
				for (i = 0; i < 16; i++){
					if(sz != UART_TEST_SIZE){
						writeb(wbuf[sz], addr + TS16550_THR);
						sz++;
						readb(addr + TS16550_SR); /* Dummy read following a write */
					}
				}
			}
			break;
		case 0x6:
		case 0x2:
			rbuf[rd] = readb(addr + TS16550_RBR);
			rd++;
			break;
		case 0x3:
			readb(addr + TS16550_LSR);
			break;
		default:
			printf("Read back an invalid 16550 IIR value!\n");
			return 1;
		}

		end = get_ticks();
		/* Add timeout here, disabling while I'm changing test size */
		/*if((end - start) > 200000) {
			printf("16550 timeout\n");
			return 1;
		}*/
	}

	/* Turn off IRQs */
	writeb(0x0, addr + TS16550_IER);

	/* Turn off loopback */
	writeb(0x10, addr + TS16550_MCR);

	for (i = 0; i < UART_TEST_SIZE; i++) {
		if(wbuf[i] != rbuf[i]) {
			printf("IDX %d: Expected 0x%X, got 0x%X\n",i,  wbuf[i], rbuf[i]);
			ret++;
		}
	}
	if(ret)
		printf("%d errors\n", ret);
	else
		printf("Took %llu 8MHz ticks (%dms)\n", end - start, (uint32_t)(end - start)/8000);

	return ret;
}

/* Tx only without loopback on */
int uart_16550_tx_test(uint32_t addr)
{
	unsigned long long start, end;
	int ret = 0;
	int i;
	int sz = 0;
	uint8_t reg;
	uint8_t wbuf[UART_TEST_SIZE];

	/* Test scratch register */
	for (i = 0; i < 0xff; i++) {
		//printf("Addr 0x%X - i %d\n", addr, i);
		writeb(i, addr + TS16550_SR);
		if(i != readb(addr + TS16550_SR)) {
			printf("16550 0x%X scratch register did not retain value\n", addr);
			return 1;
		}
	}

	/* Enable DLAB, select 8n1 */
	writeb(0x83, addr + TS16550_LCR);

	/* Set divisor to 1 */
	writeb(0x1, addr + TS16550_DLL);
	writeb(0x0, addr + TS16550_DLH);

	/* Disable DLAB */
	writeb(0x3, addr + TS16550_LCR);

	/* Enable FIFO */
	writeb(0x1, addr + TS16550_FCR);

	/* clear tx/rx fifos, enable 8-byte trigger level */
	writeb(0x83, addr + TS16550_FCR);

	/* Enable TX IRQs */
	writeb(0x2, addr + TS16550_IER);

	/* Write back 0-UART_TEST_SIZE and read it back */
	for (i = 0; i < UART_TEST_SIZE; i++)
		wbuf[i] = i;

	start = get_ticks();

	while(sz != UART_TEST_SIZE) {
		reg = readb(addr + TS16550_IIR);

		if(ctrlc()) return 1;

		switch((reg >> 1) & 0x7){
		case 0x0:
			readb(addr + TS16550_MSR);
			break;
		case 0x1:
			if(sz < UART_TEST_SIZE) {
				for (i = 0; i < 16; i++){
					if(sz != UART_TEST_SIZE){
						writeb(wbuf[sz], addr + TS16550_THR);
						sz++;
						readb(addr + TS16550_SR);/* Dummy read following a write */
					}
				}
			}
			break;
		case 0x6:
		case 0x2:
			readb(addr + TS16550_RBR);
			break;
		case 0x3:
			readb(addr + TS16550_LSR);
			break;
		default:
			printf("Read back an invalid 16550 IIR value!\n");
			return 1;
		}

		end = get_ticks();
	}

	/* Turn off IRQs */
	writeb(0x0, addr + TS16550_IER);

	printf("TX Took %llu 8MHz ticks (%dms)\n", end - start, (uint32_t)(end - start)/8000);

	return ret;
}

int weim_test(void)
{
	int ret = 0;

	printf("Running simple test\n");
	ret |= weim_simple_test();
	printf("Running 32-bit test\n");
	ret |= weim_32bit_test();
	printf("Running 16-bit test\n");
	ret |= weim_16bit_test();
	printf("Running burst test\n");
	ret |= weim_burst_test();
	printf("Running 64-bit\n");
	ret |= weim_64bit_test();

	if(ret == 0)
		printf("WEIM tests passed!\n");

	return ret;
}

static int do_post_test(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int ret = 0;
	uint32_t reg;

	/* This is largely a placeholder for now.  Intending to implement the weim test,
	 * an fram/bootcount test, then have it switch to the app load, test sanity, then 
	 * test the 16550s and potentially other cores. */

	//uint64_t start,end;
	
	/* Does not run while in app load */
	//ret |= weim_test();

	//uart_16550_tx_test(0x50000040);

	//ret |= uart_16550_test(0x50000040);

	/*start = get_ticks();
	udelay(1000*1000);
	end = get_ticks();
	printf("Took %llu 8MHz ticks (%dus)\n", end - start, (uint32_t)(end - start)/8);

	start = readl(FPGA_USEC_CTR);
	udelay(1000*1000);
	end = readl(FPGA_USEC_CTR);

	printf("Took %llu us ticks\n", end - start);*/

	return ret;
}

U_BOOT_CMD(post, 2, 1,	do_post_test,
	"Runs a POST test",
	"[-d]\n"
	"If -d is supplied, the test is destructive to data on eMMC\n"
);
