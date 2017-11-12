/*
 * Sketch for Arduino Nano 3.0 (CH340 - China) and board STM32F103C8T6 (Blue Pill)
 *
 * rtl8367c.ino: VLAN switch setup based on Realtek RTL8367C(S) chips.
 * Used TP-Link TL-SG1005D rev.7 (RTL8367S) after remove EEPROM chip from board switch!
 *
 * Copyright (C) 2017 McMCC <mcmcc_at_mail_ru>
 *
 * For ARDUINO AVR:
 *
 * Remove EEPROM 24cXX on switch board!!!!
 * Pin SDA - D6 (convertor 5v-to-3.3v pin SDA on EEPROM switch(pin 5))
 * Pin SCK - D5 (convertor 5v-to-3.3v pin SCK on EEPROM switch(pin 6))
 * Arduino Vin - get from power connector +12V
 * +5V - get from Arduino 5V
 * +3.3V - get from EEPROM pin 8 on switch board
 *
 *              Logic Level Convertor 5v-to-3.3v
 *
 *            +3.3V                          +5V   SOT-23
 * EEPROM Vcc(8) o-----*--------       -------o    D __
 *                     |       |       |             ||
 *                     |  R1   |       |  R2     ----------
 *                    --- 10K  |      --- 10K    | BSS138 |
 *                    | |      |      | |        ----------
 *                    | |   Q1 |G     | |         ||    ||
 *        EEPROM      ---   -------   ---       G --    -- S
 *     SDA(5),SCK(6)   |    -  ^  -    |
 * (RTL8367S TTL 3.3v) |    |  |  |    |     D5,D6(Arduino TTL 5v)
 *               o-----*----*---  *----*------o
 *                         S|     |D  Q1 - MOSFET N-Channel
 *                          |_|\|_|     BSS138(diode built-in)
 *                            |/|       [ or 2N7000/2N7002 ]
 *
 *
 * For ARDUINO STM32:
 *
 * No need Logic Level Convertor, use pin-to-pin conection.
 * Remove EEPROM 24cXX on switch board!!!!
 * Cut on the board STM32 the track from power USB!!!!
 * +3.3V get from EEPROM pin 8 on switch board.
 * GND get from EEPROM pin 1-4 on switch board.
 *
 * Pin SDA - PB10 (to EEPROM pin 5 on switch board)
 * Pin SCK - PB11 (to EEPROM pin 6 on switch board)
 *
 * NEW! Added support write configuration on external I2C EEPROM 24cXX.
 * !!!!!!!!!!Uncomment USE_I2C_EEPROM for using!!!!!!!!!!!!!!
 * Pin 5 SDA EEPROM connect to PB7 STM32 board (between pullup resitor 4.7-6.8K)
 * Pin 6 SCK EEPROM connect to PB6 STM32 board (between pullup resitor 4.7-6.8K)
 * Pin 1-4 and 7 EEPROM connect to G STM32 board
 * Pin 8 EEPROM connect to 3.3 STM32 board
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

/* RTK API */
#define _LITTLE_ENDIAN		1
#define RTL8367C_CVIDXNO	32

typedef struct rtk_portmask_e
{
	uint32_t bits[1];
} rtk_portmask_t;

typedef enum rtk_led_group_e
{
	LED_GROUP_0 = 0,
	LED_GROUP_1,
	LED_GROUP_2,
	LED_GROUP_END,
} rtk_led_group_t;

typedef enum rtk_led_config_e
{
	LED_CONFIG_LEDOFF = 0,
	LED_CONFIG_DUPCOL,
	LED_CONFIG_LINK_ACT,
	LED_CONFIG_SPD1000,
	LED_CONFIG_SPD100,
	LED_CONFIG_SPD10,
	LED_CONFIG_SPD1000ACT,
	LED_CONFIG_SPD100ACT,
	LED_CONFIG_SPD10ACT,
	LED_CONFIG_SPD10010ACT,
	LED_CONFIG_LOOPDETECT,
	LED_CONFIG_EEE,
	LED_CONFIG_LINKRX,
	LED_CONFIG_LINKTX,
	LED_CONFIG_MASTER,
	LED_CONFIG_ACT,
	LED_CONFIG_END,
} rtk_led_config_t;

typedef enum rtk_led_operation_e
{
	LED_OP_SCAN = 0,
	LED_OP_PARALLEL,
	LED_OP_SERIAL,
	LED_OP_END,
} rtk_led_operation_t;

typedef enum rtl8367c_accframetype_e
{
	FRAME_TYPE_BOTH = 0,
	FRAME_TYPE_TAGGED_ONLY,
	FRAME_TYPE_UNTAGGED_ONLY,
	FRAME_TYPE_MAX_BOUND
} rtl8367c_accframetype;

typedef enum rtk_vlan_acceptFrameType_e
{
	ACCEPT_FRAME_TYPE_ALL = 0,    /* untagged, priority-tagged and tagged */
	ACCEPT_FRAME_TYPE_TAG_ONLY,   /* tagged */
	ACCEPT_FRAME_TYPE_UNTAG_ONLY, /* untagged and priority-tagged */
	ACCEPT_FRAME_TYPE_END
} rtk_vlan_acceptFrameType_t;

enum RTL8367C_LEDOP
{
	LEDOP_SCAN0 = 0,
	LEDOP_SCAN1,
	LEDOP_PARALLEL,
	LEDOP_SERIAL,
	LEDOP_END,
};

typedef struct VLANCONFIGSMI
{
#ifdef _LITTLE_ENDIAN
	uint16_t mbr:11;
	uint16_t reserved:5;

	uint16_t fid_msti:4;
	uint16_t reserved2:12;

	uint16_t vbpen:1;
	uint16_t vbpri:3;
	uint16_t envlanpol:1;
	uint16_t meteridx:6;
	uint16_t reserved3:5;

	uint16_t evid:13;
	uint16_t reserved4:3;
#else
	uint16_t reserved:5;
	uint16_t mbr:11;

	uint16_t reserved2:12;
	uint16_t fid_msti:4;

	uint16_t reserved3:5;
	uint16_t meteridx:6;
	uint16_t envlanpol:1;
	uint16_t vbpri:3;
	uint16_t vbpen:1;

	uint16_t reserved4:3;
	uint16_t evid:13;
#endif
} rtl8367c_vlanconfigsmi;

typedef struct VLANCONFIGUSER
{
	uint16_t evid;
	uint16_t mbr;
	uint16_t fid_msti;
	uint16_t envlanpol;
	uint16_t meteridx;
	uint16_t vbpen;
	uint16_t vbpri;
} rtl8367c_vlanconfiguser;

typedef struct VLANTABLE
{
#ifdef _LITTLE_ENDIAN
	uint16_t mbr:8;
	uint16_t untag:8;

	uint16_t fid_msti:4;
	uint16_t vbpen:1;
	uint16_t vbpri:3;
	uint16_t envlanpol:1;
	uint16_t meteridx:5;
	uint16_t ivl_svl:1;
	uint16_t mbr_ext_0_1:2;

	uint16_t mbr_ext_2:1;
	uint16_t untagset_ext:3;
	uint16_t mtr_idx_ext:1;
	uint16_t reserved:11;
#else
	uint16_t untag:8;
	uint16_t mbr:8;

	uint16_t mbr_ext_0_1:2;
	uint16_t ivl_svl:1;
	uint16_t meteridx:5;
	uint16_t envlanpol:1;
	uint16_t vbpri:3;
	uint16_t vbpen:1;
	uint16_t fid_msti:4;

	uint16_t reserved:11;
	uint16_t mtr_idx_ext:1;
	uint16_t untagset_ext:3;
	uint16_t mbr_ext_2:1;
#endif
} rtl8367c_vlan4kentrysmi;

typedef struct USER_VLANTABLE
{
	uint16_t vid;
	uint16_t mbr;
	uint16_t untag;
	uint16_t fid_msti;
	uint16_t envlanpol;
	uint16_t meteridx;
	uint16_t vbpen;
	uint16_t vbpri;
	uint16_t ivl_svl;
} rtl8367c_user_vlan4kentry;

typedef enum rtl8367c_egtagmode_e
{
	EG_TAG_MODE_ORI = 0,
	EG_TAG_MODE_KEEP,
	EG_TAG_MODE_PRI_TAG,
	EG_TAG_MODE_REAL_KEEP,
	EG_TAG_MODE_END
} rtl8367c_egtagmode;

enum RTL8367C_TABLE_ACCESS_OP
{
	TB_OP_READ = 0,
	TB_OP_WRITE
};

enum RTL8367C_TABLE_ACCESS_TARGET
{
	TB_TARGET_ACLRULE = 1,
	TB_TARGET_ACLACT,
	TB_TARGET_CVLAN,
	TB_TARGET_L2,
	TB_TARGET_IGMP_GROUP
};

enum PRIDECISION
{
	PRIDEC_PORT = 0,
	PRIDEC_ACL,
	PRIDEC_DSCP,
	PRIDEC_1Q,
	PRIDEC_1AD,
	PRIDEC_CVLAN,
	PRIDEC_DA,
	PRIDEC_SA,
	PRIDEC_END,
};

enum RTL8367C_PRIDEC_TABLE
{
	PRIDEC_IDX0 = 0,
	PRIDEC_IDX1,
	PRIDEC_IDX_END,
};

enum FLOW_CONTROL_TYPE
{
	FC_EGRESS = 0,
	FC_INGRESS,
};

typedef struct rtk_vlan_cfg_s
{
	rtk_portmask_t mbr;
	rtk_portmask_t untag;
	uint16_t ivl_en;
	uint16_t fid_msti;
	uint16_t envlanpol;
	uint16_t meteridx;
	uint16_t vbpen;
	uint16_t vbpri;
} rtk_vlan_cfg_t;

typedef enum vlan_mbrCfgType_e
{
	MBRCFG_UNUSED = 0,
	MBRCFG_USED_BY_VLAN,
	MBRCFG_END
} vlan_mbrCfgType_t;

typedef enum rtk_qos_priDecTbl_e
{
	PRIDECTBL_IDX0 = 0,
	PRIDECTBL_IDX1,
	PRIDECTBL_END,
} rtk_qos_priDecTbl_t;

typedef uint32_t rtk_vlan_t;

rtk_vlan_t vlan_mbrCfgVid[RTL8367C_CVIDXNO];
vlan_mbrCfgType_t vlan_mbrCfgUsage[RTL8367C_CVIDXNO];
/* End RTK API */

#if !defined(__AVR__) /* if STM32 Arduino */
/*
#define USE_I2C_EEPROM				1
*/
#define PIN_SDA					PB10
#define PIN_SCK					PB11

#include <inttypes.h>
#ifndef USE_I2C_EEPROM
#include <EEPROM.h>
#else
#include <Wire.h>

/* Set I2C1, 400KHz, PB7 - SDA1, PB6 - SCL1 */
HardWire HWire(1, I2C_FAST_MODE);
const byte DEVADDR = 0x50;

char i2c_eeprom_read_byte(unsigned eeaddr)
{
	byte rdata = -1;

	/* Three lsb of Device address byte are bits 8-10 of eeaddress */
	byte devaddr = DEVADDR | ((eeaddr >> 8) & 0x07);
	byte addr    = eeaddr;

	HWire.beginTransmission(devaddr);
	HWire.write(int(addr));
	HWire.endTransmission();
	HWire.requestFrom(int(devaddr), 1);
	if (HWire.available()) {
		rdata = HWire.read();
	}
	return rdata;
}

int i2c_eeprom_write_byte(unsigned eeaddr, uint8_t data)
{
	/* Three lsb of Device address byte are bits 8-10 of eeaddress */
	byte devaddr = DEVADDR | ((eeaddr >> 8) & 0x07);
	byte addr    = eeaddr;

	HWire.beginTransmission(devaddr);
	HWire.write(int(addr));
	HWire.write(char(data));
	HWire.endTransmission();
	delay(10);

	return 0;
}
#endif

struct EERef {

	EERef( const int index )
			: index( index )	{}

	/* Access/read members. */
#ifndef USE_I2C_EEPROM
	uint8_t operator*() const		{ return EEPROM.read( (uint16_t) index ); }
#else
	uint8_t operator*() const		{ return i2c_eeprom_read_byte( (unsigned) index ); }
#endif
	operator const uint8_t() const		{ return **this; }

	/* Assignment/write members. */
	EERef &operator=( const EERef &ref )	{ return *this = *ref; }
#ifndef USE_I2C_EEPROM
	EERef &operator=( uint8_t in )		{ return EEPROM.write( (uint16_t) index, in ), *this;  }
#else
	EERef &operator=( uint8_t in )		{ return i2c_eeprom_write_byte( (unsigned) index, in ), *this;  }
#endif
	EERef &operator +=( uint8_t in )	{ return *this = **this + in; }
	EERef &operator -=( uint8_t in )	{ return *this = **this - in; }
	EERef &operator *=( uint8_t in )	{ return *this = **this * in; }
	EERef &operator /=( uint8_t in )	{ return *this = **this / in; }
	EERef &operator ^=( uint8_t in )	{ return *this = **this ^ in; }
	EERef &operator %=( uint8_t in )	{ return *this = **this % in; }
	EERef &operator &=( uint8_t in )	{ return *this = **this & in; }
	EERef &operator |=( uint8_t in )	{ return *this = **this | in; }
	EERef &operator <<=( uint8_t in )	{ return *this = **this << in; }
	EERef &operator >>=( uint8_t in )	{ return *this = **this >> in; }

	EERef &update( uint8_t in )		{ return  in != *this ? *this = in : *this; }

	/* Prefix increment/decrement */
	EERef& operator++()			{ return *this += 1; }
	EERef& operator--()			{ return *this -= 1; }

	/* Postfix increment/decrement */
	uint8_t operator++ (int) {
		uint8_t ret = **this;
		return ++(*this), ret;
	}

	uint8_t operator-- (int){
		uint8_t ret = **this;
		return --(*this), ret;
	}

	int index; /* Index of current EEPROM cell. */
};

struct EEPtr {

	EEPtr( const int index )
			: index( index )	{}

	operator const int() const		{ return index; }
	EEPtr &operator=( int in )		{ return index = in, *this; }

	/* Iterator functionality. */
	bool operator!=( const EEPtr &ptr )	{ return index != ptr.index; }
	EERef operator*()			{ return index; }

	/* Prefix & Postfix increment/decrement */
	EEPtr& operator++()			{ return ++index, *this; }
	EEPtr& operator--()			{ return --index, *this; }
	EEPtr operator++ (int)			{ return index++; }
	EEPtr operator-- (int)			{ return index--; }

	int index; /* Index of current EEPROM cell. */
};

struct EEPROMClass_EMU {

	/* Basic user access methods. */
	EERef operator[]( const int idx )	{ return idx; }
	uint8_t read( int idx )			{ return EERef( idx ); }
	void write( int idx, uint8_t val )	{ (EERef( idx )) = val; }
	void update( int idx, uint8_t val )	{ EERef( idx ).update( val ); }

	/* Functionality to 'get' and 'put' objects to and from EEPROM. */
	template< typename T > T &get( int idx, T &t ) {
		EEPtr e = idx;
		uint8_t *ptr = (uint8_t*) &t;
		for( int count = sizeof(T) ; count ; --count, ++e )  *ptr++ = *e;
		return t;
	}

	template< typename T > const T &put( int idx, const T &t ) {
		EEPtr e = idx;
		const uint8_t *ptr = (const uint8_t*) &t;
		for( int count = sizeof(T) ; count ; --count, ++e )  (*e).update( *ptr++ );
		return t;
	}
};

static EEPROMClass_EMU EEPROM_EMU;

#ifndef USE_I2C_EEPROM
#define TYPE_EEPROM				"_EMU"
#define EEPROM_update(a, b)			EEPROM.update(a, b)
#define EEPROM_read(a)				EEPROM.read(a)
#define EEPROM_write(a, b)			EEPROM.write(a, b)
#else
#define TYPE_EEPROM				"_I2C"
#define EEPROM_update(a, b)			EEPROM_EMU.update(a, b)
#define EEPROM_read(a)				EEPROM_EMU.read(a)
#define EEPROM_write(a, b)			EEPROM_EMU.write(a, b)
#endif
#define EEPROM_get(a, b)			EEPROM_EMU.get(a, b)
#define EEPROM_put(a, b)			EEPROM_EMU.put(a, b)
#define BOARD					"_STM32" TYPE_EEPROM

#else /* if AVR Arduino */

#include <EEPROM.h>

#define PIN_SDA					6
#define PIN_SCK					5
#define EEPROM_update(a, b)			EEPROM.update(a, b)
#define EEPROM_read(a)				EEPROM.read(a)
#define EEPROM_write(a, b)			EEPROM.write(a, b)
#define EEPROM_get(a, b)			EEPROM.get(a, b)
#define EEPROM_put(a, b)			EEPROM.put(a, b)
#define BOARD					"_AVR"

#endif /* End support board */

#define VERFW					"v.1.0.03_RTL8367C"
#define MAGIC_EEPROM_START			0x8367

#define MAX_VLAN_GROUP				8
#define MAX_PORTS				5

/* If Port1 = P0, Port2 = P1...etc, please disable this define */
#define PORTS_INVERSION				1

#ifdef PORTS_INVERSION
#define VERSION					VERFW BOARD "_INV"
#define PORT_INV(x)				(4 - x)
#else
#define VERSION					VERFW BOARD
#define PORT_INV(x)				(x)
#endif

struct eeprom_vlan_record {
	uint16_t vid;
	uint8_t ports_mask;
	uint8_t prio;
};

/* Magic EEPROM start + tag/untag masks group + idx + vlan record structure * 8 */
#define CFG_SIZE (2 + 2 + 1 + (sizeof(eeprom_vlan_record) * 8))

String readString;

#define CLK_DURATION(clk)	delayMicroseconds(clk)
#define DELAY			3
#define ack_timer		10

/* smi */
void _smi_start()
{
	/* change GPIO pin to Output only */
	pinMode(PIN_SCK, OUTPUT);
	pinMode(PIN_SDA, OUTPUT);

	/* Initial state: SCK: 0, SDA: 1 */
	digitalWrite(PIN_SCK, LOW);
	digitalWrite(PIN_SDA, HIGH);
	CLK_DURATION(DELAY);

	/* CLK 1: 0 -> 1, 1 -> 0 */
	digitalWrite(PIN_SCK, HIGH);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SCK, LOW);
	CLK_DURATION(DELAY);

	/* CLK 2: */
	digitalWrite(PIN_SCK, HIGH);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SDA, LOW);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SCK, LOW);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SDA, HIGH);
}

void _smi_writeBit(uint16_t signal, uint32_t bitLen)
{
	for ( ; bitLen > 0; bitLen--) {
		CLK_DURATION(DELAY);

		/* prepare data */
		if (signal & (1UL << (bitLen - 1)))
			digitalWrite(PIN_SDA, HIGH);
		else
			digitalWrite(PIN_SDA, LOW);
		CLK_DURATION(DELAY);

		/* clocking */
		digitalWrite(PIN_SCK, HIGH);
		CLK_DURATION(DELAY);
		digitalWrite(PIN_SCK, LOW);
	}
}

void _smi_readBit(uint32_t bitLen, uint32_t *rData)
{
	unsigned long u;

	/* change GPIO pin to Input only */
	pinMode(PIN_SDA, INPUT);

	for (*rData = 0; bitLen > 0; bitLen--) {
		CLK_DURATION(DELAY);

		/* clocking */
		digitalWrite(PIN_SCK, HIGH);
		CLK_DURATION(DELAY);
		u = (digitalRead(PIN_SDA) == HIGH) ? 1 : 0;
		digitalWrite(PIN_SCK, LOW);

		*rData |= (u << (bitLen - 1));
	}

	/* change GPIO pin to Output only */
	pinMode(PIN_SDA, OUTPUT);
}

void _smi_stop()
{
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SDA, LOW);
	digitalWrite(PIN_SCK, HIGH);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SDA, HIGH);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SCK, HIGH);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SCK, LOW);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SCK, HIGH);

	/* add a click */
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SCK, LOW);
	CLK_DURATION(DELAY);
	digitalWrite(PIN_SCK, HIGH);

	/* change GPIO pin to Input only */
	pinMode(PIN_SDA, INPUT);
	pinMode(PIN_SCK, INPUT);
}

uint32_t smi_read(uint32_t mAddrs)
{
	uint32_t rawData = 0, rData = 0, ACK = 0;
	uint8_t con;

	_smi_start();				/* Start SMI */
	_smi_writeBit(0x0b, 4);			/* CTRL code: 4'b1011 for RTL8370 */
	_smi_writeBit(0x4, 3);			/* CTRL code: 3'b100 */
	_smi_writeBit(0x1, 1);			/* 1: issue READ command */

	con = 0;
	do {
		con++;
		_smi_readBit(1, &ACK);		/* ACK for issuing READ command */
	} while ((ACK != 0) && (con < ack_timer));

	if (ACK != 0) Serial.println(F("smi_read_0...timeout!"));

	_smi_writeBit((mAddrs & 0xff), 8);	/* Set reg_addr[7:0] */

	con = 0;
	do {
		con++;
		_smi_readBit(1, &ACK);		/* ACK for setting reg_addr[7:0] */
	} while ((ACK != 0) && (con < ack_timer));

	if (ACK != 0) Serial.println(F("smi_read_1...timeout!"));

	_smi_writeBit((mAddrs >> 8), 8);	/* Set reg_addr[15:8] */

	con = 0;
	do {
		con++;
		_smi_readBit(1, &ACK);		/* ACK by RTL836x */
	} while ((ACK != 0) && (con < ack_timer));

	if (ACK != 0) Serial.println(F("smi_read_2...timeout!"));

	_smi_readBit(8, &rawData);		/* Read DATA [7:0] */
	rData = rawData & 0xff;

	_smi_writeBit(0x00, 1);			/* ACK by CPU */
	_smi_readBit(8, &rawData);		/* Read DATA [15: 8] */
	_smi_writeBit(0x01, 1);			/* ACK by CPU */

	rData |= (rawData << 8);

	_smi_stop();

	return rData;
}

void smi_write(uint32_t mAddrs, uint32_t rData)
{
	int8_t con;
	uint32_t ACK = 0;

	_smi_start();				/* Start SMI */
	_smi_writeBit(0x0b, 4);			/* CTRL code: 4'b1011 for RTL836x */
	_smi_writeBit(0x4, 3);			/* CTRL code: 3'b100 */
	_smi_writeBit(0x0, 1);			/* 0: issue WRITE command */

	con = 0;
	do {
		con++;
		_smi_readBit(1, &ACK);		/* ACK for issuing WRITE command */
	} while ((ACK != 0) && (con < ack_timer));

	if (ACK != 0) Serial.println(F("smi_write_0...timeout!"));

	_smi_writeBit((mAddrs & 0xff), 8);	/* Set reg_addr[7:0] */

	con = 0;
	do {
		con++;
		_smi_readBit(1, &ACK);		/* ACK for setting reg_addr[7:0] */
	} while ((ACK != 0) && (con < ack_timer));

	if (ACK != 0) Serial.println(F("smi_write_1...timeout!"));

	_smi_writeBit((mAddrs >> 8), 8);	/* Set reg_addr[15:8] */

	con = 0;
	do {
		con++;
		_smi_readBit(1, &ACK);		/* ACK or setting reg_addr[15:8] */
	} while ((ACK != 0) && (con < ack_timer));

	if (ACK != 0) Serial.println(F("smi_write_2...timeout!"));

	_smi_writeBit(rData & 0xff, 8);		/* Write Data [7:0] out */

	con = 0;
	do {
		con++;
		_smi_readBit(1, &ACK);		/* ACK for writting data [7:0] */
	} while ((ACK != 0) && (con < ack_timer));

	if (ACK != 0) Serial.println(F("smi_write_3...timeout!"));

	_smi_writeBit(rData >> 8, 8);		/* Write Data [15:8] out */

	con = 0;
	do {
		con++;
		_smi_readBit(1, &ACK);		/* ACK for writting data [15:8] */
	} while ((ACK != 0) && (con < ack_timer));

	if (ACK != 0) Serial.println(F("smi_write_4...timeout!"));

	_smi_stop();
}
/* End smi */

/* rtl8367c_asicdrv */
#define RTL8367C_REGBITLENGTH		16
#define RTL8367C_REGDATAMAX		0xffff

int rtl8367c_setAsicRegBit(uint32_t reg, uint32_t bit, uint32_t value)
{
	if(bit >= RTL8367C_REGBITLENGTH)
		return -1;

	uint32_t regData = smi_read(reg);
	if(value)
		regData = regData | (1UL << bit);
	else
		regData = regData & (~(1UL << bit));
	smi_write(reg, regData);

	return 0;
}

int rtl8367c_getAsicRegBit(uint32_t reg, uint32_t bit, uint32_t *pValue)
{
	if(bit >= RTL8367C_REGBITLENGTH)
		return -1;

	uint32_t regData = smi_read(reg);
	*pValue = (regData & (1UL << bit)) >> bit;

	return 0;
}

int rtl8367c_setAsicRegBits(uint32_t reg, uint32_t bits, uint32_t value)
{
	if(bits >= (1UL << RTL8367C_REGBITLENGTH))
		return -1;

	uint32_t bitsShift = 0;
	while(!(bits & (1UL << bitsShift)))
	{
		bitsShift++;
		if(bitsShift >= RTL8367C_REGBITLENGTH)
			return -1;
	}

	uint32_t valueShifted = (unsigned long)value << bitsShift;
	if(valueShifted > RTL8367C_REGDATAMAX)
		return -1;

	uint32_t regData = smi_read(reg);
	regData = regData & (~bits);
	regData = regData | (valueShifted & bits);
	smi_write(reg, regData);

	return 0;
}

int rtl8367c_getAsicRegBits(uint32_t reg, uint32_t bits, uint32_t *pValue)
{
	if(bits >= (1UL << RTL8367C_REGBITLENGTH))
		return -1;

	uint32_t bitsShift = 0;
	while(!(bits & (1UL << bitsShift)))
	{
		bitsShift++;
		if(bitsShift >= RTL8367C_REGBITLENGTH)
			return -1;
	}

	uint32_t regData = smi_read(reg);
	*pValue = (regData & bits) >> bitsShift;

	return 0;
}

int rtl8367c_setAsicReg(uint32_t reg, uint32_t value)
{
	smi_write(reg, value);
	return 0;
}

int rtl8367c_getAsicReg(uint32_t reg, uint32_t *pValue)
{
	*pValue = smi_read(reg);
	return 0;
}
/* End rtl8367c_asicdrv */

/* rtl8367c_asicdrv_phy */
#define RTL8367C_CMD_MASK			1
#define RTL8367C_RW_MASK			2
#define RTL8367C_PHY_INTERNALNOMAX		4
#define RTL8367C_PHY_OFFSET			5
#define RTL8367C_PHY_EXTERNALMAX		7
#define RTL8367C_PHY_REGNOMAX			0x1f
#define RTL8367C_CFG_CPU_OCPADR_MSB_MASK	0xfc0
#define RTL8367C_REG_GPHY_OCP_MSB_0		0x1d15
#define RTL8367C_REG_INDRECT_ACCESS_CTRL	0x1f00
#define RTL8367C_REG_INDRECT_ACCESS_STATUS	0x1f01
#define RTL8367C_REG_INDRECT_ACCESS_ADDRESS	0x1f02
#define RTL8367C_REG_INDRECT_ACCESS_WRITE_DATA	0x1f03
#define RTL8367C_REG_INDRECT_ACCESS_READ_DATA	0x1f04
#define RTL8367C_PHY_BASE			0x2000

int rtl8367c_setAsicPHYReg(uint32_t phyNo, uint32_t phyAddr, uint32_t value)
{
	uint32_t busyFlag = 0, checkCounter = 100, regData;

	if(phyNo > RTL8367C_PHY_INTERNALNOMAX)
		return -1;

	if(phyAddr > RTL8367C_PHY_REGNOMAX)
		return -1;

	/* Check internal phy access busy or not */
	rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
	if(busyFlag)
		return -2;

	/* Default OCP Address */
	rtl8367c_setAsicRegBits(RTL8367C_REG_GPHY_OCP_MSB_0, RTL8367C_CFG_CPU_OCPADR_MSB_MASK, 0x29);

	/* prepare access data */
	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_WRITE_DATA, value);

	/* prepare access address */
	regData = RTL8367C_PHY_BASE | ((unsigned long)phyNo << RTL8367C_PHY_OFFSET) | phyAddr;

	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_ADDRESS, regData);

	/* Set WRITE Command */
	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_CTRL, RTL8367C_CMD_MASK | RTL8367C_RW_MASK);

	while(checkCounter) {
		rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
		if(busyFlag) {
			checkCounter--;
			if(!checkCounter) {
				Serial.println(F("rtl8367c_setAsicPHYReg...timeout!"));
				return -2;
			}
		} else
			checkCounter = 0;
	}

	return 0;
}

int rtl8367c_getAsicPHYReg(uint32_t phyNo, uint32_t phyAddr, uint32_t *value)
{
	uint32_t busyFlag = 0, checkCounter = 100, regData;

	if(phyNo > RTL8367C_PHY_INTERNALNOMAX)
		return -1;

	if(phyAddr > RTL8367C_PHY_REGNOMAX)
		return -1;

	/* Check internal phy access busy or not */
	rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
	if(busyFlag)
		return -2;

	/* Default OCP Address */
	rtl8367c_setAsicRegBits(RTL8367C_REG_GPHY_OCP_MSB_0, RTL8367C_CFG_CPU_OCPADR_MSB_MASK, 0x29);

	/* prepare access address */
	regData = RTL8367C_PHY_BASE | ((unsigned long)phyNo << RTL8367C_PHY_OFFSET) | phyAddr;

	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_ADDRESS, regData);

	/* Set READ Command */
	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_CTRL, RTL8367C_CMD_MASK);

	while(checkCounter) {
		rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
		if(busyFlag) {
			checkCounter--;
			if(!checkCounter) {
				Serial.println(F("rtl8367c_getAsicPHYReg...timeout!"));
				return -2;
			}
		} else
			checkCounter = 0;
	}

	/* get PHY register */
	rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_READ_DATA, &regData);
	*value = regData;

	return 0;
}

int rtl8367c_setAsicPHYOCPReg(uint32_t phyNo, uint32_t ocpAddr, uint32_t ocpData)
{
	uint32_t busyFlag = 0, checkCounter = 100, regData;
	uint32_t ocpAddrPrefix, ocpAddr9_6, ocpAddr5_1;

	if(phyNo > RTL8367C_PHY_INTERNALNOMAX)
		return -1;

	if(ocpAddr > RTL8367C_PHY_REGNOMAX)
		return -1;

	/* Check internal phy access busy or not */
	rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
	if(busyFlag)
		return -2;

	/* OCP prefix */
	ocpAddrPrefix = ((ocpAddr & 0xFC00) >> 10);
	rtl8367c_setAsicRegBits(RTL8367C_REG_GPHY_OCP_MSB_0, RTL8367C_CFG_CPU_OCPADR_MSB_MASK, ocpAddrPrefix);

	/* prepare access data */
	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_WRITE_DATA, ocpData);

	/* prepare access address */
	ocpAddr9_6 = ((ocpAddr >> 6) & 0x000F);
	ocpAddr5_1 = ((ocpAddr >> 1) & 0x001F);
	regData = RTL8367C_PHY_BASE | (ocpAddr9_6 << 8) | (phyNo << RTL8367C_PHY_OFFSET) | ocpAddr5_1;

	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_ADDRESS, regData);

	/* Set WRITE Command */
	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_CTRL, RTL8367C_CMD_MASK | RTL8367C_RW_MASK);

	while(checkCounter) {
		rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
		if(busyFlag) {
			checkCounter--;
			if(!checkCounter) {
				Serial.println(F("rtl8367c_setAsicPHYOCPReg...timeout!"));
				return -2;
			}
		} else
			checkCounter = 0;
	}

	return 0;
}

int rtl8367c_getAsicPHYOCPReg(uint32_t phyNo, uint32_t ocpAddr, uint32_t *pRegData)
{
	uint32_t busyFlag = 0, checkCounter = 100, regData;
	uint32_t ocpAddrPrefix, ocpAddr9_6, ocpAddr5_1;

	if(phyNo > RTL8367C_PHY_INTERNALNOMAX)
		return -1;

	if(ocpAddr > RTL8367C_PHY_REGNOMAX)
		return -1;

	/* Check internal phy access busy or not */
	rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
	if(busyFlag)
		return -2;

	/* OCP prefix */
	ocpAddrPrefix = ((ocpAddr & 0xFC00) >> 10);
	rtl8367c_setAsicRegBits(RTL8367C_REG_GPHY_OCP_MSB_0, RTL8367C_CFG_CPU_OCPADR_MSB_MASK, ocpAddrPrefix);

	/* prepare access address */
	ocpAddr9_6 = ((ocpAddr >> 6) & 0x000F);
	ocpAddr5_1 = ((ocpAddr >> 1) & 0x001F);
	regData = RTL8367C_PHY_BASE | (ocpAddr9_6 << 8) | (phyNo << RTL8367C_PHY_OFFSET) | ocpAddr5_1;

	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_ADDRESS, regData);

	/* Set READ Command */
	rtl8367c_setAsicReg(RTL8367C_REG_INDRECT_ACCESS_CTRL, RTL8367C_CMD_MASK);

	while(checkCounter) {
		rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_STATUS, &busyFlag);
		if(busyFlag) {
			checkCounter--;
			if(!checkCounter) {
				Serial.println(F("rtl8367c_getAsicPHYOCPReg...timeout!"));
				return -2;
			}
		} else
			checkCounter = 0;
	}

	/* get PHY register */
	rtl8367c_getAsicReg(RTL8367C_REG_INDRECT_ACCESS_READ_DATA, &regData);
	*pRegData = regData;

	return 0;

}
/* End rtl8367c_asicdrv_phy */

/* rtl8367c_asicdrv_led */
#define RTL8367C_LED_SELECT_OFFSET		0
#define RTL8367C_LED_SERI_CLK_EN_OFFSET		0
#define RTL8367C_LED_SERI_DATA_EN_OFFSET	1
#define RTL8367C_LEDGROUPNO			3
#define RTL8367C_LED_CONFIG_SEL_OFFSET		14
#define RTL8367C_LED0_CFG_MASK			0xf
#define RTL8367C_LED1_CFG_MASK			0xf0
#define RTL8367C_LED2_CFG_MASK			0xf00
#define RTL8367C_REG_LED_SYS_CONFIG		0x1b00
#define RTL8367C_REG_LED_CONFIGURATION		0x1b03
#define RTL8367C_REG_PARA_LED_IO_EN1		0x1b24
#define RTL8367C_REG_SCAN0_LED_IO_EN1		0x1b26
#define RTL8367C_REG_PARA_LED_IO_EN3		0x1b33

int rtl8367c_setAsicLedGroupEnable(uint32_t group, uint32_t portmask)
{
	uint32_t regAddr, regDataMask;

	regAddr = RTL8367C_REG_PARA_LED_IO_EN1 + group/2;
	regDataMask = 0xFF << ((group % 2) * 8);

	rtl8367c_setAsicRegBits(regAddr, regDataMask, portmask&0xff);

	regAddr = RTL8367C_REG_PARA_LED_IO_EN3;
	regDataMask = 0x3 << (group * 2);
	rtl8367c_setAsicRegBits(regAddr, regDataMask, (portmask >> 8) & 0x7);

	return 0;
}

int rtl8367c_setAsicLedOperationMode(uint32_t mode)
{
	switch(mode)
	{
		case LEDOP_PARALLEL:
			rtl8367c_setAsicRegBit(RTL8367C_REG_LED_SYS_CONFIG, RTL8367C_LED_SELECT_OFFSET, 0);
			rtl8367c_setAsicRegBit(RTL8367C_REG_SCAN0_LED_IO_EN1,RTL8367C_LED_SERI_CLK_EN_OFFSET, 0);
			rtl8367c_setAsicRegBit(RTL8367C_REG_SCAN0_LED_IO_EN1,RTL8367C_LED_SERI_DATA_EN_OFFSET, 0);
			break;
		case LEDOP_SERIAL:
			rtl8367c_setAsicRegBit(RTL8367C_REG_LED_SYS_CONFIG, RTL8367C_LED_SELECT_OFFSET, 1);
			rtl8367c_setAsicRegBit(RTL8367C_REG_SCAN0_LED_IO_EN1,RTL8367C_LED_SERI_CLK_EN_OFFSET, 1);
			rtl8367c_setAsicRegBit(RTL8367C_REG_SCAN0_LED_IO_EN1,RTL8367C_LED_SERI_DATA_EN_OFFSET, 1);
			break;
		default:
			break;
	}
	return 0;
}

#define RTK_PORTMASK_IS_PORT_SET(__portmask__, __port__)	(((__portmask__).bits[0] & (0x00000001 << __port__)) ? 1 : 0)
#define RTK_PORTMASK_SCAN(__portmask__, __port__)		for(__port__ = 0; __port__ < MAX_PORTS; __port__++)  if(RTK_PORTMASK_IS_PORT_SET(__portmask__, __port__))
int rtk_switch_portmask_L2P_get(rtk_portmask_t *pLogicalPmask, uint32_t *pPhysicalPortmask)
{
	uint32_t phy_port = 0;

	*pPhysicalPortmask = 0;
	RTK_PORTMASK_SCAN((*pLogicalPmask), phy_port)
	{
		*pPhysicalPortmask |= (0x0001 << phy_port);
	}

	return 0;
}

int rtk_led_enable_set(rtk_led_group_t group, rtk_portmask_t *pPortmask)
{
	uint32_t pmask = 0;

	rtk_switch_portmask_L2P_get(pPortmask, &pmask);
	rtl8367c_setAsicLedGroupEnable(group, pmask);

	return 0;
}

int rtk_led_operation_set(rtk_led_operation_t mode)
{
	uint32_t regData;

	switch (mode)
	{
		case LED_OP_PARALLEL:
			regData = LEDOP_PARALLEL;
			break;
		case LED_OP_SERIAL:
			regData = LEDOP_SERIAL;
			break;
		default:
			regData = 0;
			break;
	}
	rtl8367c_setAsicLedOperationMode(regData);
	return 0;
}

int rtl8367c_setAsicLedIndicateInfoConfig(uint32_t ledno, uint32_t config)
{
	const uint16_t bits[RTL8367C_LEDGROUPNO] = { RTL8367C_LED0_CFG_MASK, RTL8367C_LED1_CFG_MASK, RTL8367C_LED2_CFG_MASK };

	if(ledno >= RTL8367C_LEDGROUPNO)
		return -1;

	if(config >= LED_CONFIG_END)
		return -1;

	rtl8367c_setAsicRegBit(RTL8367C_REG_LED_CONFIGURATION, RTL8367C_LED_CONFIG_SEL_OFFSET, 0);

	return rtl8367c_setAsicRegBits(RTL8367C_REG_LED_CONFIGURATION, bits[ledno], config);
}

int rtl8367c_getAsicLedIndicateInfoConfig(uint32_t ledno, uint32_t* pConfig)
{
	const uint16_t bits[RTL8367C_LEDGROUPNO]= { RTL8367C_LED0_CFG_MASK, RTL8367C_LED1_CFG_MASK, RTL8367C_LED2_CFG_MASK };

	if(ledno >= RTL8367C_LEDGROUPNO)
		return -1;

	/* Get register value */
	return rtl8367c_getAsicRegBits(RTL8367C_REG_LED_CONFIGURATION, bits[ledno], pConfig);
}

int rtk_led_groupConfig_set(rtk_led_group_t group, rtk_led_config_t config)
{
	if (LED_GROUP_END <= group)
		return -1;
	if (LED_CONFIG_END <= config)
		return -1;
	rtl8367c_setAsicLedIndicateInfoConfig(group, (uint32_t)config);
	return 0;
}

int rtk_led_groupConfig_get(rtk_led_group_t group, rtk_led_config_t *pConfig)
{
	if (LED_GROUP_END <= group)
		return -1;
	if(NULL == pConfig)
		return -1;
	rtl8367c_getAsicLedIndicateInfoConfig(group, (uint32_t *)pConfig);
	return 0;
}

int rtk_led_init()
{
	rtk_portmask_t ledmask;

	ledmask.bits[0] = 0x1f; /* port 0 ~ port 5 LED */

	if (rtk_led_enable_set(LED_GROUP_0, &ledmask) != 0)
		return -1;

	if (rtk_led_enable_set(LED_GROUP_1, &ledmask) != 0)
		return -1;

	if (rtk_led_enable_set(LED_GROUP_2, &ledmask) != 0)
		return -1;

	/* Set LED to Parallel mode */
	if (rtk_led_operation_set(LED_OP_PARALLEL) != 0)
		return -1;

	/* set group 0 to 100Mb/s Speed/Activity */
	if (rtk_led_groupConfig_set(LED_GROUP_0, LED_CONFIG_LINK_ACT) != 0)
		return -1;

	/* set group 1 to 1000Mb/s Speed/Activity */
	if(rtk_led_groupConfig_set(LED_GROUP_1, LED_CONFIG_LINK_ACT) != 0)
		return -1;

	/* set group 2 to Collision, Full duplex */
	if(rtk_led_groupConfig_set(LED_GROUP_2, LED_CONFIG_LINK_ACT) != 0)
		return -1;

	return 0;
}
/* End rtl8367c_asicdrv_led */

/* rtl8367c_asicdrv_portIsolation */
#define RTK_MAX_NUM_OF_PORT					8
#define RTL8367C_PORTNO						11
#define RTK_MAX_PORT_MASK					0xFF
#define RTL8367C_PORTMASK					0x7FF
#define RTL8367C_PORT_ISOLATION_PORT_MASK_BASE			0x08a2
#define RTL8367C_PORT_ISOLATION_PORT_MASK_REG(port)		(RTL8367C_PORT_ISOLATION_PORT_MASK_BASE + port)
#define RTK_PORT_ID_MAX						(RTK_MAX_NUM_OF_PORT-1)

int rtl8367c_setAsicPortIsolationPermittedPortmask(uint32_t port, uint32_t permitPortmask)
{
	if(port >= RTL8367C_PORTNO)
		return -1;

	if( permitPortmask > RTL8367C_PORTMASK)
		return -2;

	return rtl8367c_setAsicReg(RTL8367C_PORT_ISOLATION_PORT_MASK_REG(port), permitPortmask);
}

int rtl8367c_getAsicPortIsolationPermittedPortmask(uint32_t port, uint32_t *pPermitPortmask)
{
	if(port >= RTL8367C_PORTNO)
		return -1;

	return rtl8367c_getAsicReg(RTL8367C_PORT_ISOLATION_PORT_MASK_REG(port), pPermitPortmask);
}

int rtk_port_isolation_get(uint32_t port, rtk_portmask_t *pPortmask)
{
	int  retVal;

	if (port > RTK_PORT_ID_MAX)
		return -1;

	if ((retVal = rtl8367c_getAsicPortIsolationPermittedPortmask(port, &pPortmask->bits[0])) != 0)
		return retVal;

	return 0;
}

int rtk_port_isolation_set(uint32_t port, rtk_portmask_t portmask)
{
	int retVal;

	if (port > RTK_PORT_ID_MAX)
		return -1;

	if (portmask.bits[0] > RTK_MAX_PORT_MASK)
		return -2;

	if ((retVal = rtl8367c_setAsicPortIsolationPermittedPortmask(port, portmask.bits[0])) != 0)
		return retVal;

	return 0;
}
/* End rtl8367c_asicdrv_portIsolation */

/* rtl8367c_asicdrv_vlan */
#define RTL8367C_VLAN_CTRL_OFFSET					0
#define RTK_ENABLE_END							2
#define RTL8367C_PORT0_FRAME_TYPE_MASK					3
#define RTL8367C_PROTOVLAN_GIDX_MAX					3
#define RTL8367C_PROTOVLAN_GROUPNO					4
#define RTL8367C_PRIMAX							7
#define RTL8367C_TABLE_TYPE_MASK					7
#define RTL8367C_COMMAND_TYPE_MASK					8
#define RTL8367C_VLAN_BUSY_CHECK_NO					(10)
#define RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET			13
#define RTL8367C_PORT_MISC_CFG_BASE					0x000e
#define RTL8367C_FIDMAX							0xF
#define RTL8367C_PORT0_VIDX_MASK					0x1F
#define RTK_MAX_METER_ID						31
#define RTL8367C_VLAN_EGRESS_MDOE_MASK					0x30
#define RTL8367C_METERNO						64
#define RTK_PHY_PORTMASK_ALL						0xdf
#define RTL8367C_TABLE_ACCESS_CTRL_REG					0x0500
#define RTL8367C_TABLE_ACCESS_ADDR_REG					0x0501
#define RTL8367C_TABLE_ACCESS_STATUS_REG				0x0502
#define RTL8367C_TABLE_ACCESS_RDDATA_BASE				0x0520
#define RTL8367C_TABLE_ACCESS_WRDATA_BASE				0x0510
#define RTL8367C_VLAN_PVID_CTRL_BASE					0x0700
#define RTL8367C_VLAN_MEMBER_CONFIGURATION_BASE				0x0728
#define RTL8367C_REG_VLAN_CTRL						0x07a8
#define RTL8367C_VLAN_INGRESS_REG					0x07a9
#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_BASE				0x07aa
#define RTL8367C_VLAN_PORTBASED_PRIORITY_BASE				0x0851
#define RTL8367C_VIDMAX							0xFFF
#define RTL8367C_EVIDMAX						0x1FFF
#define RTL8367C_VLAN_PVID_CTRL_REG(port)				(RTL8367C_VLAN_PVID_CTRL_BASE + (port >> 1))
#define RTL8367C_PORT_VIDX_MASK(port)					(RTL8367C_PORT0_VIDX_MASK << RTL8367C_PORT_VIDX_OFFSET(port))
#define RTL8367C_VLAN_PORTBASED_PRIORITY_REG(port)			(RTL8367C_VLAN_PORTBASED_PRIORITY_BASE + (port >> 2))
#define RTL8367C_VLAN_PORTBASED_PRIORITY_MASK(port)			(7 << RTL8367C_VLAN_PORTBASED_PRIORITY_OFFSET(port))
#define RTL8367C_TABLE_ACCESS_REG_DATA(op, target)			((op << 3) | target)
#define RTL8367C_PORT_VIDX_OFFSET(port)					((port & 1) << 3)
#define RTL8367C_VLAN_PORTBASED_PRIORITY_OFFSET(port)			((port & 3) << 2)
#define RTL8367C_PORT_MISC_CFG_REG(port)				(RTL8367C_PORT_MISC_CFG_BASE + (port << 5))
#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_REG(port)			(RTL8367C_VLAN_ACCEPT_FRAME_TYPE_BASE + (port >> 3))
#define RTL8367C_VLAN_ACCEPT_FRAME_TYPE_MASK(port)			(RTL8367C_PORT0_FRAME_TYPE_MASK << ((port & 0x7) << 1))
#define RTL8367C_METERMAX						(RTL8367C_METERNO - 1)
#define RTL8367C_PORTIDMAX						(RTL8367C_PORTNO - 1)
#define RTL8367C_CVIDXMAX						(RTL8367C_CVIDXNO - 1)
#define RTK_PORTMASK_CLEAR(__portmask__)				((__portmask__).bits[0] = 0)
#define RTK_PORTMASK_PORT_SET(__portmask__, __port__)			((__portmask__).bits[0] |= (0x00000001 << __port__))

int rtl8367c_setAsicVlanAccpetFrameType(uint32_t port, rtl8367c_accframetype frameType)
{
	if(port > RTL8367C_PORTIDMAX)
		return -1;

	if(frameType >= FRAME_TYPE_MAX_BOUND)
		return -2;

	return rtl8367c_setAsicRegBits(RTL8367C_VLAN_ACCEPT_FRAME_TYPE_REG(port), RTL8367C_VLAN_ACCEPT_FRAME_TYPE_MASK(port), (uint32_t)frameType);
}

int rtl8367c_getAsicVlanAccpetFrameType(uint32_t port, rtl8367c_accframetype *pFrameType)
{
	if(port > RTL8367C_PORTIDMAX)
		return -1;

	return rtl8367c_getAsicRegBits(RTL8367C_VLAN_ACCEPT_FRAME_TYPE_REG(port), RTL8367C_VLAN_ACCEPT_FRAME_TYPE_MASK(port), (uint32_t *)pFrameType);
}

int rtl8367c_getAsicVlanPortBasedVID(uint32_t port, uint32_t *pIndex, uint32_t *pPri)
{
	uint32_t regAddr, bit_mask;
	int retVal;

	if(port > RTL8367C_PORTIDMAX)
		return -1;

	regAddr = RTL8367C_VLAN_PVID_CTRL_REG(port);
	bit_mask = RTL8367C_PORT_VIDX_MASK(port);
	retVal = rtl8367c_getAsicRegBits(regAddr, bit_mask, pIndex);
	if(retVal != 0)
		return retVal;

	regAddr = RTL8367C_VLAN_PORTBASED_PRIORITY_REG(port);
	bit_mask = RTL8367C_VLAN_PORTBASED_PRIORITY_MASK(port);
	retVal = rtl8367c_getAsicRegBits(regAddr, bit_mask, pPri);
	if(retVal != 0)
		return retVal;

	return 0;
}

void _rtl8367c_VlanMCStSmi2User(rtl8367c_vlanconfigsmi *pSmiVlanCfg, rtl8367c_vlanconfiguser *pVlanCg)
{
	pVlanCg->mbr            = pSmiVlanCfg->mbr;
	pVlanCg->fid_msti       = pSmiVlanCfg->fid_msti;
	pVlanCg->evid           = pSmiVlanCfg->evid;
	pVlanCg->meteridx       = pSmiVlanCfg->meteridx;
	pVlanCg->envlanpol      = pSmiVlanCfg->envlanpol;
	pVlanCg->vbpri          = pSmiVlanCfg->vbpri;
	pVlanCg->vbpen          = pSmiVlanCfg->vbpen;
}

int rtl8367c_getAsicVlanMemberConfig(uint32_t index, rtl8367c_vlanconfiguser *pVlanCg)
{
	int retVal;
	uint32_t page_idx;
	uint32_t regAddr;
	uint32_t regData;
	uint16_t *tableAddr;
	rtl8367c_vlanconfigsmi smi_vlancfg;

	if(index > RTL8367C_CVIDXMAX)
		return -1;

	memset(&smi_vlancfg, 0, sizeof(rtl8367c_vlanconfigsmi));
	tableAddr = (uint16_t *)&smi_vlancfg;

	for(page_idx = 0; page_idx < 4; page_idx++)  /* 4 pages per VLAN Member Config */
	{
		regAddr = RTL8367C_VLAN_MEMBER_CONFIGURATION_BASE + (index * 4) + page_idx;

		retVal = rtl8367c_getAsicReg(regAddr, &regData);
		if(retVal != 0)
			return retVal;

		*tableAddr = (uint16_t)regData;
		tableAddr++;
	}

	_rtl8367c_VlanMCStSmi2User(&smi_vlancfg, pVlanCg);
	return 0;
}

int rtk_vlan_portAcceptFrameType_set(uint32_t port, rtk_vlan_acceptFrameType_t accept_frame_type)
{
	int retVal;

	if (port > RTK_PORT_ID_MAX)
		return -1;

	if (accept_frame_type >= ACCEPT_FRAME_TYPE_END)
		return -2;

	if ((retVal = rtl8367c_setAsicVlanAccpetFrameType(port, (rtl8367c_accframetype)accept_frame_type)) != 0)
		return retVal;

	return 0;
}

int rtk_vlan_portAcceptFrameType_get(uint32_t port, rtk_vlan_acceptFrameType_t *pAccept_frame_type)
{
	int retVal;
	rtl8367c_accframetype acc_frm_type;

	if (port > RTK_PORT_ID_MAX)
		return -1;

	if ((retVal = rtl8367c_getAsicVlanAccpetFrameType(port, &acc_frm_type)) != 0)
		return retVal;

	*pAccept_frame_type = (rtk_vlan_acceptFrameType_t)acc_frm_type;

	return 0;
}

void _rtl8367c_VlanMCStUser2Smi(rtl8367c_vlanconfiguser *pVlanCg, rtl8367c_vlanconfigsmi *pSmiVlanCfg)
{
	pSmiVlanCfg->mbr        = pVlanCg->mbr;
	pSmiVlanCfg->fid_msti   = pVlanCg->fid_msti;
	pSmiVlanCfg->evid       = pVlanCg->evid;
	pSmiVlanCfg->meteridx   = pVlanCg->meteridx;
	pSmiVlanCfg->envlanpol  = pVlanCg->envlanpol;
	pSmiVlanCfg->vbpri      = pVlanCg->vbpri;
	pSmiVlanCfg->vbpen      = pVlanCg->vbpen;
}

void _rtl8367c_Vlan4kStUser2Smi(rtl8367c_user_vlan4kentry *pUserVlan4kEntry, rtl8367c_vlan4kentrysmi *pSmiVlan4kEntry)
{
	pSmiVlan4kEntry->mbr          = pUserVlan4kEntry->mbr & 0xff;
	pSmiVlan4kEntry->mbr_ext_0_1  = (pUserVlan4kEntry->mbr >> 8) & 0x3;
	pSmiVlan4kEntry->mbr_ext_2    = (pUserVlan4kEntry->mbr >> 10) & 0x1;
	pSmiVlan4kEntry->untag        = pUserVlan4kEntry->untag & 0xff;
	pSmiVlan4kEntry->untagset_ext = (pUserVlan4kEntry->untag >> 8) & 0x7;
	pSmiVlan4kEntry->fid_msti     = pUserVlan4kEntry->fid_msti;
	pSmiVlan4kEntry->vbpen        = pUserVlan4kEntry->vbpen;
	pSmiVlan4kEntry->vbpri        = pUserVlan4kEntry->vbpri;
	pSmiVlan4kEntry->envlanpol    = pUserVlan4kEntry->envlanpol;
	pSmiVlan4kEntry->meteridx     = pUserVlan4kEntry->meteridx & 0x1f;
	pSmiVlan4kEntry->mtr_idx_ext  = (pUserVlan4kEntry->meteridx >> 5) & 0x1;
	pSmiVlan4kEntry->ivl_svl      = pUserVlan4kEntry->ivl_svl;
}

void _rtl8367c_Vlan4kStSmi2User(rtl8367c_vlan4kentrysmi *pSmiVlan4kEntry, rtl8367c_user_vlan4kentry *pUserVlan4kEntry)
{
	uint16_t tmp;

	tmp = pSmiVlan4kEntry->mbr_ext_2 & 0x1;
	tmp = (tmp << 2)| pSmiVlan4kEntry->mbr_ext_0_1;
	pUserVlan4kEntry->mbr       = (tmp << 8) | pSmiVlan4kEntry->mbr;

	tmp = pSmiVlan4kEntry->untagset_ext;
	pUserVlan4kEntry->untag     = (tmp << 8) | pSmiVlan4kEntry->untag;
	pUserVlan4kEntry->fid_msti  = pSmiVlan4kEntry->fid_msti;
	pUserVlan4kEntry->vbpen     = pSmiVlan4kEntry->vbpen;
	pUserVlan4kEntry->vbpri     = pSmiVlan4kEntry->vbpri;
	pUserVlan4kEntry->envlanpol = pSmiVlan4kEntry->envlanpol;

	tmp = pSmiVlan4kEntry->mtr_idx_ext & 0x1;
	pUserVlan4kEntry->meteridx  = (tmp << 5) | pSmiVlan4kEntry->meteridx;
	pUserVlan4kEntry->ivl_svl   = pSmiVlan4kEntry->ivl_svl;
}

int rtl8367c_setAsicVlanMemberConfig(uint32_t index, rtl8367c_vlanconfiguser *pVlanCg)
{
	int retVal;
	uint32_t regAddr;
	uint32_t regData;
	uint16_t *tableAddr;
	uint32_t page_idx;
	rtl8367c_vlanconfigsmi smi_vlancfg;

	/* Error Checking  */
	if(index > RTL8367C_CVIDXMAX)
		return -1;

	if(pVlanCg->evid > RTL8367C_EVIDMAX)
		return -2;

	if(pVlanCg->mbr > RTL8367C_PORTMASK)
		return -3;

	if(pVlanCg->fid_msti > RTL8367C_FIDMAX)
		return -4;

	if(pVlanCg->meteridx > RTL8367C_METERMAX)
		return -5;

	if(pVlanCg->vbpri > RTL8367C_PRIMAX)
		return -6;

	memset(&smi_vlancfg, 0, sizeof(rtl8367c_vlanconfigsmi));
	_rtl8367c_VlanMCStUser2Smi(pVlanCg, &smi_vlancfg);
	tableAddr = (uint16_t *)&smi_vlancfg;

	for(page_idx = 0; page_idx < 4; page_idx++)  /* 4 pages per VLAN Member Config */
	{
		regAddr = RTL8367C_VLAN_MEMBER_CONFIGURATION_BASE + (index * 4) + page_idx;
		regData = *tableAddr;

		retVal = rtl8367c_setAsicReg(regAddr, regData);
		if(retVal != 0)
			return retVal;

		tableAddr++;
	}

	return 0;
}

int rtl8367c_getAsicVlan4kEntry(rtl8367c_user_vlan4kentry *pVlan4kEntry)
{
	rtl8367c_vlan4kentrysmi vlan_4k_entry;
	uint32_t page_idx;
	uint16_t *tableAddr;
	int retVal;
	uint32_t regData;
	uint32_t busyCounter;

	if(pVlan4kEntry->vid > RTL8367C_VIDMAX)
		return -1;

	/* Polling status */
	busyCounter = RTL8367C_VLAN_BUSY_CHECK_NO;
	while (busyCounter)
	{
		retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET, &regData);
		if(retVal != 0)
			return retVal;

		if(regData == 0)
			break;

		busyCounter --;
		if(busyCounter == 0) {
			Serial.println(F("rtl8367c_getAsicVlan4kEntry_0...timeout!"));
			return -2;
		}
	}

	/* Write Address (VLAN_ID) */
	regData = pVlan4kEntry->vid;
	retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_ADDR_REG, regData);
	if(retVal != 0)
		return retVal;

	/* Read Command */
	retVal = rtl8367c_setAsicRegBits(RTL8367C_TABLE_ACCESS_CTRL_REG, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK,
				RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_READ, TB_TARGET_CVLAN));
        if(retVal != 0)
                return retVal;

	/* Polling status */
	busyCounter = RTL8367C_VLAN_BUSY_CHECK_NO;
	while (busyCounter)
	{
		retVal = rtl8367c_getAsicRegBit(RTL8367C_TABLE_ACCESS_STATUS_REG, RTL8367C_TABLE_LUT_ADDR_BUSY_FLAG_OFFSET, &regData);
		if(retVal != 0)
			return retVal;

		if(regData == 0)
			break;

		busyCounter --;
		if(busyCounter == 0) {
			Serial.println(F("rtl8367c_getAsicVlan4kEntry_1...timeout!"));
			return -2;
		}
	}

	/* Read VLAN data from register */
	tableAddr = (uint16_t *)&vlan_4k_entry;
	for (page_idx = 0; page_idx < (sizeof(rtl8367c_vlan4kentrysmi) / 2); page_idx++)
	{
		retVal = rtl8367c_getAsicReg(RTL8367C_TABLE_ACCESS_RDDATA_BASE + page_idx, &regData);
		if(retVal != 0)
			return retVal;

		*tableAddr = regData;
		tableAddr++;
	}

	_rtl8367c_Vlan4kStSmi2User(&vlan_4k_entry, pVlan4kEntry);

	return 0;
}

int rtl8367c_setAsicVlan4kEntry(rtl8367c_user_vlan4kentry *pVlan4kEntry)
{
	rtl8367c_vlan4kentrysmi vlan_4k_entry;
	uint32_t page_idx;
	uint16_t *tableAddr;
	int retVal;
	uint32_t regData;

	if(pVlan4kEntry->vid > RTL8367C_VIDMAX)
		return -1;

	if(pVlan4kEntry->mbr > RTL8367C_PORTMASK)
		return -2;

	if(pVlan4kEntry->untag > RTL8367C_PORTMASK)
		return -3;

	if(pVlan4kEntry->fid_msti > RTL8367C_FIDMAX)
		return -4;

	if(pVlan4kEntry->meteridx > RTL8367C_METERMAX)
		return -5;

	if(pVlan4kEntry->vbpri > RTL8367C_PRIMAX)
		return -6;

	memset(&vlan_4k_entry, 0, sizeof(rtl8367c_vlan4kentrysmi));
	_rtl8367c_Vlan4kStUser2Smi(pVlan4kEntry, &vlan_4k_entry);

	/* Prepare Data */
	tableAddr = (uint16_t *)&vlan_4k_entry;
	for(page_idx = 0; page_idx < (sizeof(rtl8367c_vlan4kentrysmi) / 2); page_idx++)
	{
		regData = *tableAddr;
		retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_WRDATA_BASE + page_idx, regData);
		if(retVal != 0)
			return retVal;

			tableAddr++;
	}

	/* Write Address (VLAN_ID) */
	regData = pVlan4kEntry->vid;
	retVal = rtl8367c_setAsicReg(RTL8367C_TABLE_ACCESS_ADDR_REG, regData);
	if(retVal != 0)
		return retVal;

	/* Write Command */
	retVal = rtl8367c_setAsicRegBits(RTL8367C_TABLE_ACCESS_CTRL_REG, RTL8367C_TABLE_TYPE_MASK | RTL8367C_COMMAND_TYPE_MASK,
				RTL8367C_TABLE_ACCESS_REG_DATA(TB_OP_WRITE, TB_TARGET_CVLAN));
	if(retVal != 0)
		return retVal;

	return 0;
}

int rtl8367c_setAsicVlanPortBasedVID(uint32_t port, uint32_t index, uint32_t pri)
{
	uint32_t regAddr, bit_mask;
	int retVal;

	if(port > RTL8367C_PORTIDMAX)
		return -1;

	if(index > RTL8367C_CVIDXMAX)
		return -2;

	if(pri > RTL8367C_PRIMAX)
		return -3;

	regAddr = RTL8367C_VLAN_PVID_CTRL_REG(port);
	bit_mask = RTL8367C_PORT_VIDX_MASK(port);
	retVal = rtl8367c_setAsicRegBits(regAddr, bit_mask, index);
	if(retVal != 0)
		return retVal;

	regAddr = RTL8367C_VLAN_PORTBASED_PRIORITY_REG(port);
	bit_mask = RTL8367C_VLAN_PORTBASED_PRIORITY_MASK(port);
	retVal = rtl8367c_setAsicRegBits(regAddr, bit_mask, pri);
	if(retVal != 0)
		return retVal;

	return 0;
}

int rtl8367c_setAsicVlanEgressTagMode(uint32_t port, rtl8367c_egtagmode tagMode)
{
	if(port > RTL8367C_PORTIDMAX)
		return -1;

	if(tagMode >= EG_TAG_MODE_END)
		return -2;

	return rtl8367c_setAsicRegBits(RTL8367C_PORT_MISC_CFG_REG(port), RTL8367C_VLAN_EGRESS_MDOE_MASK, tagMode);
}

int rtl8367c_setAsicVlanFilter(uint32_t enabled)
{
	return rtl8367c_setAsicRegBit(RTL8367C_REG_VLAN_CTRL, RTL8367C_VLAN_CTRL_OFFSET, enabled);
}

int rtl8367c_setAsicVlanIngressFilter(uint32_t port, uint32_t enabled)
{
	if(port > RTL8367C_PORTIDMAX)
		return -1;

	return rtl8367c_setAsicRegBit(RTL8367C_VLAN_INGRESS_REG, port, enabled);
}

int rtk_vlan_init()
{
	int retVal;
	uint32_t i;
	rtl8367c_user_vlan4kentry vlan4K;
	rtl8367c_vlanconfiguser vlanMC;

	/* Clean Database */
	memset(vlan_mbrCfgVid, 0x00, sizeof(rtk_vlan_t) * RTL8367C_CVIDXNO);
	memset(vlan_mbrCfgUsage, 0x00, sizeof(vlan_mbrCfgType_t) * RTL8367C_CVIDXNO);

	/* clean 32 VLAN member configuration */
	for (i = 0; i <= RTL8367C_CVIDXMAX; i++)
	{
		vlanMC.evid = 0;
		vlanMC.mbr = 0;
		vlanMC.fid_msti = 0;
		vlanMC.envlanpol = 0;
		vlanMC.meteridx = 0;
		vlanMC.vbpen = 0;
		vlanMC.vbpri = 0;
		if ((retVal = rtl8367c_setAsicVlanMemberConfig(i, &vlanMC)) != 0)
			return retVal;
	}

	/* Set a default VLAN with vid 1 to 4K table for all ports */
	memset(&vlan4K, 0, sizeof(rtl8367c_user_vlan4kentry));
	vlan4K.vid = 1;
	vlan4K.mbr = RTK_PHY_PORTMASK_ALL;
	vlan4K.untag = RTK_PHY_PORTMASK_ALL;
	vlan4K.fid_msti = 0;
	if ((retVal = rtl8367c_setAsicVlan4kEntry(&vlan4K)) != 0)
		return retVal;

	/* Also set the default VLAN to 32 member configuration index 0 */
	memset(&vlanMC, 0, sizeof(rtl8367c_vlanconfiguser));
	vlanMC.evid = 1;
	vlanMC.mbr = RTK_PHY_PORTMASK_ALL;
	vlanMC.fid_msti = 0;
	if ((retVal = rtl8367c_setAsicVlanMemberConfig(0, &vlanMC)) != 0)
		return retVal;

	/* Set all ports PVID to default VLAN and tag-mode to original */
	for (i = 0; i < RTK_MAX_NUM_OF_PORT; i++)
	{
		if ((retVal = rtl8367c_setAsicVlanPortBasedVID(i, 0, 0)) != 0)
			return retVal;
		if ((retVal = rtl8367c_setAsicVlanEgressTagMode(i, EG_TAG_MODE_ORI)) != 0)
			return retVal;
	}

	/* Updata Databse */
	vlan_mbrCfgUsage[0] = MBRCFG_USED_BY_VLAN;
	vlan_mbrCfgVid[0] = 1;

	/* Enable Ingress filter */
	for (i = 0; i < RTK_MAX_NUM_OF_PORT; i++)
	{
		if ((retVal = rtl8367c_setAsicVlanIngressFilter(i, 1)) != 0)
			return retVal;
	}

	/* enable VLAN */
	if ((retVal = rtl8367c_setAsicVlanFilter(1)) != 0)
		return retVal;

	return 0;
}

int rtk_vlan_set(rtk_vlan_t vid, rtk_vlan_cfg_t *pVlanCfg)
{
	int retVal;
	uint32_t phyMbrPmask;
	uint32_t phyUntagPmask;
	rtl8367c_user_vlan4kentry vlan4K;
	rtl8367c_vlanconfiguser vlanMC;
	uint32_t idx;
	uint32_t empty_index = 0xffff;
	uint32_t update_evid = 0;

	/* vid must be 0~8191 */
	if (vid > RTL8367C_EVIDMAX)
		return -1;

	/* Null pointer check */
	if(NULL == pVlanCfg)
		return -10;

	if (vid > RTL8367C_VIDMAX)
	{
		/* Check untag port mask valid */
		return -2;
	}

	/* IVL_EN */
	if(pVlanCfg->ivl_en >= RTK_ENABLE_END)
		return -3;

	/* fid must be 0~15 */
	if(pVlanCfg->fid_msti > RTL8367C_FIDMAX)
		return -4;

	/* Policing */
	if(pVlanCfg->envlanpol >= RTK_ENABLE_END)
		return -3;

	/* Meter ID */
	if(pVlanCfg->meteridx > RTK_MAX_METER_ID)
		return -5;

	/* VLAN based priority */
	if(pVlanCfg->vbpen >= RTK_ENABLE_END)
		return -3;

	/* Priority */
	if(pVlanCfg->vbpri > RTL8367C_PRIMAX)
		return -4;

	/* Get physical port mask */
	if(rtk_switch_portmask_L2P_get(&(pVlanCfg->mbr), &phyMbrPmask) != 0)
		return -6;

	if(rtk_switch_portmask_L2P_get(&(pVlanCfg->untag), &phyUntagPmask) != 0)
		return -6;

	if (vid <= RTL8367C_VIDMAX)
	{
		/* update 4K table */
		memset(&vlan4K, 0, sizeof(rtl8367c_user_vlan4kentry));
		vlan4K.vid = vid;

		vlan4K.mbr    = (phyMbrPmask & 0xFFFF);
		vlan4K.untag  = (phyUntagPmask & 0xFFFF);

		vlan4K.ivl_svl      = pVlanCfg->ivl_en;
		vlan4K.fid_msti     = pVlanCfg->fid_msti;
		vlan4K.envlanpol    = pVlanCfg->envlanpol;
		vlan4K.meteridx     = pVlanCfg->meteridx;
		vlan4K.vbpen        = pVlanCfg->vbpen;
		vlan4K.vbpri        = pVlanCfg->vbpri;

		if ((retVal = rtl8367c_setAsicVlan4kEntry(&vlan4K)) != 0)
			return retVal;

		/* Update Member configuration if exist */
		for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
		{
			if(vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
			{
				if(vlan_mbrCfgVid[idx] == vid)
				{
					/* Found! Update */
					if(phyMbrPmask == 0)
					{
						/* Member port = 0x00, delete this VLAN from Member Configuration */
						memset(&vlanMC, 0, sizeof(rtl8367c_vlanconfiguser));
						if ((retVal = rtl8367c_setAsicVlanMemberConfig(idx, &vlanMC)) != 0)
							return retVal;

						/* Clear Database */
						vlan_mbrCfgUsage[idx] = MBRCFG_UNUSED;
						vlan_mbrCfgVid[idx]   = 0;
					}
					else
					{
						/* Normal VLAN config, update to member configuration */
						vlanMC.evid = vid;
						vlanMC.mbr = vlan4K.mbr;
						vlanMC.fid_msti = vlan4K.fid_msti;
						vlanMC.meteridx = vlan4K.meteridx;
						vlanMC.envlanpol= vlan4K.envlanpol;
						vlanMC.vbpen = vlan4K.vbpen;
						vlanMC.vbpri = vlan4K.vbpri;
						if ((retVal = rtl8367c_setAsicVlanMemberConfig(idx, &vlanMC)) != 0)
							return retVal;
					}

					break;
				}
			}
		}
	}
	else
	{
		/* vid > 4095 */
		for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
		{
			if(vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
			{
				if(vlan_mbrCfgVid[idx] == vid)
				{
					/* Found! Update */
					if(phyMbrPmask == 0)
					{
						/* Member port = 0x00, delete this VLAN from Member Configuration */
						memset(&vlanMC, 0, sizeof(rtl8367c_vlanconfiguser));
						if ((retVal = rtl8367c_setAsicVlanMemberConfig(idx, &vlanMC)) != 0)
							return retVal;

						/* Clear Database */
						vlan_mbrCfgUsage[idx] = MBRCFG_UNUSED;
						vlan_mbrCfgVid[idx]   = 0;
					}
					else
					{
						/* Normal VLAN config, update to member configuration */
						vlanMC.evid = vid;
						vlanMC.mbr = phyMbrPmask;
						vlanMC.fid_msti = pVlanCfg->fid_msti;
						vlanMC.meteridx = pVlanCfg->meteridx;
						vlanMC.envlanpol= pVlanCfg->envlanpol;
						vlanMC.vbpen = pVlanCfg->vbpen;
						vlanMC.vbpri = pVlanCfg->vbpri;
						if ((retVal = rtl8367c_setAsicVlanMemberConfig(idx, &vlanMC)) != 0)
							return retVal;

						break;
					}

					update_evid = 1;
				}
			}

			if(vlan_mbrCfgUsage[idx] == MBRCFG_UNUSED)
			{
				if(0xffff == empty_index)
					empty_index = idx;
			}
		}

		/* doesn't find out same EVID entry and there is empty index in member configuration */
		if( (phyMbrPmask != 0) && (update_evid == 0) && (empty_index != 0xFFFF) )
		{
			vlanMC.evid = vid;
			vlanMC.mbr = phyMbrPmask;
			vlanMC.fid_msti = pVlanCfg->fid_msti;
			vlanMC.meteridx = pVlanCfg->meteridx;
			vlanMC.envlanpol= pVlanCfg->envlanpol;
			vlanMC.vbpen = pVlanCfg->vbpen;
			vlanMC.vbpri = pVlanCfg->vbpri;
			if ((retVal = rtl8367c_setAsicVlanMemberConfig(empty_index, &vlanMC)) != 0)
				return retVal;

			vlan_mbrCfgUsage[empty_index] = MBRCFG_USED_BY_VLAN;
			vlan_mbrCfgVid[empty_index] = vid;

		}
	}

	return 0;
}

int rtk_switch_portmask_P2L_get(uint32_t physicalPortmask, rtk_portmask_t *pLogicalPmask)
{
	uint32_t phy_port;

	if(NULL == pLogicalPmask)
		return -10;

	RTK_PORTMASK_CLEAR(*pLogicalPmask);

	for(phy_port = 0; phy_port < MAX_PORTS; phy_port++)
	{
		if(physicalPortmask & (0x0001 << phy_port))
		{
			RTK_PORTMASK_PORT_SET(*pLogicalPmask, phy_port);
		}
	}

	return 0;
}

int rtk_vlan_get(rtk_vlan_t vid, rtk_vlan_cfg_t *pVlanCfg)
{
	int retVal;
	uint32_t phyMbrPmask;
	uint32_t phyUntagPmask;
	rtl8367c_user_vlan4kentry vlan4K;
	rtl8367c_vlanconfiguser vlanMC;
	uint32_t idx;

	/* vid must be 0~8191 */
	if (vid > RTL8367C_EVIDMAX)
		return -1;

	/* Null pointer check */
	if(NULL == pVlanCfg)
		return -10;

	if (vid <= RTL8367C_VIDMAX)
	{
		vlan4K.vid = vid;

		if ((retVal = rtl8367c_getAsicVlan4kEntry(&vlan4K)) != 0)
			return retVal;

		phyMbrPmask   = vlan4K.mbr;
		phyUntagPmask = vlan4K.untag;

		if(rtk_switch_portmask_P2L_get(phyMbrPmask, &(pVlanCfg->mbr)) != 0)
			return -2;

		if(rtk_switch_portmask_P2L_get(phyUntagPmask, &(pVlanCfg->untag)) != 0)
			return -2;

		pVlanCfg->ivl_en    = vlan4K.ivl_svl;
		pVlanCfg->fid_msti  = vlan4K.fid_msti;
		pVlanCfg->envlanpol = vlan4K.envlanpol;
		pVlanCfg->meteridx  = vlan4K.meteridx;
		pVlanCfg->vbpen     = vlan4K.vbpen;
		pVlanCfg->vbpri     = vlan4K.vbpri;
	}
	else
	{
		for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
		{
			if(vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
			{
				if(vlan_mbrCfgVid[idx] == vid)
				{
					if ((retVal = rtl8367c_getAsicVlanMemberConfig(idx, &vlanMC)) != 0)
						return retVal;

					phyMbrPmask   = vlanMC.mbr;

					if(rtk_switch_portmask_P2L_get(phyMbrPmask, &(pVlanCfg->mbr)) != 0)
						return -2;

					pVlanCfg->untag.bits[0] = 0;
					pVlanCfg->ivl_en    = 0;
					pVlanCfg->fid_msti  = vlanMC.fid_msti;
					pVlanCfg->envlanpol = vlanMC.envlanpol;
					pVlanCfg->meteridx  = vlanMC.meteridx;
					pVlanCfg->vbpen     = vlanMC.vbpen;
					pVlanCfg->vbpri     = vlanMC.vbpri;
				}
			}
		}
	}

	return 0;
}

int rtk_vlan_checkAndCreateMbr(rtk_vlan_t vid, uint32_t *pIndex)
{
	int retVal;
	rtl8367c_user_vlan4kentry vlan4K;
	rtl8367c_vlanconfiguser vlanMC;
	uint32_t idx;
	uint32_t empty_idx = 0xFFFF;

	/* vid must be 0~8191 */
	if (vid > RTL8367C_EVIDMAX)
		return -1;

	/* Null pointer check */
	if(NULL == pIndex)
		return -10;

	/* Get 4K VLAN */
	if (vid <= RTL8367C_VIDMAX)
	{
		memset(&vlan4K, 0, sizeof(rtl8367c_user_vlan4kentry));
		vlan4K.vid = vid;
		if ((retVal = rtl8367c_getAsicVlan4kEntry(&vlan4K)) != 0)
			return retVal;
	}

	/* Search exist entry */
	for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
	{
		if(vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
		{
			if(vlan_mbrCfgVid[idx] == vid)
			{
				/* Found! return index */
				*pIndex = idx;
				return 0;
			}
		}
	}

	/* Not found, Read H/W Member Configuration table to update database */
	for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
	{
		if ((retVal = rtl8367c_getAsicVlanMemberConfig(idx, &vlanMC)) != 0)
			return retVal;

		if( (vlanMC.evid == 0) && (vlanMC.mbr == 0))
		{
			vlan_mbrCfgUsage[idx]   = MBRCFG_UNUSED;
			vlan_mbrCfgVid[idx]     = 0;
		}
		else
		{
			vlan_mbrCfgUsage[idx]   = MBRCFG_USED_BY_VLAN;
			vlan_mbrCfgVid[idx]     = vlanMC.evid;
		}
	}

	/* Search exist entry again */
	for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
	{
		if(vlan_mbrCfgUsage[idx] == MBRCFG_USED_BY_VLAN)
		{
			if(vlan_mbrCfgVid[idx] == vid)
			{
				/* Found! return index */
				*pIndex = idx;
				return 0;
			}
		}
	}

	/* try to look up an empty index */
	for (idx = 0; idx <= RTL8367C_CVIDXMAX; idx++)
	{
		if(vlan_mbrCfgUsage[idx] == MBRCFG_UNUSED)
		{
			empty_idx = idx;
			break;
		}
	}
	if(empty_idx == 0xFFFF)
	{
		/* No empty index */
		return -2;
	}

	if (vid > RTL8367C_VIDMAX)
	{
		/* > 4K, there is no 4K entry, create on member configuration directly */
		memset(&vlanMC, 0x00, sizeof(rtl8367c_vlanconfiguser));
		vlanMC.evid = vid;
		if ((retVal = rtl8367c_setAsicVlanMemberConfig(empty_idx, &vlanMC)) != 0)
			return retVal;
	}
	else
	{
		/* Copy from 4K table */
		vlanMC.evid = vid;
		vlanMC.mbr = vlan4K.mbr;
		vlanMC.fid_msti = vlan4K.fid_msti;
		vlanMC.meteridx= vlan4K.meteridx;
		vlanMC.envlanpol= vlan4K.envlanpol;
		vlanMC.vbpen = vlan4K.vbpen;
		vlanMC.vbpri = vlan4K.vbpri;
		if ((retVal = rtl8367c_setAsicVlanMemberConfig(empty_idx, &vlanMC)) != 0)
			return retVal;
	}

	/* Update Database */
	vlan_mbrCfgUsage[empty_idx] = MBRCFG_USED_BY_VLAN;
	vlan_mbrCfgVid[empty_idx] = vid;

	*pIndex = empty_idx;
	return 0;
}

int rtk_vlan_portPvid_set(uint32_t port, rtk_vlan_t pvid, uint32_t priority)
{
	int retVal;
	uint32_t index;

	/* vid must be 0~8191 */
	if (pvid > RTL8367C_EVIDMAX)
		return -1;

	/* priority must be 0~7 */
	if (priority > RTL8367C_PRIMAX)
		return -2;

	if((retVal = rtk_vlan_checkAndCreateMbr(pvid, &index)) != 0)
		return retVal;

	if ((retVal = rtl8367c_setAsicVlanPortBasedVID(port, index, priority)) != 0)
		return retVal;

	return 0;
}

int rtk_vlan_portPvid_get(uint32_t port, rtk_vlan_t *pPvid, uint32_t *pPriority)
{
	int retVal;
	uint32_t index, pri;
	rtl8367c_vlanconfiguser mbrCfg;

	if(NULL == pPvid)
		return -10;

	if(NULL == pPriority)
		return -10;

	if ((retVal = rtl8367c_getAsicVlanPortBasedVID(port, &index, &pri)) != 0)
		return retVal;

	memset(&mbrCfg, 0, sizeof(rtl8367c_vlanconfiguser));
	if ((retVal = rtl8367c_getAsicVlanMemberConfig(index, &mbrCfg)) != 0)
		return retVal;

	*pPvid = mbrCfg.evid;
	*pPriority = pri;

	return 0;
}
/* End rtl8367c_asicdrv_vlan */

/* rtl8367c_asicdrv_qos */
#define RTK_PRIMAX						7
#define RTK_MAX_NUM_OF_QUEUE					8
#define RTL8367C_QUEUENO					8
#define RTL8367C_REMARKING_DSCP_ENABLE_OFFSET			8
#define RTL8367C_1QREMARK_ENABLE_OFFSET				12
#define RTL8367C_FLOWCTRL_TYPE_OFFSET				15
#define RTL8367C_DSCPMAX					63
#define RTL8367C_DECISIONPRIMAX					0xff
#define RTL8367C_QIDMAX						(RTL8367C_QUEUENO - 1)
#define RTK_SCAN_ALL_PHY_PORTMASK(__port__)			for(__port__ = 0; __port__ < MAX_PORTS; __port__++)
#define RTL8367C_REG_FLOWCTRL_CTRL0				0x121d
#define RTL8367C_QOS_PORT_QUEUE_NUMBER_REG(port)		(0x0900 + (port >> 2))
#define RTL8367C_QOS_PORT_QUEUE_NUMBER_OFFSET(port)		((port & 0x3) << 2)
#define RTL8367C_QOS_1Q_PRIORITY_TO_QID_OFFSET(pri)		((pri & 0x3) << 2)
#define RTL8367C_QOS_PORT_QUEUE_NUMBER_MASK(port)		(7 << RTL8367C_QOS_PORT_QUEUE_NUMBER_OFFSET(port))
#define RTL8367C_QOS_1Q_PRIORITY_TO_QID_REG(index, pri)		(0x0904 + (index << 1) + (pri >> 2))
#define RTL8367C_QOS_1Q_PRIORITY_TO_QID_MASK(pri)		(7 << RTL8367C_QOS_1Q_PRIORITY_TO_QID_OFFSET(pri))
#define RTL8367C_QOS_INTERNAL_PRIORITY_DECISION_REG(src)	(0x087b + (src >> 1))
#define RTL8367C_QOS_INTERNAL_PRIORITY_DECISION2_OFFSET(src)	((src & 1) << 3)
#define RTL8367C_QOS_INTERNAL_PRIORITY_DECISION_MASK(src)	(0xff << RTL8367C_QOS_INTERNAL_PRIORITY_DECISION2_OFFSET(src))
#define RTL8367C_QOS_INTERNAL_PRIORITY_DECISION2_REG(src)	(0x0885 + (src >> 1))
#define RTL8367C_QOS_INTERNAL_PRIORITY_DECISION2_MASK(src)	RTL8367C_QOS_INTERNAL_PRIORITY_DECISION_MASK(src)
#define RTL8367C_QOS_PORTBASED_PRIORITY_REG(port)		(0x0877 + (port >> 2))
#define RTL8367C_QOS_PORTBASED_PRIORITY_OFFSET(port)		((port & 0x3) << 2)
#define RTL8367C_QOS_PORTBASED_PRIORITY_MASK(port)		(7 << RTL8367C_QOS_PORTBASED_PRIORITY_OFFSET(port))
#define RTL8367C_REG_QOS_PORTBASED_PRIORITY_CTRL2		0x087a
#define RTL8367C_REMARKING_CTRL_REG				0x120c
#define RTL8367C_QOS_1Q_PRIORITY_REMAPPING_REG(pri)		(0x0865 + (pri >> 2))
#define RTL8367C_QOS_1Q_PRIORITY_REMAPPING_OFFSET(pri)		((pri & 0x3) << 2)
#define RTL8367C_QOS_1Q_PRIORITY_REMAPPING_MASK(pri)		(7 << RTL8367C_QOS_1Q_PRIORITY_REMAPPING_OFFSET(pri))
#define RTL8367C_QOS_1Q_REMARK_REG(pri)				(0x1211 + (pri >> 2))
#define RTL8367C_QOS_1Q_REMARK_OFFSET(pri)			((pri & 0x3) << 2)
#define RTL8367C_QOS_1Q_REMARK_MASK(pri)			(7 << RTL8367C_QOS_1Q_REMARK_OFFSET(pri))
#define RTL8367C_QOS_DSCP_REMARK_REG(pri)			(0x120d + (pri >> 1))
#define RTL8367C_QOS_DSCP_REMARK_OFFSET(pri)			(((pri) & 0x1) << 3)
#define RTL8367C_QOS_DSCP_REMARK_MASK(pri)			(0x3F << RTL8367C_QOS_DSCP_REMARK_OFFSET(pri))
#define RTL8367C_QOS_DSCP_TO_PRIORITY_REG(dscp)			(0x0867 + (dscp >> 2))
#define RTL8367C_QOS_DSCP_TO_PRIORITY_OFFSET(dscp)		((dscp & 0x3) << 2)
#define RTL8367C_QOS_DSCP_TO_PRIORITY_MASK(dscp)		(7 << RTL8367C_QOS_DSCP_TO_PRIORITY_OFFSET(dscp))

int rtl8367c_setAsicFlowControlSelect(uint32_t select)
{
	return rtl8367c_setAsicRegBit(RTL8367C_REG_FLOWCTRL_CTRL0, RTL8367C_FLOWCTRL_TYPE_OFFSET, select);
}

int rtl8367c_setAsicOutputQueueMappingIndex(uint32_t port, uint32_t index)
{
	if(port > RTL8367C_PORTIDMAX)
		return -2;

	if(index >= RTL8367C_QUEUENO)
		return -3;

	return rtl8367c_setAsicRegBits(RTL8367C_QOS_PORT_QUEUE_NUMBER_REG(port), RTL8367C_QOS_PORT_QUEUE_NUMBER_MASK(port), index);
}

int rtl8367c_setAsicPriorityToQIDMappingTable(uint32_t index, uint32_t priority, uint32_t qid)
{
	if(index >= RTL8367C_QUEUENO)
		return -2;

	if(priority > RTL8367C_PRIMAX)
		return -3;

	if(qid > RTL8367C_QIDMAX)
		return -4;

	return rtl8367c_setAsicRegBits(RTL8367C_QOS_1Q_PRIORITY_TO_QID_REG(index, priority), RTL8367C_QOS_1Q_PRIORITY_TO_QID_MASK(priority), qid);
}

int rtl8367c_setAsicPriorityDecision(uint32_t index, uint32_t prisrc, uint32_t decisionPri)
{
	int retVal;

	if(index >= PRIDEC_IDX_END)
		return -2;

	if(prisrc >= PRIDEC_END)
		return -3;

	if(decisionPri > RTL8367C_DECISIONPRIMAX)
		return -4;

	switch(index)
	{
		case PRIDEC_IDX0:
			if((retVal = rtl8367c_setAsicRegBits(RTL8367C_QOS_INTERNAL_PRIORITY_DECISION_REG(prisrc), RTL8367C_QOS_INTERNAL_PRIORITY_DECISION_MASK(prisrc), decisionPri)) != 0)
				return retVal;
			break;
		case PRIDEC_IDX1:
			if((retVal = rtl8367c_setAsicRegBits(RTL8367C_QOS_INTERNAL_PRIORITY_DECISION2_REG(prisrc), RTL8367C_QOS_INTERNAL_PRIORITY_DECISION2_MASK(prisrc), decisionPri)) != 0)
				return retVal;
			break;
		default:
			break;
	};

	return 0;
}

int rtl8367c_setAsicPriorityPortBased(uint32_t port, uint32_t priority)
{
	int retVal;

	if(port > RTL8367C_PORTIDMAX)
		return -2;

	if(priority > RTL8367C_PRIMAX )
		return -3;

	if(port < 8)
	{
		retVal = rtl8367c_setAsicRegBits(RTL8367C_QOS_PORTBASED_PRIORITY_REG(port), RTL8367C_QOS_PORTBASED_PRIORITY_MASK(port), priority);
		if(retVal != 0)
			return retVal;
	}
	else
	{
		retVal = rtl8367c_setAsicRegBits(RTL8367C_REG_QOS_PORTBASED_PRIORITY_CTRL2, 0x7 << ((port - 8) << 2), priority);
		if(retVal != 0)
			return retVal;
	}

	return 0;
}

int rtl8367c_setAsicRemarkingDot1pAbility(uint32_t port, uint32_t enabled)
{
	return rtl8367c_setAsicRegBit(RTL8367C_PORT_MISC_CFG_REG(port), RTL8367C_1QREMARK_ENABLE_OFFSET, enabled);
}

int rtl8367c_setAsicRemarkingDscpAbility(uint32_t enabled)
{
	return rtl8367c_setAsicRegBit(RTL8367C_REMARKING_CTRL_REG, RTL8367C_REMARKING_DSCP_ENABLE_OFFSET, enabled);
}

int rtl8367c_setAsicPriorityDot1qRemapping(uint32_t srcpriority, uint32_t priority)
{
	if((srcpriority > RTL8367C_PRIMAX) || (priority > RTL8367C_PRIMAX))
		return -2;

	return rtl8367c_setAsicRegBits(RTL8367C_QOS_1Q_PRIORITY_REMAPPING_REG(srcpriority), RTL8367C_QOS_1Q_PRIORITY_REMAPPING_MASK(srcpriority), priority);
}

int rtl8367c_setAsicRemarkingDot1pParameter(uint32_t priority, uint32_t newPriority)
{
	if(priority > RTL8367C_PRIMAX || newPriority > RTL8367C_PRIMAX)
		return -2;

	return rtl8367c_setAsicRegBits(RTL8367C_QOS_1Q_REMARK_REG(priority), RTL8367C_QOS_1Q_REMARK_MASK(priority), newPriority);
}

int rtl8367c_setAsicRemarkingDscpParameter(uint32_t priority, uint32_t newDscp)
{
	if(priority > RTL8367C_PRIMAX)
		return -2;

	if(newDscp > RTL8367C_DSCPMAX)
		return -3;

	return rtl8367c_setAsicRegBits(RTL8367C_QOS_DSCP_REMARK_REG(priority), RTL8367C_QOS_DSCP_REMARK_MASK(priority), newDscp);
}

int rtl8367c_setAsicPriorityDscpBased(uint32_t dscp, uint32_t priority)
{
	if(priority > RTL8367C_PRIMAX )
		return -2;

	if(dscp > RTL8367C_DSCPMAX)
		return -3;

	return rtl8367c_setAsicRegBits(RTL8367C_QOS_DSCP_TO_PRIORITY_REG(dscp), RTL8367C_QOS_DSCP_TO_PRIORITY_MASK(dscp), priority);
}

int rtk_qos_init(uint32_t queueNum)
{
	const uint16_t g_prioritytToQid[8][8] = {
		{0, 0, 0, 0, 0, 0, 0, 0},
		{0, 0, 0, 0, 7, 7, 7, 7},
		{0, 0, 0, 0, 1, 1, 7, 7},
		{0, 0, 1, 1, 2, 2, 7, 7},
		{0, 0, 1, 1, 2, 3, 7, 7},
		{0, 0, 1, 2, 3, 4, 7, 7},
		{0, 0, 1, 2, 3, 4, 5, 7},
		{0, 1, 2, 3, 4, 5, 6, 7}
	};
	const uint32_t g_priorityDecision[8] = {0x01, 0x80, 0x04, 0x02, 0x20, 0x40, 0x10, 0x08};
	const uint32_t g_prioritytRemap[8] = {0, 1, 2, 3, 4, 5, 6, 7};

	int retVal;
	uint32_t qmapidx;
	uint32_t priority;
	uint32_t priDec;
	uint32_t port;
	uint32_t dscp;

	if (queueNum <= 0 || queueNum > RTK_MAX_NUM_OF_QUEUE)
		return -2;

	/* Set Output Queue Number */
	if (RTK_MAX_NUM_OF_QUEUE == queueNum)
		qmapidx = 0;
	else
		qmapidx = queueNum;

	RTK_SCAN_ALL_PHY_PORTMASK(port)
	{
		if ((retVal = rtl8367c_setAsicOutputQueueMappingIndex(port, qmapidx)) != 0)
			return retVal;
	}

	/* Set Priority to Qid */
	for (priority = 0; priority <= RTK_PRIMAX; priority++)
	{
		if ((retVal = rtl8367c_setAsicPriorityToQIDMappingTable(queueNum - 1, priority, g_prioritytToQid[queueNum - 1][priority])) != 0)
			return retVal;
	}

	/* Set Flow Control Type to Ingress Flow Control */
	if ((retVal = rtl8367c_setAsicFlowControlSelect(FC_INGRESS)) != 0)
		return retVal;


	/* Priority Decision Order */
	for (priDec = 0; priDec < PRIDEC_END; priDec++)
	{
		if ((retVal = rtl8367c_setAsicPriorityDecision(PRIDECTBL_IDX0, priDec, g_priorityDecision[priDec])) != 0)
			return retVal;
		if ((retVal = rtl8367c_setAsicPriorityDecision(PRIDECTBL_IDX1, priDec, g_priorityDecision[priDec])) != 0)
			return retVal;
	}

	/* Set Port-based Priority to 0 */
	RTK_SCAN_ALL_PHY_PORTMASK(port)
	{
		if ((retVal = rtl8367c_setAsicPriorityPortBased(port, 0)) != 0)
			return retVal;
	}

	/* Disable 1p Remarking */
	RTK_SCAN_ALL_PHY_PORTMASK(port)
	{
		if ((retVal = rtl8367c_setAsicRemarkingDot1pAbility(port, 0)) != 0)
			return retVal;
	}

	/* Disable DSCP Remarking */
	if ((retVal = rtl8367c_setAsicRemarkingDscpAbility(0)) != 0)
		return retVal;

	/* Set 1p & DSCP  Priority Remapping & Remarking */
	for (priority = 0; priority <= RTL8367C_PRIMAX; priority++)
	{
		if ((retVal = rtl8367c_setAsicPriorityDot1qRemapping(priority, g_prioritytRemap[priority])) != 0)
			return retVal;

		if ((retVal = rtl8367c_setAsicRemarkingDot1pParameter(priority, 0)) != 0)
			return retVal;

		if ((retVal = rtl8367c_setAsicRemarkingDscpParameter(priority, 0)) != 0)
			return retVal;
	}

	/* Set DSCP Priority */
	for (dscp = 0; dscp <= 63; dscp++)
	{
		if ((retVal = rtl8367c_setAsicPriorityDscpBased(dscp, 0)) != 0)
			return retVal;
	}

	return 0;
}
/* End rtl8367c_asicdrv_qos */

/* rtk_switch */
#define RTL8367C_VLAN_EXT_CTRL2_OFFSET					0
#define RTL8367C_SCHEDULE_WFQ_CTRL_OFFSET				0
#define RTL8367C_ACL_ACCESS_MODE_MASK					1
#define RTL8367C_INT_EN_OFFSET						1
#define RTL8367C_MAX_LEN_RX_TX_MASK					3
#define RTL8367C_LUT_IPMC_LOOKUP_OP_OFFSET				3
#define RTL8367C_INGRESSBW_PORT0_RATE_CTRL1_INGRESSBW_RATE16_MASK	7
#define RTL8367C_PORT6_EGRESSBW_CTRL1_MASK				7
#define RTL8367C_PORT0_EEECFG_EEE_RX_OFFSET				8
#define RTL8367C_PORT0_EEECFG_EEE_TX_OFFSET				9
#define RTL8367C_PORT0_EEECFG_EEE_GIGA_500M_OFFSET			10
#define RTL8367C_PORT0_MISC_CFG_INGRESSBW_IFG_OFFSET			10
#define RTL8367C_PORT0_EEECFG_EEE_100M_OFFSET				11
#define RTL8367C_PORT0_MISC_CFG_INGRESSBW_FLOWCTRL_OFFSET		11
#define RTL8367C_QOS_GRANULARTY_MSB_OFFSET				16
#define RTL8367C_PORT_MISC_CFG_BASE					0x000e
#define RTL8367C_INGRESSBW_PORT_RATE_LSB_BASE				0x000f
#define RTL8367C_REG_PORT0_EEECFG					0x0018
#define RTL8367C_TRAP_PRIORITY_MASK					0x38
#define RTL8367C_UNKNOWN_UNICAST_DA_BEHAVE_MASK				0xc0
#define RTL8367C_REG_SCHEDULE_WFQ_CTRL					0x0300
#define RTL8367C_PORT_EGRESSBW_LSB_BASE					0x038c
#define RTL8367C_PORT_EGRESSBW_MSB_BASE					0x038d
#define RTL8367C_REG_ACL_ACCESS_MODE					0x06eb
#define RTL8367C_REG_VLAN_EXT_CTRL2					0x07b6
#define RTL8367C_REG_RMA_CTRL00						0x0800
#define RTL8367C_REG_MAX_LEN_RX_TX					0x0884
#define RTL8367C_PORT_SECURIT_CTRL_REG					0x08c8
#define RTL8367C_REG_LUT_CFG						0x0a30
#define RTL8367C_REG_MAX_LENGTH_LIMINT_IPG				0x1200
#define RTL8367C_REG_UTP_FIB_DET					0x13eb
#define RTL8367C_REG_IO_MISC_FUNC					0x1d32
#define RTL8367C_MAX_LENTH_CTRL_MASK					0x6000
#define RTL8367C_QOS_GRANULARTY_LSB_MASK				0xffff
#define RTL8367C_QOS_GRANULARTY_MSB_MASK				0x70000
#define RTL8367C_QOS_RATE_INPUT_MAX_HSG					(0x7ffff * 8)

int rtk_switch_init_8367c()
{
	uint32_t regData, regAddr;
	int i;

	for (i = 0; i < MAX_PORTS; i++) {
		if (rtl8367c_setAsicRegBit(RTL8367C_REG_PORT0_EEECFG + (0x20 * i), RTL8367C_PORT0_EEECFG_EEE_100M_OFFSET, 1) != 0)
			return -1;
		if (rtl8367c_setAsicRegBit(RTL8367C_REG_PORT0_EEECFG + (0x20 * i), RTL8367C_PORT0_EEECFG_EEE_GIGA_500M_OFFSET, 1) != 0)
			return -1;
		if (rtl8367c_setAsicRegBit(RTL8367C_REG_PORT0_EEECFG + (0x20 * i), RTL8367C_PORT0_EEECFG_EEE_TX_OFFSET, 1) != 0)
			return -1;
		if (rtl8367c_setAsicRegBit(RTL8367C_REG_PORT0_EEECFG + (0x20 * i), RTL8367C_PORT0_EEECFG_EEE_RX_OFFSET, 1) != 0)
			return -1;
		if (rtl8367c_getAsicPHYOCPReg(i, 0xa428, &regData) != 0)
			return -1;
		regData &= ~(0x0200);
		if (rtl8367c_setAsicPHYOCPReg(i, 0xa428, regData) != 0)
			return -1;
	}

	rtl8367c_setAsicReg(RTL8367C_REG_UTP_FIB_DET, 0x15bb);
	rtl8367c_setAsicReg(0x1303, 0x06d6);
	rtl8367c_setAsicReg(0x1304, 0x0700);
	rtl8367c_setAsicReg(0x13e2, 0x003f);
	rtl8367c_setAsicReg(0x13f9, 0x0090);
	rtl8367c_setAsicReg(0x121e, 0x03ca);
	rtl8367c_setAsicReg(0x1233, 0x0352);
	rtl8367c_setAsicReg(0x1237, 0x00a0);
	rtl8367c_setAsicReg(0x123a, 0x0030);
	rtl8367c_setAsicReg(0x1239, 0x0084);
	rtl8367c_setAsicReg(0x0301, 0x1000);
	rtl8367c_setAsicRegBit(0x18e0, 0, 0);
	rtl8367c_setAsicRegBit(0x122b, 14, 1);
	rtl8367c_setAsicRegBits(0x1305, 0xc000, 3);

	/* Set Old max packet length to 16K */
	rtl8367c_setAsicRegBits(RTL8367C_REG_MAX_LENGTH_LIMINT_IPG, RTL8367C_MAX_LENTH_CTRL_MASK, 3);
	rtl8367c_setAsicRegBits(RTL8367C_REG_MAX_LEN_RX_TX, RTL8367C_MAX_LEN_RX_TX_MASK, 3);

	/* ACL Mode */
	rtl8367c_setAsicRegBits(RTL8367C_REG_ACL_ACCESS_MODE, RTL8367C_ACL_ACCESS_MODE_MASK, 1);

	/* Max rate */
	for (i = 0; i < MAX_PORTS; i++) {
		regAddr = RTL8367C_INGRESSBW_PORT_RATE_LSB_BASE + (i << 5);
		regData = (RTL8367C_QOS_RATE_INPUT_MAX_HSG >> 3) & RTL8367C_QOS_GRANULARTY_LSB_MASK;
		rtl8367c_setAsicReg(regAddr, regData);
		regAddr += 1;
		regData = ((RTL8367C_QOS_RATE_INPUT_MAX_HSG >> 3) & RTL8367C_QOS_GRANULARTY_MSB_MASK) >> RTL8367C_QOS_GRANULARTY_MSB_OFFSET;
		rtl8367c_setAsicRegBits(regAddr, RTL8367C_INGRESSBW_PORT0_RATE_CTRL1_INGRESSBW_RATE16_MASK, regData);
		regAddr = RTL8367C_PORT_MISC_CFG_BASE + (i << 5);
		rtl8367c_setAsicRegBit(regAddr, RTL8367C_PORT0_MISC_CFG_INGRESSBW_IFG_OFFSET, 0);
		rtl8367c_setAsicRegBit(regAddr, RTL8367C_PORT0_MISC_CFG_INGRESSBW_FLOWCTRL_OFFSET, 1);
		regAddr = 0;
	}

	for (i = 0; i < MAX_PORTS; i++) {
		regAddr = RTL8367C_PORT_EGRESSBW_LSB_BASE + (i << 1);
		regData = RTL8367C_QOS_GRANULARTY_LSB_MASK & (RTL8367C_QOS_RATE_INPUT_MAX_HSG >> 3);
		rtl8367c_setAsicReg(regAddr, regData);
		regAddr = RTL8367C_PORT_EGRESSBW_MSB_BASE + (i << 1);
		regData = (RTL8367C_QOS_GRANULARTY_MSB_MASK & (RTL8367C_QOS_RATE_INPUT_MAX_HSG >> 3)) >> RTL8367C_QOS_GRANULARTY_MSB_OFFSET;
		rtl8367c_setAsicRegBits(regAddr, RTL8367C_PORT6_EGRESSBW_CTRL1_MASK, regData);
		rtl8367c_setAsicRegBit(RTL8367C_REG_SCHEDULE_WFQ_CTRL, RTL8367C_SCHEDULE_WFQ_CTRL_OFFSET, 1);
	}

	/* Change unknown DA to per port setting */
	rtl8367c_setAsicRegBits(RTL8367C_PORT_SECURIT_CTRL_REG, RTL8367C_UNKNOWN_UNICAST_DA_BEHAVE_MASK, 3);

	/* LUT lookup OP = 1 */
	rtl8367c_setAsicRegBit(RTL8367C_REG_LUT_CFG, RTL8367C_LUT_IPMC_LOOKUP_OP_OFFSET, 1);

	/* Set RMA */
	rtl8367c_setAsicRegBits(RTL8367C_REG_RMA_CTRL00, RTL8367C_TRAP_PRIORITY_MASK, 0);
	rtl8367c_setAsicReg(RTL8367C_REG_RMA_CTRL00 + 2, 0);

	/* INT EN */
	rtl8367c_setAsicRegBit(RTL8367C_REG_IO_MISC_FUNC, RTL8367C_INT_EN_OFFSET, 1);

	/* Reset VLAN Table */
	rtl8367c_setAsicRegBit(RTL8367C_REG_VLAN_EXT_CTRL2, RTL8367C_VLAN_EXT_CTRL2_OFFSET, 1);

	if (rtk_led_init() != 0)
		return -1;

	return 0;
}
/* End rtk_switch */

void rtl8367c_link_status()
{
	uint32_t val, lr_speed;

	Serial.println(F("\n------ Link Status -------"));
	for (int i = 0; i < MAX_PORTS; i++) {

		rtl8367c_setAsicPHYReg(PORT_INV(i), 31, 0); /* Set PHY Address page */
		rtl8367c_getAsicPHYReg(PORT_INV(i), 26, &val);

		Serial.print(F("Port"));
		Serial.print(i + 1, DEC);
		Serial.print(F(": "));
		if (val & (1 << 2)) {

			lr_speed = (val & 0x30) >> 4;
			if (lr_speed == 0) {
				Serial.print(F("10M "));
			}
			else if (lr_speed == 1) {
				Serial.print(F("100M "));
			}
			else if (lr_speed == 2) {
				Serial.print(F("1000M "));
			}
			if (((val & 0x8) >> 3) == 1) {
				Serial.println(F("FD"));
			} else {
				Serial.println(F("HD"));
			}
		} else {
			Serial.println(F("Link DOWN"));
		}
	}
	Serial.println(F("---- End Link Status ------"));
}

void rtl8367c_ing_mode_set(uint32_t mode)
{
	for (uint32_t i = 0; i < MAX_PORTS; i++)
		rtl8367c_setAsicVlanIngressFilter(i, mode);
}

void rtl8367c_port_tagget(uint32_t port)
{
	rtk_vlan_portAcceptFrameType_set(port, ACCEPT_FRAME_TYPE_TAG_ONLY);
}

void rtl8367c_port_untagget(uint32_t port)
{
	rtk_vlan_portAcceptFrameType_set(port, ACCEPT_FRAME_TYPE_UNTAG_ONLY);
}

void rtl8367c_add_vlan_table(uint8_t member_mask_ports, uint8_t untag_mask_ports, uint32_t vid, uint8_t prio)
{
	rtk_vlan_cfg_t VlanCfg;

	memset(&VlanCfg, 0, sizeof(rtk_vlan_cfg_t));

	VlanCfg.mbr.bits[0] = member_mask_ports;
	VlanCfg.untag.bits[0] = untag_mask_ports;
	VlanCfg.ivl_en = 1;
	VlanCfg.fid_msti = 0;
	VlanCfg.envlanpol = 0;
	VlanCfg.meteridx = 0;
	VlanCfg.vbpen = 1;
	VlanCfg.vbpri = prio;

	rtk_vlan_set((rtk_vlan_t)vid, &VlanCfg);
}

void rtl8367c_vlan_table_dump()
{
	uint32_t i;
	uint8_t mbr_mask = 0;
	int j, empty = 1;
	char fvid[4];
	rtk_vlan_cfg_t VlanCfg;

	Serial.println(F("\n---- Start table list -------"));
	for (i = 0; i < 4095; i++) {
		if(rtk_vlan_get((rtk_vlan_t)i, &VlanCfg) != 0)
			continue;

		mbr_mask = (uint8_t)VlanCfg.mbr.bits[0];

		if(mbr_mask)
		{
			if(empty)
				Serial.println(F(" VID    Ports    Priority"));
			empty = 0;
			sprintf(fvid, "%04d  ", i);
			Serial.print(fvid);
			for (j = 0; j < MAX_PORTS; j++) {
				if((mbr_mask & (1 << j)) != 0) {
					Serial.print(PORT_INV(j) + 1, DEC);
					Serial.print(F(" "));
				} else
					Serial.print(F("- "));
			}
			Serial.print(F("    "));
			Serial.print((VlanCfg.vbpri) & 0x7, DEC); /* Prio */
			Serial.println(F(""));
		}
	}
	if(empty)
		Serial.println(F("Table is empty!"));
	Serial.println(F("---- End table list ---------"));
}

void rtl8367c_init()
{
	int i, ret = -1;;
	uint32_t regValue;

	Serial.print(F("\nInit RTL8367C(S) switch..."));

	/* soft reset switch */
	rtl8367c_setAsicReg(0x1322, 1);

	/* 1sec. */
	delay(1000);

	rtl8367c_setAsicReg(0x13a0, 0x1234);
	rtl8367c_getAsicReg(0x13a0, &regValue);
	if(regValue == 0x1234) {
		for(i = 0; ret && i < 10; ++i)
		{
			ret = rtk_switch_init_8367c();
			if(ret) delay(100);
			else break;
		}
		delay(100);
		Serial.println(F("SUCCESS!"));
	} else
		Serial.println(F("ERROR!"));
}

void getString()
{
	while (1)
	{
		while (Serial.available() <= 0) {}
		char c = Serial.read();
		if(((c < 0x30) && (c != 0xd)) || (c > 0x39))
			continue;
		Serial.print(c);
		if(c == 0xd) break;
		readString += c;
		delay(3);
	}
}

int16_t input_vlan_id()
{
	int32_t res = 0;

	while(1) {
		Serial.print(F("Set VLAN ID (1~4094): "));
		getString();
		res = atoi(&readString[0]);
		readString = "";
		if((res < 1) || (res > 4094)) {
			Serial.println(F("\nNot valid VLAN ID"));
			delay(300);
			continue;
		} else
			break;
	}
	Serial.println(F(""));
	return (int16_t)res & 0xfff;
}

int8_t get_yes_no()
{
	while (1)
	{
		while (Serial.available() <= 0) {}
		char c = Serial.read();
		if((c == 'y') || (c == 'Y')) {
			Serial.print(c);
			return 1;
		}
		if((c == 'n') || (c == 'N')) {
			Serial.print(c);
			return 0;
		}
		delay(3);
	}
	return -1;
}

int8_t input_ports_mask()
{
	int8_t b = 0, mask = 0;

	for (int i = 0; i < MAX_PORTS; i++) {
		Serial.print(F("Port"));
		Serial.print(i + 1, DEC);
		Serial.print(F(": "));
		b = get_yes_no();
		mask |= b << PORT_INV(i);
		Serial.println(F(""));
		delay(3);
	}
	return mask;
}

int8_t input_prio_set()
{
	int8_t b = 0, prio = 0;
	b = get_yes_no();
	Serial.println(F(""));
	if(!b)
		return 0;
	Serial.print(F("Num priority (1~7): "));
	while (1) {
		while (Serial.available() <= 0) {}
		int c = Serial.read() - '0';
		if (c > 0 && c < 8) {
			Serial.println(c);
			return (int8_t)c;
		}
	}
	return -1;
}

void eeprom_erase()
{
#if !defined(__AVR__)
#ifndef USE_I2C_EEPROM
	EEPROM.format();
#endif
#endif
	for (int i = 0; i < CFG_SIZE /* EEPROM.length() */; i++) { /* 5 + 32(4 * 8 VID) = 37 */
		EEPROM_write(i, 0);
	}
}

void reset_factory_defaults()
{
	Serial.print(F("\nErase Configuration..."));
	eeprom_erase();
	Serial.println(F("OK"));
	rtl8367c_init();
}

int check_magic(int write)
{
	uint16_t magic = 0;

	EEPROM_get(0, magic);

	if(magic == MAGIC_EEPROM_START)
		return 1;
	if(write) {
		EEPROM_put(0, MAGIC_EEPROM_START);
		return 1;
	}
	if(magic == 0xffff) /* Force convert 0xff to 0x00 */
		eeprom_erase();
	return 0;
}

int check_idx(int write)
{
	int idx = EEPROM_read(2);
	if(write >= 0) {
		idx |= 1 << write;
		EEPROM_update(2, (unsigned char)idx);
	}
	return idx;
}

void del_idx(int num)
{
	int addr = 5, idx = EEPROM_read(2);
	eeprom_vlan_record vlan;

	idx &= ~(1 << num);
	EEPROM_update(2, (unsigned char)idx);
	vlan.vid = 0;
	vlan.ports_mask = 0;
	addr += sizeof(eeprom_vlan_record) * num;
	EEPROM_put(addr, vlan);
}

void load_configuration()
{
	eeprom_vlan_record vlan;
	int i, j, idx, addr = 5, qos_lock = 0;
	uint8_t u_ports_mask = 0, t_ports_mask = 0, pvid_mask = 0;
	rtk_portmask_t iso;

	if(!check_magic(0))
		return;

	if((idx = check_idx(-1)) == 0)
		return;

	u_ports_mask = EEPROM_read(3);
	t_ports_mask = EEPROM_read(4);

	if(!u_ports_mask && !t_ports_mask)
		return;

	rtk_vlan_init();
	/* Force clean VLAN Table */
	rtl8367c_setAsicRegBit(RTL8367C_REG_VLAN_EXT_CTRL2, RTL8367C_VLAN_EXT_CTRL2_OFFSET, 1);

	for (i = 0; i < MAX_VLAN_GROUP; i++) {
		if((idx & (1 << i)) == 0) {
			addr += sizeof(eeprom_vlan_record);
			continue;
		}

		EEPROM_get(addr, vlan);

		pvid_mask = vlan.ports_mask & ~(t_ports_mask);

		/* Set isolation */
		for (j = 0; j < MAX_PORTS; j++) {
			if((pvid_mask & (1 << j)) != 0) {
				iso.bits[0] = vlan.ports_mask;
				rtk_port_isolation_set((uint32_t)j, iso);
			}
		}

		/* Set VLAN */
		rtl8367c_add_vlan_table(vlan.ports_mask, u_ports_mask, vlan.vid, vlan.prio);

		/* Set PVID */
		for (j = 0; j < MAX_PORTS; j++) {
			if((pvid_mask & (1 << j)) != 0)
				rtk_vlan_portPvid_set((uint32_t)j, vlan.vid, (uint32_t)vlan.prio);
		}

		/* Init QoS */
		if(vlan.prio && !qos_lock) {
			rtk_qos_init(MAX_VLAN_GROUP);
			qos_lock = 1;
		}

		addr += sizeof(eeprom_vlan_record);
	}

	/* UnTagget ports */
	for (i = 0; i < MAX_PORTS; i++) {
		if((u_ports_mask & (1 << i)) != 0)
			rtl8367c_port_untagget((uint32_t)i);
	}

	/* Tagget ports */
	for (i = 0; i < MAX_PORTS; i++) {
		if((t_ports_mask & (1 << i)) != 0)
			rtl8367c_port_tagget((uint32_t)i);
	}

	rtl8367c_ing_mode_set(1);
}

void show_configuration()
{
	eeprom_vlan_record vlan;
	int i, j, idx, addr = 5;
	int8_t ports_mask = 0;

	Serial.println(F(""));
	if(!check_magic(0)) {
		Serial.println(F("Configuration is empty!"));
		return;
	}

	if((idx = check_idx(-1)) == 0) {
		Serial.println(F("VLAN groups is empty!"));
		return;
	}

	Serial.println(F("---------- Configuration ----------"));
	Serial.println(F("       [ Arduino Firmware ]"));
	Serial.print(F("Version: "));
	Serial.println(F(VERSION));
	Serial.println(F("       [ Ports Groups ]"));
	/* UnTagged ports */
	Serial.print(F("UnTagged ports: "));
	ports_mask = EEPROM_read(3);
	for (i = 0; i < MAX_PORTS; i++) {
		if((ports_mask & (1 << i)) != 0) {
			Serial.print(PORT_INV(i) + 1, DEC);
			Serial.print(F(" "));
		}
	}
	Serial.println(F(""));

	/* Tagged ports */
	Serial.print(F("Tagged ports:   "));
	ports_mask = EEPROM_read(4);
	for (i = 0; i < MAX_PORTS; i++) {
		if((ports_mask & (1 << i)) != 0) {
			Serial.print(PORT_INV(i) + 1, DEC);
			Serial.print(F(" "));
		}
	}
	Serial.println(F(""));

	/* VLAN Groups */
	Serial.println(F("       [ VLAN Groups ]"));
	for (i = 0; i < MAX_VLAN_GROUP; i++) {
		if((idx & (1 << i)) == 0) {
			addr += sizeof(eeprom_vlan_record);
			continue;
		}
		EEPROM_get(addr, vlan);
		Serial.print(F("IDX"));
		Serial.print(i + 1, DEC);
		Serial.print(F(": Ports: "));
		for (j = 0; j < MAX_PORTS; j++) {
			if((vlan.ports_mask & (1 << j)) != 0) {
				Serial.print(PORT_INV(j) + 1, DEC);
				Serial.print(F(" "));
			} else
				Serial.print(F("- "));
		}
		Serial.print(F("VID: "));
		char fvid[4];
		sprintf(fvid, "%04d", vlan.vid);
		Serial.print(fvid);
		Serial.print(F(" Priority: "));
		Serial.println(vlan.prio, DEC);

		addr += sizeof(eeprom_vlan_record);
	}
	Serial.println(F("-------- End Configuration --------"));
}

void setup()
{
	Serial.begin(115200);
#if !defined(__AVR__) /* flash 128Kbyte, if 64Kbyte use 0x800Fxxx */
#ifndef USE_I2C_EEPROM
	EEPROM.PageBase0 = 0x801F000;
	EEPROM.PageBase1 = 0x801F800;
	EEPROM.PageSize  = 0x400; /* set EEPROM Emulation size 1024 byte, if need 2048 = 0x800 */
	EEPROM.init();
	delay(1000);
#else
	HWire.begin();
	delay(50);
#endif
#else
	delay(50);
#endif
	Serial.println(F("\n"));
	Serial.setTimeout(60L*60L*1000L);
	pinMode(PIN_SDA, INPUT);
	pinMode(PIN_SCK, INPUT);
	Serial.println(F("\nWaiting for the switch chip to start-up (6 sec)..."));
	rtl8367c_init();
	load_configuration();
	delay(1000);
}

void loop()
{
	Serial.println(F("\nRTL8367C(S) switch setup"));
	Serial.println(F("0. Link Status"));
	Serial.println(F("1. Show VLAN Table from switch"));
	Serial.println(F("2. Show system configuration"));
	Serial.println(F("3. Add Tagged ports group"));
	Serial.println(F("4. Add UnTagged ports group"));
	Serial.println(F("5. Add VLAN ID, ports group and priority"));
	Serial.println(F("6. Delete VLAN ID, ports group and priority by IDX"));
	Serial.println(F("7. Apply new configuration"));
	Serial.println(F("8. Reset in factory defaults"));
	Serial.print(F("Enter you choice: "));
	int c, ch = -1, i, idx, addr;
	int8_t mask;
	while (1)
	{
		while (Serial.available() <= 0) {}
		c = Serial.read();
#if !defined(__AVR__)
		if(c == 0xd) { /* for STM32 board fix */
			Serial.println(F(""));
			break;
		}
#endif
		ch = c - '0';
		if (ch >= 0 && ch < 9) break;
		delay(3);
	}
	if (ch >= 0 && ch < 9)
	{
		Serial.println(ch);
		switch(ch) {
			case 0:
				rtl8367c_link_status();
				break;
			case 1:
				rtl8367c_vlan_table_dump();
				break;
			case 2:
				show_configuration();
				break;
			case 3:
				{
					Serial.println(F("\nTagged ports group set (y/n):"));
					mask = input_ports_mask();
					EEPROM_update(4, (unsigned char)mask);
					check_magic(1);
					Serial.println(F("Save configuration OK!"));
					break;
				}
			case 4:
				{
					Serial.println(F("\nUnTagged ports group set (y/n):"));
					mask = input_ports_mask();
					EEPROM_update(3, (unsigned char)mask);
					check_magic(1);
					Serial.println(F("Save configuration OK!"));
					break;
				}
			case 5:
				{
					int free_idx = 0;
					addr = 5;
					if((idx = check_idx(-1)) == 0xff) {
						Serial.println(F("\nTable VLAN ID and ports group is FULL!"));
						break;
					}
					if(!EEPROM_read(3) && !EEPROM_read(4)) {
						Serial.println(F("\nPlease first set masks Tagged or UnTagged ports group!"));
						break;
					}
					eeprom_vlan_record vlan;
					for (i = 0; i < MAX_VLAN_GROUP; i++) { /* Get free idx */
						if((idx & (1 << i)) == 0) {
							free_idx = i;
							addr += sizeof(eeprom_vlan_record) * i;
							break;
						}
					}
					vlan.vid = input_vlan_id();
					Serial.print(F("VLAN ID set: "));
					Serial.println(vlan.vid, DEC);
					Serial.println(F("Ports group set (y/n):"));
					vlan.ports_mask = input_ports_mask();
					Serial.print(F("Priority set (y/n): "));
					vlan.prio = input_prio_set();
					Serial.println(F(""));
					check_magic(1);
					check_idx(free_idx);
					EEPROM_put(addr, vlan);
					Serial.println(F("Save configuration OK!"));
					break;
				}
			case 6:
				{
					Serial.print(F("\nEnter number IDX for delete (1~8): "));
					while (1)
					{
						while (Serial.available() <= 0) {}
						idx = Serial.read()-'0';
						if (idx > 0 && idx < 9)
							break;
						delay(3);
					}
					Serial.println(idx);
					del_idx(idx - 1);
					Serial.print(F("Delete IDX"));
					Serial.print(idx, DEC);
					Serial.println(F(" and save configuration OK!"));
					break;
				}
			case 7:
				rtl8367c_init();
				load_configuration();
				break;
			case 8:
				reset_factory_defaults();
				break;
		}
		delay(1000);
	}
}
