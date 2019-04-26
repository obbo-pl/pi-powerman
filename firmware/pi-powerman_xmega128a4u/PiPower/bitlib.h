/*
 * bitlib.h Common bitwise operations and port operation library
 *
 * Created: 2017-05-02
 * Atmel Studio 6 (Version: 6.2.1563 - Service Pack 2)
 *  Author: Krzysztof Markiewicz
 *  www.obbo.pl
 *
 * This program is distributed under the terms of the GNU General Public License
 */


#ifndef BITLIB_H_
#define BITLIB_H_


#define	hibyte(x) (uint8_t)(x >> 8)
#define	lobyte(x) (uint8_t)(x & 0x00ff)
#define setbit(value, bit) ((value) |= (uint8_t)(1 << bit))
#define clrbit(value, bit) ((value) &= (uint8_t)~(1 << bit))
#define testbit(value, bit) (uint8_t)(((uint8_t)value >> bit) & 0x01)
#define togglebit(value, bit) ((value) ^= (uint8_t)(1 << bit))
#define bitmask(bit) ((uint8_t)(1 << bit))


#endif /* BITLIB_H_ */