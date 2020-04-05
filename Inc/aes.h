/*
 * aes.h
 *
 *  Created on: 22.10.2018
 *      Author: Szymon
 */

#ifndef AES_H_
#define AES_H_

#include <stdint.h>     /* C99 types */
#include <string.h>     /* memset */

#define MSG(args...)    fprintf(stderr, args) /* message that is destined to the user */

#define AES_MICSUB 0x30 // internal use only

#define AES_ENC       0x00
#define AES_DEC       0x01
#define AES_MIC       0x02
#define AES_CTR       0x04
#define AES_MICNOAUX  0x08

#define msbf4_read(p)    ((p)[0]<<24 | (p)[1]<<16 | (p)[2]<<8 | (p)[3])
#define msbf4_write(p,v) (p)[0]=(v)>>24,(p)[1]=(v)>>16,(p)[2]=(v)>>8,(p)[3]=(v)
#define swapmsbf(x)      ( (x&0xFF)<<24 | (x&0xFF00)<<8 | (x&0xFF0000)>>8 | (x>>24) )

#define u1(v)                       ((uint8_t)(v))

#define AES_key4(r1,r2,r3,r0,i)    r1 = ki[i+1]; \
                                   r2 = ki[i+2]; \
                                   r3 = ki[i+3]; \
                                   r0 = ki[i]

#define AES_expr4(r1,r2,r3,r0,i)   r1 ^= AES_E4[u1(i)];     \
                                   r2 ^= AES_E3[u1(i>>8)];  \
                                   r3 ^= AES_E2[u1(i>>16)]; \
                                   r0 ^= AES_E1[  (i>>24)]

#define AES_expr(a,r0,r1,r2,r3,i)  a = ki[i];                    \
                                   a ^= (AES_S[   r0>>24 ]<<24); \
                                   a ^= (AES_S[u1(r1>>16)]<<16); \
                                   a ^= (AES_S[u1(r2>> 8)]<< 8); \
                                   a ^=  AES_S[u1(r3)    ]

int aes_verifyMic (const uint8_t * key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t * pdu, int len);


void aes_appendMic (const uint8_t * key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t * pdu, int len);


void aes_cipher (const uint8_t * key, uint32_t devaddr, uint32_t seqno, int dndir, uint8_t * payload, int len);



#endif /* AES_H_ */
