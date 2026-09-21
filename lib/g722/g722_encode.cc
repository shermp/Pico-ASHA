/*
 * SpanDSP - a series of DSP components for telephony
 *
 * g722_encode.c - The ITU G.722 codec, encode part.
 *
 * Written by Steve Underwood <steveu@coppice.org>
 *
 * Copyright (C) 2005 Steve Underwood
 *
 * All rights reserved.
 *
 *  Despite my general liking of the GPL, I place my own contributions 
 *  to this code in the public domain for the benefit of all mankind -
 *  even the slimy ones who might try to proprietize my work and use it
 *  to my detriment.
 *
 * Based on a single channel 64kbps only G.722 codec which is:
 *
 *****    Copyright (c) CMU    1993      *****
 * Computer Science, Speech Group
 * Chengxiang Lu and Alex Hauptmann
 *
 * $Id: g722_encode.c,v 1.14 2006/07/07 16:37:49 steveu Exp $
 */

/*! \file */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "g722_typedefs.h"
#include "g722_enc_dec.h"

#if !defined(FALSE)
#define FALSE 0
#endif
#if !defined(TRUE)
#define TRUE (!FALSE)
#endif

#define PACKED_OUTPUT   (0)
#define BITS_PER_SAMPLE (8)

#ifndef BUILD_FEATURE_G722_USE_INTRINSIC_SAT
static __inline int16_t saturate(int32_t amp)
{
    int16_t amp16;

    /* Hopefully this is optimised for the common case - not clipping */
    amp16 = (int16_t) amp;
    if (amp == amp16)
        return amp16;
    if (amp > 0x7FFF)
        return  0x7FFF;
    return  0x8000;
}
#else
static __inline int16_t saturate(int32_t val)
{
    register int32_t res;
    __asm volatile (
        "SSAT %0, #16, %1\n\t"
        :"=r"(res)
        :"r"(val)
        :);
    return (int16_t)res;
}
#endif
/*- End of function --------------------------------------------------------*/

static void block4(g722_band_t *band, int d)
{
    int wd1;
    int wd2;
    int wd3;
    int i;
    int ap1, ap2;
    int sg0, sg1, sg2, sgi;
    int sz;

    /* Block 4, RECONS */
    band->d[0] = d;
    band->r[0] = saturate(band->s + d);

    /* Block 4, PARREC */
    band->p[0] = saturate(band->sz + d);

    /* Block 4, UPPOL2 */
    sg0 = band->p[0] >> 15;
    sg1 = band->p[1] >> 15;
    sg2 = band->p[2] >> 15;
    wd1 = saturate(band->a[1] << 2);

    wd2 = (sg0 == sg1)  ?  -wd1  :  wd1;
    if (wd2 > 32767)
        wd2 = 32767;

    ap2 = (wd2 >> 7) + ((sg0 == sg2)  ?  128  :  -128);
    ap2 += (band->a[2]*32512) >> 15;
    if (ap2 > 12288)
        ap2 = 12288;
    else if (ap2 < -12288)
        ap2 = -12288;
    /* Block 4, UPPOL1 */
    wd1 = (sg0 == sg1)  ?  192  :  -192;
    wd2 = (band->a[1]*32640) >> 15;

    ap1 = saturate(wd1 + wd2);
    wd3 = saturate(15360 - ap2);
    if (ap1 > wd3)
        ap1 = wd3;
    else if (ap1 < -wd3)
        ap1 = -wd3;
    /* Block 4, UPZERO */
    /* Block 4, FILTEZ */
    wd1 = (d == 0)  ?  0  :  128;

    sg0 = d >> 15;

    /* Block 4, DELAYA. Run this backwards so UPZERO and FILTEZ can share
       one pass without overwriting any delayed samples before they are read. */
    sz = 0;
    for (i = 6;  i > 0;  i--)
    {
        int bi;

        sgi = band->d[i] >> 15;
        wd2 = (sgi == sg0) ? wd1 : -wd1;
        wd3 = (band->b[i]*32640) >> 15;
        bi = saturate(wd2 + wd3);
        band->b[i] = bi;
        band->d[i] = band->d[i - 1];
        wd3 = saturate(band->d[i] + band->d[i]);
        sz += (bi*wd3) >> 15;
    }
    band->sz = sz;

    /* ap[] and bp[] are reference-algorithm scratch. Their values have
       already been consumed, so only persist state read by the next sample. */
    band->r[2] = band->r[1];
    band->r[1] = band->r[0];
    band->p[2] = band->p[1];
    band->p[1] = band->p[0];
    band->a[2] = ap2;
    band->a[1] = ap1;

    /* Block 4, FILTEP */
    wd1 = saturate(band->r[1] + band->r[1]);
    wd1 = (band->a[1]*wd1) >> 15;
    wd2 = saturate(band->r[2] + band->r[2]);
    wd2 = (band->a[2]*wd2) >> 15;
    band->sp = saturate(wd1 + wd2);

    /* Block 4, PREDIC */
    band->s = saturate(band->sp + band->sz);
}
/*- End of function --------------------------------------------------------*/

g722_encode_state_t *g722_encode_init(g722_encode_state_t *s,
                                             unsigned int rate, [[maybe_unused]] int options)
{
    if (s == NULL)
    {
#ifdef G722_SUPPORT_MALLOC
        if ((s = (g722_encode_state_t *) malloc(sizeof(*s))) == NULL)
#endif
            return NULL;
    }
    memset(s, 0, sizeof(*s));
    if (rate == 48000)
        s->bits_per_sample = 6;
    else if (rate == 56000)
        s->bits_per_sample = 7;
    else
        s->bits_per_sample = 8;
    s->band[0].det = 32;
    s->band[1].det = 8;
    return s;
}
/*- End of function --------------------------------------------------------*/

int g722_encode_release(g722_encode_state_t *s)
{
    free(s);
    return 0;
}
/*- End of function --------------------------------------------------------*/

/* WebRtc, tlegrand:
 * Only define the following if bit-exactness with reference implementation
 * is needed. Will only have any effect if input signal is saturated.
 */
//#define RUN_LIKE_REFERENCE_G722
#ifdef RUN_LIKE_REFERENCE_G722
int16_t limitValues (int16_t rl)
{

    int16_t yl;

    yl = (rl > 16383) ? 16383 : ((rl < -16384) ? -16384 : rl);

    return (yl);
}
/*- End of function --------------------------------------------------------*/
#endif

static int16_t q6[32] =
{
       0,   35,   72,  110,  150,  190,  233,  276,
     323,  370,  422,  473,  530,  587,  650,  714,
     786,  858,  940, 1023, 1121, 1219, 1339, 1458,
    1612, 1765, 1980, 2195, 2557, 2919,    0,    0
};
static int16_t wl[8] =
{
    -60, -30, 58, 172, 334, 538, 1198, 3042
};
static int16_t rl42[16] =
{
    0, 7, 6, 5, 4, 3, 2, 1, 7, 6, 5, 4, 3, 2, 1, 0
};
static int16_t ilb[32] =
{
    2048, 2093, 2139, 2186, 2233, 2282, 2332,
    2383, 2435, 2489, 2543, 2599, 2656, 2714,
    2774, 2834, 2896, 2960, 3025, 3091, 3158,
    3228, 3298, 3371, 3444, 3520, 3597, 3676,
    3756, 3838, 3922, 4008
};
static int16_t qm4[16] =
{
         0, -20456, -12896, -8968,
     -6288,  -4240,  -2584, -1200,
     20456,  12896,   8968,  6288,
      4240,   2584,   1200,     0
};
static int16_t qm2[4] =
{
    -7408,  -1616,   7408,   1616
};
/* The repeated prefix lets the QMF scan the circular sample history in
   physical order, without a wrap branch in its inner loop. */
static const int16_t qmf_coeffs[23] =
{
       3,  -11,   12,   32, -210,  951, 3876, -805,  362, -156,   53,  -11,
       3,  -11,   12,   32, -210,  951, 3876, -805,  362, -156,   53,
};
static int16_t wh[3] = {0, -214, 798};

int g722_encode(g722_encode_state_t *s, uint8_t g722_data[],
                       const int16_t amp[], int len)
{
    int dlow;
    int dhigh;
    int el;
    int wd;
    int wd1;
    int ril;
    int wd2;
    int il4;
    int ih2;
    int wd3;
    int eh;
    int mih;
    int i;
    int j;
    int qmf_pos;
    /* Low and high band PCM from the QMF */
    int xlow;
    int xhigh;
    int g722_bytes;
    /* Even and odd tap accumulators */
    int sumeven;
    int sumodd;
    int ihigh;
    int ilow;
    int code;

    g722_bytes = 0;
    xhigh = 0;
    /* Encoder bit packing uses out_bits; retain the QMF head in the otherwise
       unused input-bit field without changing the public state layout. */
    qmf_pos = s->in_bits;
    for (j = 0;  j < len;  )
    {
        if (s->itu_test_mode)
        {
            xlow =
            xhigh = amp[j++] >> 1;
        }
        else
        {
            {
                /* Apply the transmit QMF. Keep the history circular: moving
                   its start is much cheaper on Cortex-M0+ than copying 22
                   words for every pair of input samples. */
                //TODO: if len is odd, then this can be a buffer overrun
                s->x[qmf_pos] = amp[j++];
                s->x[qmf_pos + 1] = amp[j++];
                qmf_pos += 2;
                if (qmf_pos == 24)
                    qmf_pos = 0;
    
                /* Discard every other QMF output */
                sumeven = 0;
                sumodd = 0;
                int qmf_pair = qmf_pos >> 1;
                int odd_coeff = 12 - qmf_pair;
                if (odd_coeff == 12)
                    odd_coeff = 0;
                const int16_t *odd = &qmf_coeffs[odd_coeff];
                const int16_t *even = &qmf_coeffs[qmf_pair + 11];
                for (i = 0;  i < 24;  i += 2)
                {
                    sumodd += s->x[i]*(*odd++);
                    sumeven += s->x[i + 1]*(*even--);
                }
                /* We shift by 12 to allow for the QMF filters (DC gain = 4096), plus 1
                   to allow for us summing two filters, plus 1 to allow for the 15 bit
                   input to the G.722 algorithm. */
                xlow = (sumeven + sumodd) >> 14;
                xhigh = (sumeven - sumodd) >> 14;

#ifdef RUN_LIKE_REFERENCE_G722
                /* The following lines are only used to verify bit-exactness
                 * with reference implementation of G.722. Higher precision
                 * is achieved without limiting the values.
                 */
                xlow = limitValues(xlow);
                xhigh = limitValues(xhigh);
#endif
            }
        }
        /* Block 1L, SUBTRA */
        el = saturate(xlow - s->band[0].s);

        /* Block 1L, QUANTL */
        wd = (el >= 0)  ?  el  :  -(el + 1);

        int lower = 1;
        int upper = 30;
        while (lower < upper)
        {
            i = (lower + upper) >> 1;
            wd1 = (q6[i]*s->band[0].det) >> 12;
            if (wd < wd1)
                upper = i;
            else
                lower = i + 1;
        }
        i = lower;
        if (el < 0)
            ilow = ((i < 3) ? 64 : 34) - i;
        else
            ilow = 62 - i;

        /* Block 2L, INVQAL */
        ril = ilow >> 2;
        wd2 = qm4[ril];
        dlow = (s->band[0].det*wd2) >> 15;

        /* Block 3L, LOGSCL */
        il4 = rl42[ril];
        wd = (s->band[0].nb*127) >> 7;
        s->band[0].nb = wd + wl[il4];
        if (s->band[0].nb < 0)
            s->band[0].nb = 0;
        else if (s->band[0].nb > 18432)
            s->band[0].nb = 18432;

        /* Block 3L, SCALEL */
        wd1 = (s->band[0].nb >> 6) & 31;
        wd2 = 8 - (s->band[0].nb >> 11);
        wd3 = (wd2 < 0)  ?  (ilb[wd1] << -wd2)  :  (ilb[wd1] >> wd2);
        s->band[0].det = wd3 << 2;

        block4(&s->band[0], dlow);
        {
	    int nb;

            /* Block 1H, SUBTRA */
            eh = saturate(xhigh - s->band[1].s);

            /* Block 1H, QUANTH */
            wd = (eh >= 0)  ?  eh  :  -(eh + 1);
            wd1 = (564*s->band[1].det) >> 12;
            mih = (wd >= wd1)  ?  2  :  1;
            ihigh = ((eh < 0) ? 2 : 4) - mih;

            /* Block 2H, INVQAH */
            wd2 = qm2[ihigh];
            dhigh = (s->band[1].det*wd2) >> 15;

            /* Block 3H, LOGSCH */
            ih2 = 2 - (ihigh & 1);
            wd = (s->band[1].nb*127) >> 7;

            nb = wd + wh[ih2];
            if (nb < 0)
                nb = 0;
            else if (nb > 22528)
                nb = 22528;
	    s->band[1].nb = nb;

            /* Block 3H, SCALEH */
            wd1 = (s->band[1].nb >> 6) & 31;
            wd2 = 10 - (s->band[1].nb >> 11);
            wd3 = (wd2 < 0)  ?  (ilb[wd1] << -wd2)  :  (ilb[wd1] >> wd2);
            s->band[1].det = wd3 << 2;

            block4(&s->band[1], dhigh);
#if   BITS_PER_SAMPLE == 8
            code = ((ihigh << 6) | ilow);
#elif BITS_PER_SAMPLE == 7
            code = ((ihigh << 6) | ilow) >> 1;
#elif BITS_PER_SAMPLE == 6
            code = ((ihigh << 6) | ilow) >> 2;
#endif
        }

#if PACKED_OUTPUT == 1
            /* Pack the code bits */
            s->out_buffer |= (code << s->out_bits);
            s->out_bits += s->bits_per_sample;
            if (s->out_bits >= 8)
            {
                g722_data[g722_bytes++] = (uint8_t) (s->out_buffer & 0xFF);
                s->out_bits -= 8;
                s->out_buffer >>= 8;
            }
#else
            g722_data[g722_bytes++] = (uint8_t) code;
#endif
    }
    s->in_bits = qmf_pos;
    return g722_bytes;
}
/*- End of function --------------------------------------------------------*/
/*- End of file ------------------------------------------------------------*/
