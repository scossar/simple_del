/* A copy of objects from pure_data/src/d_delay.c
 * See `c_syntax_if_else_statements` for details about some of the C specific
 * stuff (for example, why there's a header file)
 *
 * See `pure_data_audio_delay_objects.md` for a general overview of what's going
 * on. TODO: include that markdown file in the source code.
 * */

/*
 * Implements an "include guard" pattern.
 * The first time the header is included, `SIMPLE_DEL_SHARED_H` is not defined,
 * so the preprocessor includes everything up to the #endif.
 * `SIMPLE_DEL_SHARED_H` is defined for future reference. The next time the
 * header is included, the preprocessor will skip everything between the #ifndef
 * and #endif
 */
#ifndef SIMPLE_DEL_SHARED_H
#define SIMPLE_DEL_SHARED_H

#include "m_pd.h"

#define XTRASAMPS 4
#define SAMPBLK 4

// core structure that manages the delay buffer. used by both delwrite and
// delread
typedef struct simple_delwritectl
{
  int c_n; // size of the delay buffer in samples
  t_sample *c_vec; // pointer to delay buffer: allocated with getbytes(XTRASAMPS * sizeof(t_sample))
  int c_phase; // current write position in the buffer
} t_simple_delwritectl;

typedef struct _simple_delwrite
{
  t_object x_obj;
  t_symbol *x_sym;
  t_float x_deltime; /* delay in msec */
  t_simple_delwritectl x_cspace;
  int x_sortno; /* DSP sort number at which this was last put on chain */
  int x_rsortno; /* DSP sort number for first delread or write in the chain */
  int x_vecsize; /* vector size for delread~ to use */
  t_float x_sr; /* system samplerate? */
  t_float x_f;
} t_simple_delwrite;

t_simple_delwrite *simple_delwrite_findbyname(t_symbol *s);
void simple_delwrite_update(t_simple_delwrite *x);
void simple_delwrite_check(t_simple_delwrite *x, int vecsize, t_float sr);

static inline t_sample cubic_interpolate(t_sample *buffer, int phase, int mask, t_sample frac)
{
  t_sample a = buffer[phase];
  t_sample b = buffer[(phase - 1) & mask];
  t_sample c = buffer[(phase - 2) & mask];
  t_sample d = buffer[(phase - 3) & mask];
  t_sample cminusb = c - b;

  return b + frac * (
      cminusb - 0.1666667f * (1.0f - frac) * (
          (d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b)
      )
  );
}
#endif
