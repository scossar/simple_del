#include "simple_del_shared.h"
#include <m_pd.h>

typedef struct _delay {
  t_object x_obj;

  t_float x_s_per_msec; // samples per msec
  t_float x_delay_buffer_msecs;
  int x_delay_buffer_samples; // number of samples in delay buffer
  t_sample *x_delay_buffer;
  int x_pd_block_size;
  int x_phase; // current __write__ position

  t_inlet *x_delay_msecs_inlet;
} t_delay;

t_class *delay_class = NULL;

static void delay_buffer_update(t_delay *x);
static void delay_set_delay_samples(t_delay *x, t_float f);

static void *delay_new(t_floatarg buffer_msecs)
{
  t_delay *x = (t_delay *)pd_new(delay_class);

  x->x_delay_buffer_msecs = buffer_msecs;
  x->x_s_per_msec = 0.0f;
  x->x_delay_buffer_samples = 0;
  x->x_pd_block_size = 0;
  x->x_phase = 0;
  
  x->x_delay_buffer = getbytes(XTRASAMPS * sizeof(t_sample));

  delay_buffer_update(x);

  x->x_delay_msecs_inlet = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
  // pd_float((t_pd *)x->x_delay_msecs_inlet, x->x_delay_msecs); // sets inlet initial value from

  outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}

static void delay_buffer_update(t_delay *x)
{
  int nsamps = x->x_delay_buffer_msecs * x->x_s_per_msec;
  if (nsamps < 1) nsamps = 1;

  // round up to a multiple of SAMPBLK
  // see: `twos_complement_bit_masking.md`
  nsamps += (- nsamps) & (SAMPBLK - 1);

  // add a block of samples (to ensure buffer is big enough?)
  nsamps += x->x_pd_block_size;

  if (x->x_delay_buffer_samples < nsamps) {
    x->x_delay_buffer = (t_sample *)resizebytes(x->x_delay_buffer,
                                                x->x_delay_buffer_samples + XTRASAMPS * sizeof(t_sample),
                                                (nsamps + XTRASAMPS) * sizeof(t_sample));
    x->x_delay_buffer_samples = nsamps;
    x->x_phase = XTRASAMPS;
  }
}

static void delay_set_system_params(t_delay *x, int blocksize, t_float sr)
{
  x->x_pd_block_size = blocksize;
  x->x_s_per_msec = sr * 0.001f;
}

static t_int *delay_perform(t_int *w)
{
  t_delay *x = (t_delay*)(w[1]);
  t_sample *in1 = (t_sample *)(w[2]);
  t_sample *in2 = (t_sample *)(w[3]);
  t_sample *out = (t_sample *)(w[4]);
  int n = (int)(w[5]);

  int write_phase = x->x_phase;
  write_phase += n;

  int delay_buffer_samps = x->x_delay_buffer_samples;

  t_sample *vp = x->x_delay_buffer;
  t_sample *wp = vp + write_phase;
  // t_sample *rp;
  t_sample *ep = vp + (x->x_delay_buffer_samples + XTRASAMPS);

  t_sample limit = delay_buffer_samps - 4;
  // t_sample fn = n - 1;


  if (limit < 1) { // buffer to small for interpolation
    while (n--) {
      t_sample f = *in1++;
      if (PD_BIGORSMALL(f)) f = 0.0f;

      *wp++ = f;
      if (wp == ep) {
        vp[0] = ep[-4];
        vp[1] = ep[-3];
        vp[2] = ep[-2];
        vp[3] = ep[-1];
        write_phase -= delay_buffer_samps;
      }
      *out++ = 0;
    }
    return (w+6);
  }

  while (n--) {
    // write to delay buffer
    t_sample f = *in1++;
    if (PD_BIGORSMALL(f)) f = 0.0f;
    *wp++ = f;
    if (wp == ep) {
      vp[0] = ep[-4];
      vp[1] = ep[-3];
      vp[2] = ep[-2];
      vp[3] = ep[-1];
      wp = vp + XTRASAMPS;
      write_phase -= delay_buffer_samps;
    }

    // calculate delay in samples from delay time signal
    t_sample delay_time = *in2++;
    if (delay_time < 1.0f) delay_time = 1.0f;
    t_sample delsamps = x->x_s_per_msec * delay_time;

    // constrain delay to valid range
    if (delsamps < 4.0f) delsamps = 4.0f; // min for cubic interpolation
    if (delsamps > limit) delsamps = limit;

    // calculate read position and fractional part
    int idelsamps = (int)delsamps;
    t_sample frac = delsamps - (t_sample)idelsamps;

    // calculate read pointer position with wrapping
    int read_phase = write_phase - idelsamps;
    if (read_phase < XTRASAMPS) read_phase += delay_buffer_samps;

    // get 4 samples for cubic interpolation
    t_sample a, b, c, d;

    t_sample *rp = vp + read_phase;

    // get sample d (3 samples before current read position)
    if (read_phase >= XTRASAMPS + 3) {
      d = rp[-3];
    } else { // handle wrapping for sample d
      int idx = read_phase - 3;
      if (idx < XTRASAMPS) idx += delay_buffer_samps;
      d = vp[idx];
    }

    // get sample c
    if (read_phase >= XTRASAMPS + 2) {
      c = rp[-2];
    } else {
      int idx = read_phase - 2;
      if (idx < XTRASAMPS) idx += delay_buffer_samps;
      c = vp[idx];
    }

    // sample b
    if (read_phase >= XTRASAMPS + 1) {
      b = rp[-1];
    } else {
      int idx = read_phase - 1;
      if (idx < XTRASAMPS) idx += delay_buffer_samps;
      b = vp[idx];
    }

    // sample a
    a = *rp;

    // cubic interpolation
    t_sample cminusb = c - b;
    *out++ = b + frac * (
        cminusb - 0.1666667f * (1.0f - frac) * (
            (d - a - 3.0f * cminusb) * frac + (d + 2.0f * a - 3.0f * b)
        )
    );
  }

  x->x_phase = write_phase;
  return (w+6);
}

// static t_int *delay_perform(t_int *w)
// {
//   t_delay *x = (t_delay*)(w[1]);
//   t_sample *in1 = (t_sample *)(w[2]);
//   t_sample *in2 = (t_sample *)(w[3]);
//   t_sample *out = (t_sample *)(w[4]);
//   int n = (int)(w[5]);
//
//   int write_phase = x->x_phase;
//   write_phase += n;
//
//   int delay_buffer_samps = x->x_delay_buffer_samples;
//
//   t_sample limit = delay_buffer_samps - n;
//   t_sample fn = n - 1;
//
//   t_sample *vp = x->x_delay_buffer;
//   t_sample *wp = vp + write_phase;
//   t_sample *rp;
//   t_sample *ep = vp + (x->x_delay_buffer_samples + XTRASAMPS);
//
//   if (limit < 0) { // blocksize is larger than delay buffer size
//     while (n--) {
//       t_sample f = *in1++;
//       if (PD_BIGORSMALL(f)) f = 0.0f;
//
//       *wp++ = f;
//
//       if (wp == ep) {
//         vp[0] = ep[-4];
//         vp[1] = ep[-3];
//         vp[2] = ep[-2];
//         vp[3] = ep[-1];
//         write_phase -= delay_buffer_samps;
//       }
//
//       *out++ = 0;
//     }
//     return (w+6);
//   }
//
//   while (n--) {
//     // write to delay buffer
//     t_sample f = *in1++;
//     if (PD_BIGORSMALL(f)) f = 0.0f;
//     *wp++ = f;
//     if (wp == ep) {
//       vp[0] = ep[-4];
//       vp[1] = ep[-3];
//       vp[2] = ep[-2];
//       vp[3] = ep[-1];
//       wp = vp + XTRASAMPS;
//       write_phase -= delay_buffer_samps;
//     }
//
//     // read from buffer
//     t_sample delsamps = x->x_s_per_msec * *in2++;
//     t_sample frac;
//     int idelsamps;
//     t_sample a, b, c, d, cminusb;
//
//     if (!(delsamps >= 1.00001f)) delsamps = 1.00001f; // too small or NaN
//     if (delsamps > limit) delsamps = limit; // too big
//
//     delsamps += fn;
//     fn = fn - 1.0f;
//
//     idelsamps = delsamps; // moving the pointer?
//     frac = delsamps - (t_sample)idelsamps;
//     rp = wp - idelsamps;
//     if (rp < vp + XTRASAMPS) rp += delay_buffer_samps;
//     d = rp[-3];
//     c = rp[-2];
//     b = rp[-1];
//     a = rp[0];
//     cminusb = c - b;
//
//     *out++ = b + frac * (
//         cminusb - 0.1666667f * (1.-frac) * (
//             (d - a - 3.0f * cminusb) * frac + (d + 2.0f*a - 3.0f*b)
//         )
//     );
//
//     if (rp == ep) rp -= delay_buffer_samps;
//   }
//
//   x->x_phase = write_phase;
//   return (w+6);
// }

static void delay_dsp(t_delay *x, t_signal **sp)
{
  dsp_add(delay_perform, 5, x, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[0]->s_length);
  delay_set_system_params(x, sp[0]->s_length, sp[0]->s_sr);
  delay_buffer_update(x);
}

static void delay_free(t_delay *x)
{
  if (x->x_delay_buffer != NULL) {
    freebytes(x->x_delay_buffer,
              (x->x_delay_buffer_samples + XTRASAMPS) * sizeof(t_sample));
    x->x_delay_buffer = NULL;
  }
}

void delay_tilde_setup(void)
{
  delay_class = class_new(gensym("delay~"),
                          (t_newmethod)delay_new,
                          (t_method)delay_free,
                          sizeof(t_delay),
                          CLASS_DEFAULT,
                          A_DEFFLOAT, 0);

  class_addmethod(delay_class, (t_method)delay_dsp,
                  gensym("dsp"), A_CANT, 0);

  CLASS_MAINSIGNALIN(delay_class, t_delay, x_delay_buffer_msecs);
}
