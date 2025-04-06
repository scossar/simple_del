#include "simple_del_shared.h"
#include <m_pd.h>

typedef struct _delay1 {
  t_object x_obj;

  t_float x_s_per_msec; // samples per msec
  t_float x_delay_buffer_msecs;
  int x_delay_buffer_samples; // number of samples in delay buffer
  int x_delay_msecs; // number of msecs to delay
  int x_delay_samples; // number of samples of delay
  t_sample *x_delay_buffer;
  int x_pd_block_size;
  int x_phase; // current __write__ position

} t_delay1;

t_class *delay1_class = NULL;

static void delay_buffer_update(t_delay1 *x);
static void delay_set_delay_samples(t_delay1 *x, t_float f);

static void *delay_new(t_floatarg buffer_msecs, t_floatarg delay_msecs)
{
  t_delay1 *x = (t_delay1 *)pd_new(delay1_class);

  x->x_delay_buffer_msecs = buffer_msecs;
  x->x_delay_msecs = delay_msecs;

  x->x_s_per_msec = 0.0f;
  x->x_delay_buffer_samples = 0;
  x->x_pd_block_size = 0;
  x->x_phase = 0;
  x->x_delay_samples = 0;
  
  x->x_delay_buffer = getbytes(XTRASAMPS * sizeof(t_sample));

  delay_buffer_update(x);
  delay_set_delay_samples(x, x->x_delay_msecs);

  outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}

static void delay_buffer_update(t_delay1 *x)
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

static void delay_set_system_params(t_delay1 *x, int blocksize, t_float sr)
{
  if (blocksize > x->x_pd_block_size) {
    x->x_pd_block_size = blocksize;
  }

  // awkward, but maybe clearer than the original source code?
  if ((sr * 0.001f) > x->x_s_per_msec) {
    x->x_s_per_msec = sr * 0.001f;
  }
}

static void delay_set_delay_samples(t_delay1 *x, t_float f)
{
  // currently this is just reassigning x->x_delay_msecs to x->x_delay_msecs
  // this approach might make sense later when x->delay_msecs can be set via a
  // message instead of just as a creating argument.
  x->x_delay_msecs = f;
  x->x_delay_samples = (int)(0.5 + x->x_s_per_msec * x->x_delay_msecs) + x->x_pd_block_size;
}

static t_int *delay1_perform(t_int *w)
{
  t_delay1 *x = (t_delay1*)(w[1]);
  t_sample *in1 = (t_sample *)(w[2]);
  t_sample *out = (t_sample *)(w[3]);
  int n = (int)(w[4]);

  int write_phase = x->x_phase;
  write_phase += n;
  int read_phase = write_phase - x->x_delay_samples;

  int delay_buffer_samps = x->x_delay_buffer_samples;

  t_sample *vp = x->x_delay_buffer;
  t_sample *wp = vp + write_phase;
  t_sample *rp;
  t_sample *ep = vp + (x->x_delay_buffer_samples + XTRASAMPS);

  if (read_phase < 0) read_phase += delay_buffer_samps;
  rp = vp + read_phase;

  while (n--) {
    // write input to delay buffer
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

    *out++ = *rp++;

    if (rp == ep) rp -= delay_buffer_samps;
  }

  x->x_phase = write_phase;
  return (w+5);
}

static void delay1_dsp(t_delay1 *x, t_signal **sp)
{
  dsp_add(delay1_perform, 4, x, sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_length);
  delay_set_system_params(x, sp[0]->s_length, sp[0]->s_sr);
  delay_buffer_update(x);
  delay_set_delay_samples(x, x->x_delay_msecs);
}

static void delay_free(t_delay1 *x)
{
  if (x->x_delay_buffer != NULL) {
    freebytes(x->x_delay_buffer,
              x->x_delay_buffer_samples * sizeof(t_sample));
    x->x_delay_buffer = NULL;
  }
}

void delay1_tilde_setup(void)
{
  delay1_class = class_new(gensym("delay1~"),
                          (t_newmethod)delay_new,
                          (t_method)delay_free,
                          sizeof(t_delay1),
                          CLASS_DEFAULT,
                          A_DEFFLOAT, A_DEFFLOAT, 0);

  class_addmethod(delay1_class, (t_method)delay1_dsp,
                  gensym("dsp"), A_CANT, 0);

  CLASS_MAINSIGNALIN(delay1_class, t_delay1, x_delay_buffer_msecs);
}
