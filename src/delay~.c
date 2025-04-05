#include "simple_del_shared.h"
#include <m_pd.h>

typedef struct _delay {
  t_object x_obj;

  t_float x_s_per_msec; // samples per msec
  t_float x_delay_buffer_msecs;
  int x_delay_buffer_samples; // number of samples in delay buffer
  int x_delay_msecs; // number of msecs to delay
  int x_delay_samples; // number of samples of delay
  t_sample *x_delay_buffer;
  int x_pd_block_size;
  int x_phase; // current __write__ position
} t_delay;

t_class *delay_class = NULL;

static void delay_buffer_update(t_delay *x);
static void delay_set_delay_samples(t_delay *x, t_float f);

static void *delay_new(t_floatarg msecs)
{
  t_delay *x = (t_delay *)pd_new(delay_class);

  x->x_delay_buffer_msecs = msecs;

  x->x_s_per_msec = 0.0f;
  x->x_delay_buffer_samples = 0;
  x->x_pd_block_size = 0;
  x->x_phase = 0;
  x->x_delay_msecs = 0;
  x->x_delay_samples = 0;
  
  x->x_delay_buffer = getbytes(XTRASAMPS * sizeof(t_sample));

  delay_buffer_update(x);
  delay_set_delay_samples(x, x->x_delay_msecs);

  outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}

static void delay_buffer_update(t_delay *x)
{
  int nsamps = x->x_delay_buffer_msecs * x->x_s_per_msec;
  if (nsamps < 1) nsamps = 1;

  // round up (?) to a multiple of SAMPBLK
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
  if (blocksize > x->x_pd_block_size) {
    x->x_pd_block_size = blocksize;
  }

  // awkward, but maybe clearer than the original source code?
  if ((sr * 0.001f) > x->x_s_per_msec) {
    x->x_s_per_msec = sr * 0.001f;
  }
}

static void delay_set_delay_samples(t_delay *x, t_float f)
{
  // x->x_delay_msecs = f;
  x->x_delay_msecs = 250;
  x->x_delay_samples = (int)(0.5 + x->x_s_per_msec * x->x_delay_msecs) + x->x_pd_block_size;
}

static t_int *delay_perform(t_int *w)
{
  t_delay *x = (t_delay*)(w[1]);
  t_sample *in = (t_sample *)(w[2]);
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
    t_sample f = *in++;

    if (PD_BIGORSMALL(f)) f = 0.0f;

    *wp++ = f;

    *out++ = *rp++;

    if (wp == ep) {
      vp[0] = ep[-4];
      vp[1] = ep[-3];
      vp[2] = ep[-2];
      vp[3] = ep[-1];
      wp = vp + XTRASAMPS;
      write_phase -= delay_buffer_samps;
    }
    if (rp == ep) rp -= delay_buffer_samps;
  }

  x->x_phase = write_phase;
  return (w+5);
}

static void delay_dsp(t_delay *x, t_signal **sp)
{
  dsp_add(delay_perform, 4, x, sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_length);
  delay_set_system_params(x, sp[0]->s_length, sp[0]->s_sr);
  delay_buffer_update(x);
  delay_set_delay_samples(x, x->x_delay_msecs);
  post("delay msecs: %f", x->x_delay_msecs);
}

static void delay_free(t_delay *x)
{
  if (x->x_delay_buffer != NULL) {
    freebytes(x->x_delay_buffer,
              x->x_delay_buffer_samples * sizeof(t_sample));
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
