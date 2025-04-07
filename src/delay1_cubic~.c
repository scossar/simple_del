#include "simple_del_shared.h"
#include <m_pd.h>

typedef struct _delay1_cubic {
  t_object x_obj;

  t_float x_s_per_msec; // samples per msec
  t_float x_delay_buffer_msecs;
  int x_delay_buffer_samples; // number of samples in delay buffer
  int x_delay_msecs; // number of msecs to delay
  t_float x_delay_samples; // number of samples of delay
  t_sample *x_delay_buffer;
  int x_pd_block_size;
  int x_phase; // current __write__ position

} t_delay1_cubic;

t_class *delay1_cubic_class = NULL;

static void delay_buffer_update(t_delay1_cubic *x);
static void delay_set_delay_samples(t_delay1_cubic *x, t_float f);

static void *delay_new(t_floatarg buffer_msecs, t_floatarg delay_msecs)
{
  t_delay1_cubic *x = (t_delay1_cubic *)pd_new(delay1_cubic_class);

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

static void delay_buffer_update(t_delay1_cubic *x)
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

static void delay_set_system_params(t_delay1_cubic *x, int blocksize, t_float sr)
{
  x->x_pd_block_size = blocksize;
  x->x_s_per_msec = sr * 0.001f;
}

static void delay_set_delay_samples(t_delay1_cubic *x, t_float f)
{
  // currently this is reassigning x->x_delay_msecs to x->x_delay_msecs
  // this approach might make sense later when x->delay_msecs can be set via a
  // message instead of just as a creating argument.
  x->x_delay_msecs = f;
  x->x_delay_samples = x->x_s_per_msec * x->x_delay_msecs;
}

static t_int *delay1_cubic_perform(t_int *w)
{
  t_delay1_cubic *x = (t_delay1_cubic*)(w[1]);
  t_sample *in1 = (t_sample *)(w[2]);
  t_sample *out = (t_sample *)(w[3]);
  int n = (int)(w[4]);

  int delay_buffer_samps = x->x_delay_buffer_samples;
  int delay_samples_int = (int)x->x_delay_samples;

  float delay_frac = x->x_delay_samples - delay_samples_int;
  int write_phase = x->x_phase;
  write_phase += n; // increment write position by block size for new loop

  int read_phase = write_phase - delay_samples_int;
  if (read_phase < 0) read_phase += delay_buffer_samps; 

  t_sample *vp = x->x_delay_buffer;
  t_sample *wp = vp + write_phase;
  t_sample *ep = vp + (x->x_delay_buffer_samples + XTRASAMPS);

  while (n--) {
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

    if (read_phase < XTRASAMPS) read_phase += delay_samples_int;
    t_sample a = vp[read_phase];
    t_sample b = vp[read_phase - 1];
    t_sample c = vp[read_phase - 2];
    t_sample d = vp[read_phase - 3];
    t_sample cminusb = c-b;

    *out++ = b + delay_frac * (
        cminusb - 0.1666667f * (1.-delay_frac) * (
            (d - a - 3.0f * cminusb) * delay_frac + (d + 2.0f*a - 3.0f*b)
        )
    );

    read_phase++;
    if (read_phase > delay_buffer_samps) {
      read_phase -= delay_buffer_samps;
    }
  }

  x->x_phase = write_phase;
  return (w+5);
}

static void delay1_cubic_dsp(t_delay1_cubic *x, t_signal **sp)
{
  dsp_add(delay1_cubic_perform, 4, x, sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_length);
  delay_set_system_params(x, sp[0]->s_length, sp[0]->s_sr);
  delay_buffer_update(x);
  delay_set_delay_samples(x, x->x_delay_msecs);
}

static void delay_free(t_delay1_cubic *x)
{
  if (x->x_delay_buffer != NULL) {
    freebytes(x->x_delay_buffer,
              (x->x_delay_buffer_samples + XTRASAMPS) * sizeof(t_sample));
    x->x_delay_buffer = NULL;
  }
}

void delay1_cubic_tilde_setup(void)
{
  delay1_cubic_class = class_new(gensym("delay1_cubic~"),
                          (t_newmethod)delay_new,
                          (t_method)delay_free,
                          sizeof(t_delay1_cubic),
                          CLASS_DEFAULT,
                          A_DEFFLOAT, A_DEFFLOAT, 0);

  class_addmethod(delay1_cubic_class, (t_method)delay1_cubic_dsp,
                  gensym("dsp"), A_CANT, 0);

  CLASS_MAINSIGNALIN(delay1_cubic_class, t_delay1_cubic, x_delay_buffer_msecs);
}
