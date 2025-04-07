#include "simple_del_shared.h"
#include <m_pd.h>

typedef struct _delay2 {
  t_object x_obj;

  t_float x_s_per_msec; // samples per msec
  t_float x_delay_buffer_msecs;
  int x_delay_buffer_samples; // number of samples in delay buffer
  t_float x_delay_msecs; // number of msecs to delay
  t_float x_delay_samples; // number of samples of delay
  t_sample *x_delay_buffer;
  int x_pd_block_size;
  int x_phase; // current __write__ position

  t_float x_wet_dry;
  t_float x_feedback;

  t_inlet *x_delay_msec_inlet;

} t_delay2;

t_class *delay2_class = NULL;

static void delay_buffer_update(t_delay2 *x);
static void delay_set_delay_samples(t_delay2 *x, t_float f);

static void *delay2_new(t_floatarg buffer_msecs, t_floatarg delay_msecs)
{
  t_delay2 *x = (t_delay2 *)pd_new(delay2_class);

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

  x->x_delay_msec_inlet = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
  // set inlet initial float value
  pd_float((t_pd *)x->x_delay_msec_inlet, x->x_delay_msecs);

  outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}

static void delay_initialize_delay_buffer(t_delay2 *x)
{
  int buffer_size = 1;
  while (buffer_size < x->x_delay_buffer_msecs * x->x_s_per_msec + XTRASAMPS + x->x_pd_block_size) {
    buffer_size *= 2;
  }

  x->x_delay_buffer = (t_sample *)resizebytes(x->x_delay_buffer,
                                              x->x_delay_buffer_samples *
                                              sizeof(t_sample), buffer_size * sizeof(t_sample));
  x->x_delay_buffer_samples = buffer_size;
}

static void delay_buffer_update(t_delay2 *x)
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

static void delay_set_system_params(t_delay2 *x, int blocksize, t_float sr)
{
  x->x_pd_block_size = blocksize;
  x->x_s_per_msec = sr * 0.001f;
}

static void delay_set_delay_samples(t_delay2 *x, t_float f)
{
  // currently this is reassigning x->x_delay_msecs to x->x_delay_msecs
  // this approach might make sense later when x->delay_msecs can be set via a
  // message instead of just as a creating argument.
  x->x_delay_msecs = f;
  x->x_delay_samples = (int)(0.5 + x->x_s_per_msec * x->x_delay_msecs);
}

static t_int *delay2_perform(t_int *w)
{
  t_delay2 *x = (t_delay2*)(w[1]);
  t_sample *in1 = (t_sample *)(w[2]);
  t_sample *in2 = (t_sample *)(w[3]);
  t_sample *out = (t_sample *)(w[4]);
  int n = (int)(w[5]);

  int delay_buffer_samples = x->x_delay_buffer_samples;
  int write_phase = x->x_phase;
  write_phase += n; // increment write position by block size for new loop

  t_sample *vp = x->x_delay_buffer;
  t_sample *wp = vp + write_phase;
  t_sample *ep = vp + (x->x_delay_buffer_samples + XTRASAMPS);

  t_sample fn = n - 1; // last index of n

  t_float wet_dry = x->x_wet_dry;
  t_float wet_dry_inv = 1.0f - wet_dry;
  t_float feedback = x->x_feedback;
  t_float feedback_inv = 1.0f - feedback;

  t_sample limit = delay_buffer_samples - n;
  if (limit < 0) {
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
        write_phase -= delay_buffer_samples;
      }

      *out++ = 0;
    }
    return (w+6);
  }

  while (n--) {
    t_sample f = *in1++;
    if (PD_BIGORSMALL(f)) f = 0.0f;
    if (wp == ep) {
      vp[0] = ep[-4];
      vp[1] = ep[-3];
      vp[2] = ep[-2];
      vp[3] = ep[-1];
      wp = vp + XTRASAMPS;
      write_phase -= delay_buffer_samples;
    }

    t_sample delsamps = x->x_s_per_msec * *in2++;
    int idelsamps;

    if (!(delsamps >= 1.00001f)) delsamps = 1.00001f;
    if (delsamps > limit) delsamps = limit;

    // create a sliding window of pd block size
    delsamps += fn;
    fn = fn - 1.0f;
    idelsamps = delsamps;
    t_sample delay_frac = delsamps - (t_sample)idelsamps;
    int read_phase = write_phase - idelsamps;

    // possibly delay_buffer_samples should be set to a power of 2 so that bit
    // masking can be used here
    // there's an example in `linreg~.c` for how that could be done
    if (read_phase < XTRASAMPS) read_phase += delay_buffer_samples;
    t_sample a = vp[read_phase];
    t_sample b = vp[read_phase - 1];
    t_sample c = vp[read_phase - 2];
    t_sample d = vp[read_phase - 3];
    t_sample cminusb = c-b;

    t_sample delayed_output = b + delay_frac * (
        cminusb - 0.1666667f * (1.-delay_frac) * (
            (d - a - 3.0f * cminusb) * delay_frac + (d + 2.0f*a - 3.0f*b)
        )
    );

    *out++ = wet_dry * delayed_output + wet_dry_inv * f;

    *wp++ = f * feedback_inv + delayed_output * feedback;
  }

  x->x_phase = write_phase;
  return (w+6);
}

static void delay2_dsp(t_delay2 *x, t_signal **sp)
{
  dsp_add(delay2_perform, 5, x, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[0]->s_length);
  delay_set_system_params(x, sp[0]->s_length, sp[0]->s_sr);
  delay_buffer_update(x);
  delay_set_delay_samples(x, x->x_delay_msecs);
}

static void delay_free(t_delay2 *x)
{
  if (x->x_delay_buffer != NULL) {
    freebytes(x->x_delay_buffer,
              (x->x_delay_buffer_samples + XTRASAMPS) * sizeof(t_sample));
    x->x_delay_buffer = NULL;
  }
}

static void delay_wet_dry(t_delay2 *x, t_floatarg f)
{
  if (f < 0.0f || f > 1.0f) {
    pd_error(x, "delay2~: wet/dry mix must be in the range (0, 1). Setting to 0.");
    f = 0.0f;
  }
  x->x_wet_dry = f;
}

static void delay_feedback(t_delay2 *x, t_floatarg f)
{
  if (f < 0.0f || f > 0.99f) {
    pd_error(x, "delay2~: feedback must be in the range (0, 1). Setting to 0");
    f = 0.0f;
  }
  x->x_feedback = f;
}

void delay2_tilde_setup(void)
{
  delay2_class = class_new(gensym("delay2~"),
                          (t_newmethod)delay2_new,
                          (t_method)delay_free,
                          sizeof(t_delay2),
                          CLASS_DEFAULT,
                          A_DEFFLOAT, A_DEFFLOAT, 0);

  class_addmethod(delay2_class, (t_method)delay2_dsp,
                  gensym("dsp"), A_CANT, 0);

  class_addmethod(delay2_class, (t_method)delay_wet_dry,
                  gensym("wet_dry"), A_FLOAT, 0);
  class_addmethod(delay2_class, (t_method)delay_feedback,
                  gensym("feedback"), A_FLOAT, 0);

  CLASS_MAINSIGNALIN(delay2_class, t_delay2, x_delay_buffer_msecs);
}
