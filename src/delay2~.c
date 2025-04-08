#include "simple_del_shared.h"
#include <m_pd.h>

typedef struct _delay2 {
  t_object x_obj;

  t_float x_s_per_msec; // samples per msec
  t_float x_delay_buffer_msecs;
  int x_delay_buffer_samples; // number of samples in delay buffer
  int x_delay_buffer_initial_samples;
  t_float x_delay_msecs; // number of msecs to delay
  t_float x_delay_samples; // number of samples of delay
  t_sample *x_delay_buffer;
  int x_pd_block_size;
  int x_phase; // current __write__ position
  t_float x_tap1_level;
  t_float x_tap2_level;

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

  x->x_delay_buffer_msecs = (buffer_msecs > 1) ? buffer_msecs : 1;
  x->x_delay_msecs = (delay_msecs > 1) ? delay_msecs : 1;

  x->x_s_per_msec = 0.0f;
  x->x_pd_block_size = 0;
  x->x_phase = 0;
  x->x_delay_samples = 0;
  
  x->x_delay_buffer_samples = 1024; // initialize with 2^10;
  x->x_delay_buffer = getbytes(x->x_delay_buffer_samples * sizeof(t_sample));
  if (x->x_delay_buffer == NULL) {
    pd_error(x, "delay2~: unable to assign memory to delay buffer");
    return NULL;
  }

  x->x_tap1_level = 0.5f;
  x->x_tap2_level = 0.5f;

  x->x_delay_msec_inlet = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
  // set inlet initial float value
  pd_float((t_pd *)x->x_delay_msec_inlet, x->x_delay_msecs);

  outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}

static void delay_buffer_update(t_delay2 *x)
{
  int buffer_size = 1;
  while (buffer_size < (x->x_delay_buffer_msecs * x->x_s_per_msec + x->x_pd_block_size)) {
    buffer_size *= 2;
  }

  x->x_delay_buffer = (t_sample *)resizebytes(x->x_delay_buffer,
                                              x->x_delay_buffer_samples *
                                              sizeof(t_sample), buffer_size * sizeof(t_sample));
  // todo: maybe return an int value for success/failure?
  if (x->x_delay_buffer == NULL) {
    pd_error(x, "delay2~: unable to resize x_delay_buffer");
    return;
  }

  x->x_delay_buffer_samples = buffer_size;
  x->x_phase = 0;
  post("delay2~: (debug) updated delay buffer");
  post("delay2~: (debug) x_delay_buffer_samples: %d", x->x_delay_buffer_samples);
}

static void delay_set_system_params(t_delay2 *x, int blocksize, t_float sr)
{
  x->x_pd_block_size = blocksize;
  x->x_s_per_msec = sr * 0.001f;
}

static void delay_set_delay_samples(t_delay2 *x, t_float f)
{
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
  int delay_buffer_mask = delay_buffer_samples - 1;
  int write_phase = x->x_phase;
  write_phase = write_phase & delay_buffer_mask;

  t_sample *vp = x->x_delay_buffer;

  t_float wet_dry = x->x_wet_dry;
  t_float wet_dry_inv = 1.0f - wet_dry;
  t_float feedback = x->x_feedback;
  t_float feedback_inv = 1.0f - feedback;
  t_float tap1_level = x->x_tap1_level;
  t_float tap2_level = x->x_tap2_level;

  t_sample limit = delay_buffer_samples - n;
  if (limit < 0) {
    while (n--) {
      t_sample f = *in1++;
      if (PD_BIGORSMALL(f)) f = 0.0f;
      vp[write_phase] = f;
      *out++ = 0;
      write_phase = (write_phase + 1) & delay_buffer_mask;
    }
    x->x_phase = write_phase;
    return (w+6);
  }

  while (n--) {
    t_sample f = *in1++;
    if (PD_BIGORSMALL(f)) f = 0.0f;

    t_sample delms = *in2++;

    // first tap
    t_sample delsamps1 = x->x_s_per_msec * delms;

    if (!(delsamps1 >= 1.00001f)) delsamps1 = 1.00001f;
    if (delsamps1 > limit) delsamps1 = limit;

    int idelsamps1 = delsamps1;
    int read_phase1 = (write_phase - idelsamps1) & delay_buffer_mask;

    t_sample frac1 = delsamps1 - (t_sample)idelsamps1;

    t_sample delayed_output1 = cubic_interpolate(vp, read_phase1, delay_buffer_mask, frac1);

    // second tap
    t_sample delsamps2 = x->x_s_per_msec * 2.0f * delms;

    if (!(delsamps2 > 1.00001f)) delsamps2 = 1.00001f;
    if (delsamps2 > limit) delsamps2 = limit;

    int idelsamps2 = delsamps2;
    int read_phase2 = (write_phase - idelsamps2) & delay_buffer_mask;

    t_sample frac2 = delsamps2 - (t_sample)idelsamps2;

    t_sample delayed_output2 = cubic_interpolate(vp, read_phase2, delay_buffer_mask, frac2);

    // mix the taps
    t_sample output = delayed_output1 * tap1_level + delayed_output2 * tap2_level;

    *out++ = wet_dry * output + wet_dry_inv * f;

    vp[write_phase] = f * feedback_inv + delayed_output1 * feedback;

    write_phase = (write_phase + 1) & delay_buffer_mask;
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
              x->x_delay_buffer_samples * sizeof(t_sample));
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

  // dummy float arg is required by Pd
  CLASS_MAINSIGNALIN(delay2_class, t_delay2, x_delay_buffer_msecs);
}
