#include "simple_del_shared.h"

typedef struct _multitap {
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
  int x_num_taps;
  int x_feedback_tap;

  t_float x_wet_dry;
  t_float x_feedback;

  t_inlet *x_delay_msec_inlet;

} t_multitap;

t_class *multitap_class = NULL;

static void delay_buffer_update(t_multitap *x);
static void delay_set_delay_samples(t_multitap *x, t_float f);

static void *multitap_new(t_floatarg buffer_msecs, t_floatarg delay_msecs)
{
  t_multitap *x = (t_multitap *)pd_new(multitap_class);

  x->x_delay_buffer_msecs = (buffer_msecs > 1) ? buffer_msecs : 1;
  x->x_delay_msecs = (delay_msecs > 1) ? delay_msecs : 1;

  x->x_s_per_msec = 0.0f;
  x->x_pd_block_size = 0;
  x->x_phase = 0;
  x->x_delay_samples = 0;
  
  x->x_delay_buffer_samples = 1024; // initialize with 2^10;
  x->x_delay_buffer = getbytes(x->x_delay_buffer_samples * sizeof(t_sample));
  if (x->x_delay_buffer == NULL) {
    pd_error(x, "multitap~: unable to assign memory to delay buffer");
    return NULL;
  }

  x->x_num_taps = 4; // hardcoded for now
  x->x_feedback_tap = 1;

  x->x_delay_msec_inlet = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
  // set inlet initial float value
  pd_float((t_pd *)x->x_delay_msec_inlet, x->x_delay_msecs);

  outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}

static void delay_buffer_update(t_multitap *x)
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
    pd_error(x, "multitap~: unable to resize x_delay_buffer");
    return;
  }

  x->x_delay_buffer_samples = buffer_size;
  x->x_phase = 0;
  post("multitap~: (debug) updated delay buffer");
  post("multitap~: (debug) x_delay_buffer_samples: %d", x->x_delay_buffer_samples);
}

static void delay_set_system_params(t_multitap *x, int blocksize, t_float sr)
{
  x->x_pd_block_size = blocksize;
  x->x_s_per_msec = sr * 0.001f;
}

static void delay_set_delay_samples(t_multitap *x, t_float f)
{
  x->x_delay_msecs = f;
  x->x_delay_samples = (int)(0.5 + x->x_s_per_msec * x->x_delay_msecs);
}

static t_int *multitap_perform(t_int *w)
{
  t_multitap *x = (t_multitap *)(w[1]);
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
  t_float tap_level = 1.0f / x->x_num_taps;

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

    t_sample out_delays = 0.0f;
    t_sample tap_delay = 0.0f;

    for (int i = 0; i < x->x_num_taps; i++) {
      int tap = i + 1;
      t_sample delsamps = x->x_s_per_msec * ((float)tap) * delms;

      if (!(delsamps >= 1.00001f)) delsamps = 1.00001f;
      if (delsamps > limit) delsamps = limit;

      int idelsamps = delsamps;
      int read_phase = (write_phase - idelsamps) & delay_buffer_mask;

      t_sample frac = delsamps - (t_sample)idelsamps;
      t_sample delay_line = cubic_interpolate(vp, read_phase, delay_buffer_mask, frac);
      out_delays += tap_level * delay_line;
      if (tap == x->x_feedback_tap) tap_delay = delay_line;
    }

    *out++ = wet_dry * out_delays + wet_dry_inv * f;

    vp[write_phase] = f * feedback_inv + tap_delay * feedback;

    write_phase = (write_phase + 1) & delay_buffer_mask;
  }

  x->x_phase = write_phase;
  return (w+6);
}

static void multitap_dsp(t_multitap *x, t_signal **sp)
{
  dsp_add(multitap_perform, 5, x, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[0]->s_length);
  delay_set_system_params(x, sp[0]->s_length, sp[0]->s_sr);
  delay_buffer_update(x);
  delay_set_delay_samples(x, x->x_delay_msecs);
}

static void delay_free(t_multitap *x)
{
  if (x->x_delay_buffer != NULL) {
    freebytes(x->x_delay_buffer,
              x->x_delay_buffer_samples * sizeof(t_sample));
    x->x_delay_buffer = NULL;
  }
}

static void delay_wet_dry(t_multitap *x, t_floatarg f)
{
  if (f < 0.0f || f > 1.0f) {
    pd_error(x, "multitap~: wet/dry mix must be in the range (0, 1). Setting to 0.");
    f = 0.0f;
  }
  x->x_wet_dry = f;
}

static void delay_feedback(t_multitap *x, t_floatarg f)
{
  if (f < 0.0f || f > 0.99f) {
    pd_error(x, "multitap~: feedback must be in the range (0, 1). Setting to 0");
    f = 0.0f;
  }
  x->x_feedback = f;
}

static void delay_taps(t_multitap *x, t_floatarg f)
{
  if (f < 1) {
    pd_error(x, "multitap~: there needs to be at least 1 tap. Setting to 1");
    f = 1.0f;
  }
  x->x_num_taps = (int)f;
}

static void delay_feedback_tap(t_multitap *x, t_floatarg f)
{
  x->x_feedback_tap = (int)f;
}

void multitap_tilde_setup(void)
{
  multitap_class = class_new(gensym("multitap~"),
                          (t_newmethod)multitap_new,
                          (t_method)delay_free,
                          sizeof(t_multitap),
                          CLASS_DEFAULT,
                          A_DEFFLOAT, A_DEFFLOAT, 0);

  class_addmethod(multitap_class, (t_method)multitap_dsp,
                  gensym("dsp"), A_CANT, 0);

  class_addmethod(multitap_class, (t_method)delay_wet_dry,
                  gensym("wet_dry"), A_FLOAT, 0);
  class_addmethod(multitap_class, (t_method)delay_feedback,
                  gensym("feedback"), A_FLOAT, 0);
  class_addmethod(multitap_class, (t_method)delay_taps,
                  gensym("taps"), A_FLOAT, 0);
  class_addmethod(multitap_class, (t_method)delay_feedback_tap,
                  gensym("feedback_tap"), A_FLOAT, 0);

  // dummy float arg is required by Pd
  CLASS_MAINSIGNALIN(multitap_class, t_multitap, x_delay_buffer_msecs);
}
