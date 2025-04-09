#include "simple_del_shared.h"
#include <m_pd.h>

typedef struct _stereotaps {
  t_object x_obj;

  t_float x_s_per_msec; // samples per msec
  t_float x_delay_buffer_msecs;
  int x_delay_buffer_samples; // number of samples in delay buffer
  int x_delay_buffer_initial_samples;
  t_float x_delay_msecs; // number of msecs to delay
  t_float x_delay_samples; // number of samples of delay
  t_sample *x_delay_buffer_l;
  t_sample *x_delay_buffer_r;
  int x_pd_block_size;
  int x_phase; // current __write__ position
  int x_num_taps;
  int x_feedback_tap_l;
  int x_feedback_tap_r;

  t_float x_wet_dry;
  t_float x_feedback;

  t_inlet *x_delay_msec_inlet;
  t_outlet *x_out1;
  t_outlet *x_out2;

} t_stereotaps;

t_class *stereotaps_class = NULL;

static void delay_buffer_update(t_stereotaps *x);
static void delay_set_delay_samples(t_stereotaps *x, t_float f);

static void *stereotaps_new(t_floatarg buffer_msecs, t_floatarg delay_msecs)
{
  t_stereotaps *x = (t_stereotaps *)pd_new(stereotaps_class);

  x->x_delay_buffer_msecs = (buffer_msecs > 1) ? buffer_msecs : 1;
  x->x_delay_msecs = (delay_msecs > 1) ? delay_msecs : 1;

  x->x_s_per_msec = 0.0f;
  x->x_pd_block_size = 0;
  x->x_phase = 0;
  x->x_delay_samples = 0;
  
  x->x_delay_buffer_samples = 1024; // initialize with 2^10;

  x->x_delay_buffer_l = getbytes(x->x_delay_buffer_samples * sizeof(t_sample));
  if (x->x_delay_buffer_l == NULL) {
    pd_error(x, "stereotaps~: unable to assign memory to delay buffer");
    return NULL;
    }
  x->x_delay_buffer_r = getbytes(x->x_delay_buffer_samples * sizeof(t_sample));
  if (x->x_delay_buffer_r == NULL) {
    pd_error(x, "stereotaps~: unable to assign memory to delay buffer");
    return NULL;
    }

  x->x_num_taps = 4; // hardcoded for now
  x->x_feedback_tap_l = 3; // these probably don't make sense as default values
  x->x_feedback_tap_r = 4;

  x->x_delay_msec_inlet = inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
  // set inlet initial float value
  pd_float((t_pd *)x->x_delay_msec_inlet, x->x_delay_msecs);

  x->x_out1 = outlet_new(&x->x_obj, &s_signal);
  x->x_out2 = outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}

static void delay_buffer_update(t_stereotaps *x)
{
  int buffer_size = 1;
  while (buffer_size < (x->x_delay_buffer_msecs * x->x_s_per_msec + x->x_pd_block_size)) {
    buffer_size *= 2;
  }

  x->x_delay_buffer_l = (t_sample *)resizebytes(x->x_delay_buffer_l,
                                              x->x_delay_buffer_samples *
                                              sizeof(t_sample), buffer_size * sizeof(t_sample));
  if (x->x_delay_buffer_l == NULL) {
    pd_error(x, "stereotaps~: unable to resize x_delay_buffer_l");
    return;
    }
  x->x_delay_buffer_r = (t_sample *)resizebytes(x->x_delay_buffer_r,
                                              x->x_delay_buffer_samples *
                                              sizeof(t_sample), buffer_size * sizeof(t_sample));
  if (x->x_delay_buffer_r == NULL) {
    pd_error(x, "stereotaps~: unable to resize x_delay_buffer_r");
    return;
    }

  x->x_delay_buffer_samples = buffer_size;
  x->x_phase = 0;
  post("stereotaps~: (debug) updated delay buffer");
  post("stereotaps~: (debug) x_delay_buffer_samples: %d", x->x_delay_buffer_samples);
}

static void delay_set_system_params(t_stereotaps *x, int blocksize, t_float sr)
{
  x->x_pd_block_size = blocksize;
  x->x_s_per_msec = sr * 0.001f;
}

static void delay_set_delay_samples(t_stereotaps *x, t_float f)
{
  x->x_delay_msecs = f;
  x->x_delay_samples = (int)(0.5 + x->x_s_per_msec * x->x_delay_msecs);
}

static t_int *stereotaps_perform(t_int *w)
{
  t_stereotaps *x = (t_stereotaps *)(w[1]);
  t_sample *in1 = (t_sample *)(w[2]);
  t_sample *in2 = (t_sample *)(w[3]);
  t_sample *out1 = (t_sample *)(w[4]);
  t_sample *out2 = (t_sample *)(w[5]);
  int n = (int)(w[6]);

  int delay_buffer_samples = x->x_delay_buffer_samples;
  int delay_buffer_mask = delay_buffer_samples - 1;
  int write_phase = x->x_phase;
  write_phase = write_phase & delay_buffer_mask;

  // todo: get pointers to left and right buffers
  t_sample *vpl = x->x_delay_buffer_l;
  t_sample *vpr = x->x_delay_buffer_r;

  t_float wet_dry = x->x_wet_dry;
  t_float wet_dry_inv = (1.0f - wet_dry);
  t_float feedback = x->x_feedback;
  t_float feedback_inv = (1.0f - feedback);
  t_float tap_level = (1.0f / x->x_num_taps);

  t_sample limit = delay_buffer_samples - n;

  // separate stereo/mono code for now
  if (limit < 0) {
    while (n--) {
      t_sample f = *in1++;
      if (PD_BIGORSMALL(f)) f = 0.0f;
      vpl[write_phase] = 0.5f * f;
      vpr[write_phase] = 0.5f * f;
      *out1++ = 0;
      *out2++ = 0;
      write_phase = (write_phase + 1) & delay_buffer_mask;
    }
    x->x_phase = write_phase;
    return (w+7);
  }

  while (n--) {
    t_sample f = *in1++;
    if (PD_BIGORSMALL(f)) f = 0.0f;

    t_sample delms = *in2++;

    t_sample out_delays_left = 0.0f;
    t_sample out_delays_right = 0.0f;
    t_sample tap_delay_left = 0.0f;
    t_sample tap_delay_right = 0.0f;

    for (int i = 0; i < x->x_num_taps; i++) {
      int tap = i + 1;
      t_sample delsamps = x->x_s_per_msec * (float)tap * delms;

      if (!(delsamps >= 1.00001f)) delsamps = 1.00001f;
      if (delsamps > limit) delsamps = limit;

      int idelsamps = delsamps;
      int read_phase = (write_phase - idelsamps) & delay_buffer_mask;
      t_sample frac = delsamps - (t_sample)idelsamps;
      t_sample delay_line_left = cubic_interpolate(vpl, read_phase, delay_buffer_mask, frac);
      t_sample delay_line_right = cubic_interpolate(vpr, read_phase, delay_buffer_mask, frac);
      out_delays_left += tap_level * delay_line_left;
      out_delays_right += tap_level * delay_line_right;
      
      if (tap == x->x_feedback_tap_l) tap_delay_left = delay_line_left;
      if (tap == x->x_feedback_tap_r) tap_delay_right = delay_line_right;
    }

    *out1++ = wet_dry * out_delays_left + wet_dry_inv * f;
    *out2++ = wet_dry * out_delays_right + wet_dry_inv * f;

    vpl[write_phase] = f * feedback_inv + tap_delay_left * feedback;
    vpr[write_phase] = f * feedback_inv + tap_delay_right * feedback;

    write_phase = (write_phase + 1) & delay_buffer_mask;
  }

  x->x_phase = write_phase;
  return (w+7);
}

static void stereotaps_dsp(t_stereotaps *x, t_signal **sp)
{
  dsp_add(stereotaps_perform, 6, x, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec, sp[3]->s_vec, sp[0]->s_length);
  delay_set_system_params(x, sp[0]->s_length, sp[0]->s_sr);
  delay_buffer_update(x);
  delay_set_delay_samples(x, x->x_delay_msecs);
}

static void delay_free(t_stereotaps *x)
{
  if (x->x_delay_buffer_l != NULL) {
    freebytes(x->x_delay_buffer_l,
              x->x_delay_buffer_samples * sizeof(t_sample));
    x->x_delay_buffer_l = NULL;
  }

  if (x->x_delay_buffer_r != NULL) {
    freebytes(x->x_delay_buffer_r,
              x->x_delay_buffer_samples * sizeof(t_sample));
    x->x_delay_buffer_r = NULL;
  }

  if (x->x_out1 != NULL) {
    outlet_free(x->x_out1);
  }
  if (x->x_out2 != NULL) {
    outlet_free(x->x_out2);
  }
}

static void delay_wet_dry(t_stereotaps *x, t_floatarg f)
{
  if (f < 0.0f || f > 1.0f) {
    pd_error(x, "stereotaps~: wet/dry mix must be in the range (0, 1). Setting to 0.");
    f = 0.0f;
  }
  x->x_wet_dry = f;
}

static void delay_feedback(t_stereotaps *x, t_floatarg f)
{
  if (f < 0.0f || f > 0.99f) {
    pd_error(x, "stereotaps~: feedback must be in the range (0, 1). Setting to 0");
    f = 0.0f;
  }
  x->x_feedback = f;
}

static void delay_taps(t_stereotaps *x, t_floatarg f)
{
  if (f < 1) {
    pd_error(x, "stereotaps~: there needs to be at least 1 tap. Setting to 1");
    f = 1.0f;
  }
  x->x_num_taps = (int)f;
}

static void delay_feedback_tap_l(t_stereotaps *x, t_floatarg f)
{
  x->x_feedback_tap_l = (int)f;
}

static void delay_feedback_tap_r(t_stereotaps *x, t_floatarg f)
{
  x->x_feedback_tap_r = (int)f;
}

void stereotaps_tilde_setup(void)
{
  stereotaps_class = class_new(gensym("stereotaps~"),
                          (t_newmethod)stereotaps_new,
                          (t_method)delay_free,
                          sizeof(t_stereotaps),
                          CLASS_DEFAULT,
                          A_DEFFLOAT, A_DEFFLOAT, 0);

  class_addmethod(stereotaps_class, (t_method)stereotaps_dsp,
                  gensym("dsp"), A_CANT, 0);

  class_addmethod(stereotaps_class, (t_method)delay_wet_dry,
                  gensym("wet_dry"), A_FLOAT, 0);
  class_addmethod(stereotaps_class, (t_method)delay_feedback,
                  gensym("feedback"), A_FLOAT, 0);
  class_addmethod(stereotaps_class, (t_method)delay_taps,
                  gensym("taps"), A_FLOAT, 0);
  class_addmethod(stereotaps_class, (t_method)delay_feedback_tap_l,
                  gensym("feedback_tap_l"), A_FLOAT, 0);
  class_addmethod(stereotaps_class, (t_method)delay_feedback_tap_r,
                  gensym("feedback_tap_r"), A_FLOAT, 0);

  // dummy float arg is required by Pd
  CLASS_MAINSIGNALIN(stereotaps_class, t_stereotaps, x_delay_buffer_msecs);
}
