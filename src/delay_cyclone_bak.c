#include "m_pd.h"
#include <string.h>

#define DELAY_DEFMAXSIZE 512
#define DELAY_GUARD 4

typedef struct _delay {
  t_object x_obj;
  t_inlet *x_inlet;
  t_sample *x_buf; // should this be t_sample?
  t_sample *x_bufend;
  t_sample *x_whead;
  int x_sr;
  int x_maxsize; // buffer size in samples
  int x_maxsofar; // largest max size so far
  int x_delsize; // current delay in samples
  int x_prevdelsize; // previous delay in samples
  t_float x_ramplength; // length of ramp in ms
  double x_rampinc; // samplewise ramp increment
  double x_rampvals[2]; // current ramp scalar values [0] current [1] previous
  int x_remain; // number of samples remaining in ramp
  // int x_hasfeeders; // true if right inlet has signal input (probably leaving
  // off)
  t_float x_bufini[DELAY_DEFMAXSIZE + 2*DELAY_GUARD - 1]; // default stack
  // buffer
} t_delay;

static t_class *delay_class = NULL;

static void delay_clear(t_delay *x)
{
  // note the use of the deference operator (*)
  memset(x->x_buf, 0, (x->x_maxsize + 2*DELAY_GUARD - 1) * sizeof(t_sample));
}

static void delay_bounds(t_delay *x)
{
  x->x_whead = x->x_buf;
  x->x_bufend = x->x_buf + x->x_maxsize;
}

static void delay_maxsize(t_delay *x, t_float f)
{
  int maxsize = (f < 1 ? 1 : (int)(f));

  if (maxsize > x->x_maxsofar) {
    x->x_maxsofar = maxsize;
    if (x->x_buf == x->x_bufini) {
      x->x_buf = (t_float *)getbytes((maxsize + 2*DELAY_GUARD - 1) * sizeof(t_sample));
      if (x->x_buf == NULL) {
        x->x_buf = x->x_bufini;
        x->x_maxsize = DELAY_DEFMAXSIZE;
        pd_error(x, "delay~: unable to resize buffer; using size %d", DELAY_DEFMAXSIZE);
      }
    } else if (x->x_buf) {
      x->x_buf = (t_float *)resizebytes(x->x_buf,
                  (x->x_maxsize + 2*DELAY_GUARD - 1) * sizeof(t_sample),
                  (maxsize + 2*DELAY_GUARD - 1) * sizeof(t_sample));
      if (x->x_buf == NULL) {
        x->x_buf = x->x_bufini;
        x->x_maxsize = DELAY_DEFMAXSIZE;
        pd_error(x, "delay~: unable to resize buffer; using size %d", DELAY_DEFMAXSIZE);
      }
    }
  }

  x->x_maxsize = maxsize;
  if (x->x_delsize > maxsize) {
    x->x_delsize = maxsize;
  }
  x->x_remain = 0;
  delay_clear(x);
  delay_bounds(x);
}

static void delay_delay(t_delay *x, t_floatarg f)
{
  x->x_prevdelsize = x->x_delsize;
  x->x_delsize = (f > 0 ? (int)f : 0);

  if (x->x_delsize > x->x_maxsize) {
    x->x_delsize = x->x_maxsize;
  }

  x->x_remain = (int)(x->x_ramplength * x->x_sr * 0.001);
  x->x_rampinc = (double)(1.0)/(double)(x->x_remain);
  x->x_rampvals[0] = 0.0;
  x->x_rampvals[1] = 1.0;
}

static void delay_ramp(t_delay *x, t_floatarg f)
{
  x->x_ramplength = (f < 0.0 ? 0.0f : f);
}

static t_int *delay_perform(t_int *w)
{
  t_delay *x = (t_delay *)(w[1]);
  int nblock = (int)(w[2]);
  t_sample *in1 = (t_sample *)(w[3]);
  t_sample *in2 = (t_sample *)(w[4]);
  t_sample *out = (t_sample *)(w[5]);
  t_sample *buf = x->x_buf;
  t_sample *ep = x->x_bufend;
  t_sample *wp = x->x_whead;
  int maxsize = x->x_maxsize;

  if (*in2 != x->x_delsize && !x->x_remain) {
    delay_delay(x, *in2); // hmmm: second arg seems to be of wrong type
  }

  int nleft = x->x_remain;

  if (nleft) {
    double incr = x->x_rampinc;
    double currramp = x->x_rampvals[0];
    double prevramp = x->x_rampvals[1];
    t_sample *prevrp = wp - x->x_prevdelsize;
    t_sample *currrp = wp - x->x_delsize;
    if (prevrp < buf) prevrp += maxsize + 1;
    if (currrp < buf) currrp += maxsize + 1;

    if (nleft > nblock) {
      nleft -= nblock;

      while (nblock--) {
        t_sample f = *in1++;
        if (PD_BIGORSMALL(f)) f = 0.0f; // exponent outside (-64, 64)
        currramp += incr;
        prevramp -= incr;
        *wp = f;

        *out++ = (t_sample)(*prevrp * prevramp + *currrp * currramp);

        if (prevrp++ == ep) prevrp = buf;
        if (currrp++ == ep) currrp = buf;
        if (wp++ == ep) wp = buf;
      }
    } else {
      while (nleft) {
        nleft--;
        nblock--;
        t_sample f = *in1++;
        if (PD_BIGORSMALL(f)) f = 0.0f;

        *wp = f;
        *out++ = *currrp;

        if (currrp++ == ep) currrp = buf;
        if (wp++ == ep) wp = buf;
      }
    } // end else
    x->x_rampvals[0] = currramp;
    x->x_rampvals[1] = prevramp;
    x->x_remain = nleft;
  } else if (x->x_delsize) {
    t_sample *rp = wp - x->x_delsize;
    if (rp < buf) rp += maxsize + 1;

    while (nblock--) {
      t_sample f = *in1++;
      if (PD_BIGORSMALL(f)) f = 0.0f;

      *out++ = *rp;
      *wp = f;
      if (rp++ == ep) rp = buf;
      if (wp++ == ep) wp = buf;
    }

  } else {
    while (nblock--) {
      t_sample f = *in1++;
      if (PD_BIGORSMALL(f)) f = 0.0f;

      *wp = f;
      *out++ = *wp;
      if (wp++ == ep) wp = buf;
    }
  }
  x->x_whead = wp;

  return (w+6);
}

static void delay_dsp(t_delay *x, t_signal **sp)
{
  x->x_sr = sp[0]->s_sr;
  x->x_remain = 0;
  delay_clear(x);
  delay_bounds(x);
  dsp_add(delay_perform, 5, x, sp[0]->s_length, sp[0]->s_vec, sp[1]->s_vec, sp[2]->s_vec);
}

static void *delay_new(t_floatarg f1, t_floatarg f2)
{
  t_delay *x = (t_delay *)pd_new(delay_class);
  int maxsize = (f1 > 0 ? (int)f1 :DELAY_DEFMAXSIZE);
  x->x_maxsofar = DELAY_DEFMAXSIZE;
  x->x_maxsize = x->x_maxsofar;
  x->x_whead = x->x_bufini;
  x->x_buf = x->x_whead;
  delay_maxsize(x, maxsize);
  x->x_delsize = (f2 > 0 ? (int)f2 : 0);

  if (x->x_delsize > maxsize) x->x_delsize = maxsize;

  inlet_new(&x->x_obj, &x->x_obj.ob_pd, &s_signal, &s_signal);
  outlet_new(&x->x_obj, &s_signal);

  return (void *)x;
}

static void delay_free(t_delay *x)
{
  if (x->x_buf != NULL) {
    freebytes(x->x_buf, x->x_maxsofar * sizeof(t_sample));
    x->x_buf = NULL;
  }
}

void delay_tilde_setup(void)
{
  delay_class = class_new(gensym("delay~"),
                          (t_newmethod)delay_new,
                          (t_method)delay_free,
                          sizeof(t_delay),
                          CLASS_DEFAULT,
                          A_DEFFLOAT, A_DEFFLOAT, 0);
  class_addmethod(delay_class, (t_method)delay_dsp, gensym("dsp"), A_CANT, 0);
  class_addmethod(delay_class, (t_method)delay_clear, gensym("clear"), 0);
  class_addmethod(delay_class, (t_method)delay_ramp, gensym("ramp"), A_FLOAT, 0);
  class_addmethod(delay_class, (t_method)delay_maxsize, gensym("maxsize"), A_DEFFLOAT, 0);
  CLASS_MAINSIGNALIN(delay_class, t_delay, x_delsize); // guessing here
}

