#include "m_pd.h"

#define DEL_XTRASAMPS 4
#define DEL_SAMPBLK 4

t_class *delay_class = NULL;

typedef struct _delay {
  t_object x_obj;

  int x_phase; // current delay buffer write position

  int x_vecsize; // size of delay buffer in samples
  int x_blocksize; // pd block size size (probably 64) (called c_n in Pd code)
  t_float x_sr; // system (current Pd) sample rate
  t_float x_deltime; // delay in msecs
  t_float x_delsamps; // delay in samples

  t_sample *x_del_vec; // pointer to the delay buffer (c_vec in Pd code)

  t_outlet *x_out;
} t_delay;


static void delay_float(t_delay *x, t_float f)
{
  x->x_deltime = f;
  x->x_delsamps = (int)(0.5 + x->x_sr * x->x_deltime) + x->x_blocksize;


  // could this happen?
  if (x->x_delsamps < x->x_blocksize) {
    x->x_delsamps = x->x_blocksize;
  } else if (x->x_delsamps > x->x_vecsize) {
    x->x_delsamps = x->x_vecsize;
  }
}

static void delay_check(t_delay *x, int blocksize, t_float sr)
{
  x->x_sr = sr;
  x->x_blocksize = blocksize;
}

static void delay_update(t_delay *x)
{
  int nsamps = x->x_deltime * x->x_sr * (t_float)(0.001f);
  if (nsamps < 1) nsamps = 1;

  // round up to a multiple of DEL_SAMPBLK (4)
  nsamps += ((- nsamps) & DEL_SAMPBLK - 1);

  nsamps += x->x_blocksize;

  if (x->x_vecsize != nsamps) {
    x->x_del_vec = (t_sample *)resizebytes(x->x_del_vec,
                                           (x->)
  }
}

static void *delay_new(t_floatarg msec)
{
  t_delay *x = (t_delay *)pd_new(delay_class);
  x->x_phase = 0; // initial position for writing to delay buffer

  // variables that require dsp function args to properly set?
  x->x_blocksize = 1; // obviously wrong at this point
  x->x_vecsize = 1;
  x->x_sr = 1;
  x->x_delsamps = 0;

  x->x_del_vec = NULL;

  // sets x->x_deltime and x->delsamps (to initial sane values?)
  delay_float(x, msec);

  x->x_out = outlet_new(&x->x_obj, &s_signal);
  return (void *)x;
}

static t_int *delay_perform(t_int *w)
{
  t_delay *x = (t_delay *)(w[1]);
  t_sample *in = (t_sample *)(w[2]);
  t_sample *out = (t_sample *)(w[3]);
  int n = (int)(w[4]);

  while (n--) {
    t_sample f = *in++;
    *out++ = f;
  }
  return (w+5);
}

static void delay_dsp(t_delay *x, t_signal **sp)
{
  post("in dsp setup method");
  post("vector/bin size %d", sp[0]->s_length); // returns 64
  post("sample rate: %f", sp[0]->s_sr); // returns 48000.000000
  dsp_add(delay_perform, 4, x, sp[0]->s_vec, sp[1]->s_vec, sp[0]->s_length);
  delay_check(x, sp[0]->s_length, sp[0]->s_sr);
  delay_float(x, x->x_deltime);
}

void delay_tilde_setup(void)
{
  delay_class = class_new(gensym("delay~"),
                          (t_newmethod)delay_new,
                          0,
                          sizeof(t_delay),
                          CLASS_DEFAULT,
                          A_DEFFLOAT, 0);

  class_addmethod(delay_class, (t_method)delay_dsp, gensym("dsp"), A_CANT, 0);
  class_addmethod(delay_class, (t_method)delay_float);
  // I'm unsure about the third arg, described as a "dummy" floating point variable out of the dataspace (in the docs). It seems that it can be any float in the dataspace?
  CLASS_MAINSIGNALIN(delay_class, t_delay, x_deltime);
}
