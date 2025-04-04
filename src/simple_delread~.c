#include "simple_del_shared.h"
#include <m_pd.h>

extern int ugen_getsortno(void);

static t_class *simple_delread_class = NULL;

typedef struct _simple_delread {
  t_object x_obj;
  t_symbol *x_sym;
  t_float x_deltime; /* delay in msecs */
  int x_delsamps; /* delay in samples */
  t_float x_sr; /* samples per msec */
  t_float x_n; /* vector size */
  int x_zerodel; /* 0 or vecsize depending on read/write order */
} t_simple_delread;

static void simple_delread_float(t_simple_delread *x, t_float f);

static void *simple_delread_new(t_symbol *s, t_floatarg f)
{
  t_simple_delread *x = (t_simple_delread *)pd_new(simple_delread_class);
  x->x_sym = s;
  x->x_sr = 1;
  x->x_n = 1;
  x->x_zerodel = 0;
  simple_delread_float(x, f);
  outlet_new(&x->x_obj, &s_signal);
  return (void *)x;
}

static void simple_delread_float(t_simple_delread *x, t_float f)
{
  // the delread~ object needs to find the corresponding delwrite~ object (done
  // through symbol lookup)
  t_simple_delwrite *delwriter = (t_simple_delwrite *)simple_delwrite_findbyname(x->x_sym);
  x->x_deltime = f;
  if (delwriter) {
    x->x_delsamps = (int)(0.5 + x->x_sr * x->x_deltime)
      + x->x_n - x->x_zerodel;
    if (x->x_delsamps < x->x_n) {
      x->x_delsamps = x->x_n;
    } else if (x->x_delsamps > delwriter->x_cspace.c_n) {
      x->x_delsamps = delwriter->x_cspace.c_n;
    }
  }
}

static t_int *simple_delread_perform(t_int *w)
{
  t_sample *out = (t_sample *)(w[1]); // output signal vector
  t_simple_delwritectl *c = (t_simple_delwritectl *)(w[2]); // delay buffer
  // control
  int delsamps = *(int *)(w[3]); // delay time in samples
  int n = (int)(w[4]); // block size

  // calculate read position by subtracting delay from current write position
  int phase = c->c_phase - delsamps;
  int nsamps = c->c_n; // size of delay buffer


  t_sample *vp = c->c_vec; // start of the buffer
  t_sample *bp; // current position, assigned below
  t_sample *ep = vp + (c->c_n + XTRASAMPS); // end of the buffer, including
  // extra samples

  // handle negative phase (wrap arount)
  if (phase < 0) {
    phase += nsamps;
  }

  bp = vp + phase; // current position

  while (n--) {
    *out++ = *bp++; // read from the buffer

    // wrap around if the end of the buffer is reached
    if (bp == ep) {
      bp -= nsamps;
    }
  }
  return (w+5);
}

static void simple_delread_dsp(t_simple_delread *x, t_signal **sp)
{
  t_simple_delwrite *delwriter = (t_simple_delwrite *)simple_delwrite_findbyname(x->x_sym);
  x->x_sr = sp[0]->s_sr * 0.001;
  x->x_n = sp[0]->s_length;
  if (delwriter) {
    // ensures that all delread~ and delwrite~ objects in a chain have
    // compatible vector sizes and sample rates
    simple_delwrite_check(delwriter, sp[0]->s_n, sp[0]->s_sr);

    // x_zerodel handles the case where a delread~ object comes after its
    // corresponding delwrite in the DSP chain. allows delay to work even when
    // reading from a buffer that's being written to in the same DSP cycle.
    x->x_zerodel = (delwriter->x_sortno == ugen_getsortno() ?
                    0 : delwriter->x_vecsize);
    simple_delread_float(x, x->x_deltime);
    dsp_add(simple_delread_perform, 4,
            sp[0]->s_vec, &delwriter->x_cspace, &x->x_delsamps, (t_int)sp[0]->s_length);

    if (delwriter->x_cspace.c_n > 0 && sp[0]->s_n > delwriter->x_cspace.c_n) {
      pd_error(x, "simple_delread~ %s: blocksize larger than simple_delwrite~ buffer", x->x_sym->s_name);
    }
  } else if (*x->x_sym->s_name) {
    pd_error(x, "simple_delread~ %s: no such simple_delwrite~", x->x_sym->s_name);
  }
}

void simple_delread_tilde_setup(void)
{
  simple_delread_class = class_new(gensym("simple_delread~"),
                                   (t_newmethod)simple_delread_new,
                                   0,
                                   sizeof(t_simple_delread),
                                   0,
                                   A_DEFSYM, A_DEFFLOAT, 0);
  class_addmethod(simple_delread_class, (t_method)simple_delread_dsp, gensym("dsp"), A_CANT, 0);
  class_addfloat(simple_delread_class, (t_method)simple_delread_float);
  class_sethelpsymbol(simple_delread_class, gensym("delay-tilde-objects"));
}
