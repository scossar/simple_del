#include "simple_del_shared.h"
#include <m_pd.h>
#include <string.h>

extern int ugen_getsortno(void);

/* Copied as an exercise from pure_data/src/d_delay.c */

t_class *simple_delwrite_class;

/* a wrapper around pd_findbyclass. Solves the problem of not being able to find
 * simple_delwrite_class in simple_delread~.c (maybe there's another way) */
t_simple_delwrite *simple_delwrite_findbyname(t_symbol *s)
{
  return (t_simple_delwrite *)pd_findbyclass(s, simple_delwrite_class);
}

/* handles buffer allocation and resizing */
void simple_delwrite_update(t_simple_delwrite *x)
{
  // calculates buffer size in samples based on delay time
  int nsamps = x->x_deltime * x->x_sr * (t_float)(0.001f);
  if (nsamps < 1) nsamps = 1;

  // round up to a multiple of SAMPBLK (4)
  nsamps += ((- nsamps) & (SAMPBLK - 1));

  // add the vector size to ensure the buffer has enough space (set in delwrite_check from its vecsize arg
  // (sp[0]->s_length))
  nsamps += x->x_vecsize;

  // resize the buffer if needed
  if (x->x_cspace.c_n != nsamps) {
    x->x_cspace.c_vec = (t_sample *)resizebytes(x->x_cspace.c_vec,
                         (x->x_cspace.c_n + XTRASAMPS) * sizeof(t_sample),
                         (nsamps + XTRASAMPS) * sizeof(t_sample));
    x->x_cspace.c_n = nsamps;
    x->x_cspace.c_phase = XTRASAMPS;
    #if 0
      post("delay line resized to %d samples", nsamps);
    #endif
  }
}

static void simple_delwrite_clear(t_simple_delwrite *x)
{
  if (x->x_cspace.c_n > 0) {
    memset(x->x_cspace.c_vec, 0, sizeof(t_sample) * (x->x_cspace.c_n + XTRASAMPS));
  }
}

// ensures that delread and delwrite objects in a chain have compatible vector
// sizes and sample rates
void simple_delwrite_check(t_simple_delwrite *x, int vecsize, t_float sr)
{
  if (x->x_rsortno != ugen_getsortno()) {
    x->x_vecsize = vecsize;
    x->x_sr = sr;
    x->x_rsortno = ugen_getsortno();
  }
#if 1
  else
  {
    if (vecsize > x->x_vecsize) {
      x->x_vecsize = vecsize;
    }
    if (sr > x->x_sr) {
      x->x_sr = sr;
    }
  }

#else
  else if (vecsize != x->x_vecsize) {
    pd_error(x, "delread/delwrite/vd vector size mismatch");
  }
#endif
}

static void *simple_delwrite_new(t_symbol *s, t_floatarg msec)
{
  t_simple_delwrite *x = (t_simple_delwrite *)pd_new(simple_delwrite_class);
  if (!*s->s_name) s = gensym("simple_delwrite~");
  pd_bind(&x->x_obj.ob_pd, s);
  x->x_sym = s;
  x->x_deltime = msec;
  x->x_cspace.c_n = 0;
  x->x_cspace.c_vec = getbytes(XTRASAMPS * sizeof(t_sample));
  x->x_sortno = 0;
  x->x_vecsize = 0;
  x->x_sr = 0;
  x->x_f = 0;
  return (void *)x;
}

static t_int *simple_delwrite_perform(t_int *w)
{
  t_sample *in = (t_sample *)(w[1]); // input signal vector
  t_simple_delwritectl *c = (t_simple_delwritectl *)(w[2]); // delay buffer
  // control
  int n = (int)(w[3]); // block size
  int phase = c->c_phase; // current write position
  int nsamps = c->c_n; // size of delay buffer

  // various pointers into the buffer (current write position and buffer
  // boundaries)
  t_sample *vp = c->c_vec; // (pointer to) the start of the buffer
  t_sample *bp = vp + phase; // current write position
  t_sample *ep = vp + (c->c_n + XTRASAMPS); // end of the buffer (including
  // extra samples)
  phase += n; // advance phase by block size

  while (n--) {
    t_sample f = *in++;
    if (PD_BIGORSMALL(f)) { /* exponent outside (-64,64) */
      f = 0;
    }
    *bp++ = f; // current write position = f, then advance pointer

    // if the end of the buffer is reached (bp == ep) wrap around and copy the
    // last few samples to the start (the start being vp[0]). These are the
    // XTRASAMPS (4)
    if (bp == ep) {
      vp[0] = ep[-4];
      vp[1] = ep[-3];
      vp[2] = ep[-2];
      vp[3] = ep[-1];
      bp = vp + XTRASAMPS;
      phase -= nsamps;
    }
  }
  c->c_phase = phase;
  return (w+4);
}

static void simple_delwrite_dsp(t_simple_delwrite *x, t_signal **sp)
{
  dsp_add(simple_delwrite_perform, 3, sp[0]->s_vec, &x->x_cspace , (t_int)sp[0]->s_length);
  x->x_sortno = ugen_getsortno();
  simple_delwrite_check(x, sp[0]->s_length, sp[0]->s_sr);
  simple_delwrite_update(x);
}

static void simple_delwrite_free(t_simple_delwrite *x)
{
  pd_unbind(&x->x_obj.ob_pd, x->x_sym);
  if (x->x_cspace.c_vec != NULL) {
    freebytes(x->x_cspace.c_vec,
              (x->x_cspace.c_n + XTRASAMPS) * sizeof(t_sample));
  }
}

void simple_delwrite_tilde_setup(void)
{
  simple_delwrite_class = class_new(gensym("simple_delwrite~"),
                                    (t_newmethod)simple_delwrite_new,
                                    (t_method)simple_delwrite_free,
                                    sizeof(t_simple_delwrite),
                                    CLASS_DEFAULT,
                                    A_DEFSYM, A_DEFFLOAT, 0);
  CLASS_MAINSIGNALIN(simple_delwrite_class, t_simple_delwrite, x_f);
  class_addmethod(simple_delwrite_class, (t_method)simple_delwrite_dsp,
                  gensym("dsp"), A_CANT, 0);
  class_addmethod(simple_delwrite_class, (t_method)simple_delwrite_clear, gensym("clear"), 0);
  /* important? I had the idea it was needed for pd_findbyclass to work, but not
   * so sure about that */
  class_sethelpsymbol(simple_delwrite_class, gensym("simple_delwrite~"));
}

