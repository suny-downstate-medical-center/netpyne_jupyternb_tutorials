/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__cal
#define _nrn_initial _nrn_initial__cal
#define nrn_cur _nrn_cur__cal
#define _nrn_current _nrn_current__cal
#define nrn_jacob _nrn_jacob__cal
#define nrn_state _nrn_state__cal
#define _net_receive _net_receive__cal 
#define _f_settables _f_settables__cal 
#define settables settables__cal 
#define states states__cal 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gbar _p[0]
#define ica _p[1]
#define m _p[2]
#define alpha _p[3]
#define beta _p[4]
#define Dm _p[5]
#define v _p[6]
#define _g _p[7]
#define _ion_ica	*_ppvar[0]._pval
#define _ion_dicadv	*_ppvar[1]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_settables(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_cal", _hoc_setdata,
 "settables_cal", _hoc_settables,
 0, 0
};
 
static void _check_settables(double*, Datum*, Datum*, _NrnThread*); 
static void _check_table_thread(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, int _type) {
   _check_settables(_p, _ppvar, _thread, _nt);
 }
 /* declare global and static user variables */
#define usetable usetable_cal
 double usetable = 1;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "usetable_cal", 0, 1,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "gbar_cal", "mho/cm2",
 "ica_cal", "mA/cm2",
 0,0
};
 static double delta_t = 1;
 static double m0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "usetable_cal", &usetable_cal,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[2]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"cal",
 "gbar_cal",
 0,
 "ica_cal",
 0,
 "m_cal",
 0,
 0};
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 8, _prop);
 	/*initialize range parameters*/
 	gbar = 0;
 	_prop->param = _p;
 	_prop->param_size = 8;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 3, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 	_ppvar[0]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[1]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _cal_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
     _nrn_thread_table_reg(_mechtype, _check_table_thread);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 8, 3);
  hoc_register_dparam_semantics(_mechtype, 0, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 cal /Users/salvadord/Documents/ISB/Models/netpyne_repo/netpyne/tutorials/netpyne-course-2021/x86_64/cal.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double *_t_alpha;
 static double *_t_beta;
static int _reset;
static char *modelname = "Calcium high-threshold L type current for RD Traub, J Neurophysiol 89:909-921, 2003";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int _f_settables(_threadargsprotocomma_ double);
static int settables(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static void _n_settables(_threadargsprotocomma_ double _lv);
 static int _slist1[1], _dlist1[1];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   settables ( _threadargscomma_ v ) ;
   Dm = alpha * ( 1.0 - m ) - beta * m ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 settables ( _threadargscomma_ v ) ;
 Dm = Dm  / (1. - dt*( ( alpha )*( ( ( - 1.0 ) ) ) - ( beta )*( 1.0 ) )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   settables ( _threadargscomma_ v ) ;
    m = m + (1. - exp(dt*(( alpha )*( ( ( - 1.0 ) ) ) - ( beta )*( 1.0 ))))*(- ( ( alpha )*( ( 1.0 ) ) ) / ( ( alpha )*( ( ( - 1.0 ) ) ) - ( beta )*( 1.0 ) ) - m) ;
   }
  return 0;
}
 static double _mfac_settables, _tmin_settables;
  static void _check_settables(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  static int _maktable=1; int _i, _j, _ix = 0;
  double _xi, _tmax;
  if (!usetable) {return;}
  if (_maktable) { double _x, _dx; _maktable=0;
   _tmin_settables =  - 120.0 ;
   _tmax =  40.0 ;
   _dx = (_tmax - _tmin_settables)/641.; _mfac_settables = 1./_dx;
   for (_i=0, _x=_tmin_settables; _i < 642; _x += _dx, _i++) {
    _f_settables(_p, _ppvar, _thread, _nt, _x);
    _t_alpha[_i] = alpha;
    _t_beta[_i] = beta;
   }
  }
 }

 static int settables(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _lv) { 
#if 0
_check_settables(_p, _ppvar, _thread, _nt);
#endif
 _n_settables(_p, _ppvar, _thread, _nt, _lv);
 return 0;
 }

 static void _n_settables(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _lv){ int _i, _j;
 double _xi, _theta;
 if (!usetable) {
 _f_settables(_p, _ppvar, _thread, _nt, _lv); return; 
}
 _xi = _mfac_settables * (_lv - _tmin_settables);
 if (isnan(_xi)) {
  alpha = _xi;
  beta = _xi;
  return;
 }
 if (_xi <= 0.) {
 alpha = _t_alpha[0];
 beta = _t_beta[0];
 return; }
 if (_xi >= 641.) {
 alpha = _t_alpha[641];
 beta = _t_beta[641];
 return; }
 _i = (int) _xi;
 _theta = _xi - (double)_i;
 alpha = _t_alpha[_i] + _theta*(_t_alpha[_i+1] - _t_alpha[_i]);
 beta = _t_beta[_i] + _theta*(_t_beta[_i+1] - _t_beta[_i]);
 }

 
static int  _f_settables ( _threadargsprotocomma_ double _lv ) {
   double _ltmp ;
 alpha = 1.6 / ( 1.0 + exp ( - 0.072 * ( _lv - 5.0 ) ) ) ;
   _ltmp = _lv + 8.9 ;
   if ( fabs ( _ltmp ) < 1e-6 ) {
     beta = 0.1 * exp ( - _ltmp / 5.0 ) ;
     }
   else {
     beta = 0.02 * _ltmp / ( exp ( _ltmp / 5.0 ) - 1.0 ) ;
     }
    return 0; }
 
static void _hoc_settables(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 
#if 1
 _check_settables(_p, _ppvar, _thread, _nt);
#endif
 _r = 1.;
 settables ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 1;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
  }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 1; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 0, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 1, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  m = m0;
 {
   settables ( _threadargscomma_ v ) ;
   m = alpha / ( alpha + beta ) ;
   m = 0.0 ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];

#if 0
 _check_settables(_p, _ppvar, _thread, _nt);
#endif
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
 }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   ica = gbar * m * m * ( v - 125.0 ) ;
   }
 _current += ica;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dica;
  _dica = ica;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dicadv += (_dica - ica)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ica += ica ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   states(_p, _ppvar, _thread, _nt);
  } }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(m) - _p;  _dlist1[0] = &(Dm) - _p;
   _t_alpha = makevector(642*sizeof(double));
   _t_beta = makevector(642*sizeof(double));
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/Users/salvadord/Documents/ISB/Models/netpyne_repo/netpyne/tutorials/netpyne-course-2021/netpyne-course-2021/cal.mod";
static const char* nmodl_file_text = 
  "TITLE Calcium high-threshold L type current for RD Traub, J Neurophysiol 89:909-921, 2003\n"
  "\n"
  "COMMENT\n"
  "\n"
  "	Implemented by Maciej Lazarewicz 2003 (mlazarew@seas.upenn.edu)\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "INDEPENDENT { t FROM 0 TO 1 WITH 1 (ms) }\n"
  "\n"
  "UNITS { \n"
  "	(mV) = (millivolt) \n"
  "	(mA) = (milliamp) \n"
  "}\n"
  " \n"
  "NEURON { \n"
  "	SUFFIX cal\n"
  "	USEION ca WRITE ica\n"
  "	RANGE  gbar, ica\n"
  "}\n"
  "\n"
  "PARAMETER { \n"
  "	gbar = 0.0 	(mho/cm2)\n"
  "	v  		(mV)  \n"
  "}\n"
  " \n"
  "ASSIGNED { \n"
  "	ica 		(mA/cm2) \n"
  "	alpha beta	(/ms)\n"
  "}\n"
  " \n"
  "STATE {\n"
  "	m\n"
  "}\n"
  "\n"
  "BREAKPOINT { \n"
  "	SOLVE states METHOD cnexp\n"
  "	ica = gbar * m * m * ( v - 125 ) \n"
  "}\n"
  " \n"
  "INITIAL { \n"
  "	settables(v) \n"
  "	m = alpha / ( alpha + beta )\n"
  "	m = 0\n"
  "}\n"
  " \n"
  "DERIVATIVE states { \n"
  "	settables(v) \n"
  "	m' = alpha * ( 1 - m ) - beta * m \n"
  "}\n"
  "\n"
  "UNITSOFF \n"
  "\n"
  "PROCEDURE settables(v) { LOCAL tmp\n"
  "	TABLE alpha, beta FROM -120 TO 40 WITH 641\n"
  "\n"
  "	alpha = 1.6 / ( 1 + exp( - 0.072 * ( v - 5 ) ) )\n"
  "	tmp = v + 8.9\n"
  "	if ( fabs( tmp ) < 1e-6 ) {\n"
  "		beta  = 0.1 * exp( - tmp / 5 ) \n"
  "	}else{\n"
  "		beta  = 0.02 * tmp / ( exp( tmp / 5 ) - 1 )\n"
  "	}\n"
  "}\n"
  "\n"
  "UNITSON\n"
  ;
#endif
