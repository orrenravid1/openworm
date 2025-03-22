#include <stdio.h>
#include "hocdec.h"
extern int nrnmpi_myid;
extern int nrn_nobanner_;
#if defined(__cplusplus)
extern "C" {
#endif

extern void _CaPool_reg(void);
extern void _Leak_reg(void);
extern void _ca_boyle_reg(void);
extern void _k_fast_reg(void);
extern void _k_slow_reg(void);
extern void _neuron_to_neuron_elec_syn_reg(void);
extern void _neuron_to_neuron_exc_syn_reg(void);
extern void _neuron_to_neuron_inh_syn_reg(void);
extern void _offset_current_reg(void);

void modl_reg() {
  if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
    fprintf(stderr, "Additional mechanisms from files\n");
    fprintf(stderr, " \"CaPool.mod\"");
    fprintf(stderr, " \"Leak.mod\"");
    fprintf(stderr, " \"ca_boyle.mod\"");
    fprintf(stderr, " \"k_fast.mod\"");
    fprintf(stderr, " \"k_slow.mod\"");
    fprintf(stderr, " \"neuron_to_neuron_elec_syn.mod\"");
    fprintf(stderr, " \"neuron_to_neuron_exc_syn.mod\"");
    fprintf(stderr, " \"neuron_to_neuron_inh_syn.mod\"");
    fprintf(stderr, " \"offset_current.mod\"");
    fprintf(stderr, "\n");
  }
  _CaPool_reg();
  _Leak_reg();
  _ca_boyle_reg();
  _k_fast_reg();
  _k_slow_reg();
  _neuron_to_neuron_elec_syn_reg();
  _neuron_to_neuron_exc_syn_reg();
  _neuron_to_neuron_inh_syn_reg();
  _offset_current_reg();
}

#if defined(__cplusplus)
}
#endif
