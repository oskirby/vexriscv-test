#include <stdlib.h>
#include "Vvextest_sim.h"
#include "verilated.h"
#include "verilated_vcd_c.h"

template <class MODULE> class TESTBENCH {
public:
	VerilatedVcdC *m_trace;
	unsigned long m_tickcount;
	MODULE *m_core;

	TESTBENCH(void) {
		m_core = new MODULE;
		m_tickcount = 0;
	}

	virtual ~TESTBENCH(void) {
		delete m_core;
		m_core = NULL;
	}

	virtual void opentrace(const char *vcdname) {
		if (!m_trace) {
			m_trace = new VerilatedVcdC;
			m_core->trace(m_trace, 99);
			m_trace->open(vcdname);
		}
	}

	virtual void close(void) {
		if (m_trace) {
			m_trace->close();
			m_trace = NULL;
		}
	}

	virtual void reset(void) {
		m_core->rst = 1;
		this->tick();
		m_core->rst = 0;
	}

	virtual void tick(void) {
		m_tickcount++;
		
		// Allow combinatorial logic to settle before ticking the clock.
		m_core->clk = 0;
		m_core->eval();
		if (m_trace) m_trace->dump(10*m_tickcount-2);

		// Simulate the rising edge.
		m_core->clk = 1;
		m_core->eval();
		if (m_trace) m_trace->dump(10*m_tickcount);

		// Simulate the falling edge.
		m_core->clk = 0;
		m_core->eval();
		if (m_trace) {
			m_trace->dump(10*m_tickcount+5);
			m_trace->flush();
		}
	}

	virtual bool done(void) {
		return Verilated::gotFinish();
	}
};

int main(int argc, char **argv) {
	// Initialize Verilators variables
	Verilated::commandArgs(argc, argv);
	TESTBENCH<Vvextest_sim> *tb = new TESTBENCH<Vvextest_sim>();

	Verilated::traceEverOn(true);
	tb->opentrace("vextest_sim.vcd");

	while (!tb->done()) {
		tb->tick();
	} exit(EXIT_SUCCESS);
}

