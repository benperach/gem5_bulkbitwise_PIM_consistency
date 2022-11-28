# gem5_bulkbitwise_PIM_consistency

This project extends the project in https://github.com/benperach/gem5_PIM_extension.git (refered to here as the previous project), which is based on the gem5 v19 simulator.
This project adds consistency and coherency support to the previous project. This support is only implemented for the ruby MESI_Two_Level coherency protocol.
For all other aspects of the simulator, please see the previous project.

This project was developed during the work on the paper https://arxiv.org/abs/2211.07542. Explanation about the design is avilable in the paper.
If you use this code, please cite the paper.

In this code we implement the four consistency models for Bulk-Bitwise PIM mentioned in the paper: Atomic, Store, Scope, and Scope-Relax. (Note that the Store model is refered to as the "Write" model in the code.)
The ruby MESI coherency protocol for the Atomic, Store, and Scope models use the same code (exsiting as the MESI_Two_Level_pim_atomic protocol). 
The difference between the three protocols is in the ruby sequencer, deciding on when to release PIM ops.
The Scope-Relax protocol has its own ruby code.

In addition, there is an implementation for a clflush instruction, used for the baseline evaluation in the paper (while using the MESI_Two_Level_pim protocol taken from the previous project).
The MESI_Two_Level_pim_uc coherence protocol address all read and write operations to a PIM page as uncacheable (also used in the evaluation of the paper).
