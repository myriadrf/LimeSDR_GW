quartus_cpf -c test/gen_pof_file_dual_boot.cof
quartus_cpf -c -q 25MHz -g 3.3 -n p LimeSDR-Mini_lms7_trx_HW_1.2.pof output.svf

