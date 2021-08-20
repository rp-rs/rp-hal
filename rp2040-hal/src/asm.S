  .section .text.multicore_trampoline
  .global multicore_trampoline
  .thumb_func
multicore_trampoline:
  pop {r0, r1, pc}
