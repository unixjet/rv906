/*Copyright 2020-2021 T-Head Semiconductor Co., Ltd.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
int get_vtimer()
{
  volatile unsigned int   LoadCount;
  asm ("csrr %[LoadCount], time\n"
      :[LoadCount]"=r"(LoadCount)
      :
      :
      );
  //LoadCount = *TIMER_ADDR;
  return LoadCount;
  //int *TIMER_ADDR;
  //TIMER_ADDR = 0xE0013000;
  //volatile unsigned int   LoadCount;
  //LoadCount = *TIMER_ADDR;
  //return LoadCount;
}

/* M8 T4b (design section 4.5 item 4, ledger L13 class): rv906 exit
 * protocol. The donor's sim_end() writes the magic 0xffff0000 to
 * 0x6000FFF8, which the donor tb sniffs on the writeback bus and
 * $finish-es on (tb.v:259-282). On rv906 that address is unmapped
 * (axi_err, invisible), so the same program point exits through
 * crt0_m8.s __exit (tohost = 1 = PASS; the C++ harness polls the
 * `tohost` symbol). __exit never returns (spins after the store).
 * This is an rv906-glue-only delta; both sides still end at the same
 * point (sim_end is the last call of the VCUNT_SIM block).
 */
extern void __exit(void);

void sim_end()
{
  __exit();
}
