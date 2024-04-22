// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#include <fstream>
#include <getopt.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../powerEstimation/ModelImporter.h"
#include "global.h"
#include "opts.h"
#include "virtual_reg.h"

// Options without short name:
#define SCHED_ALONE_PROB 1003
#define SCHED_PART_PROB 1004
#define PRINT_SCHED_POP 1005
#define PRINT_MERGE_POP 1006
#define INCLUDE_DDG_WEIGHTS 1007
#define SCHED_TOURNAMENT 1008
#define MERGE_TOURNAMENT 1009
#define SCHED_TRIAL_ROUNDS 1010
#define INITIAL_ASSEMBLER_PATH 1011
#define PRINT_REG_POP 1012
#define NO_HEURISTIC_REG 1013
#define PORT_REG_ROUNDS 1014
#define PORT_REG_TOURNAMENT 1015
#define RA_ENABLE_SKIP 1016
#define RA_SKIP_OFFSET 1017
#define RA_SMALLER_PENALTY 1018
#define ENABLE_SCHED_SKIP 1019
#define SCHED_SKIP_OFFSET 1020
#define SCHED_RANDOM_RATIO 1021
#define SCHED_ELITE_COUNT 1022
#define START_SLM 1023
#define STOP_SLM 1024
#define ENABLE_LOG 1025
#define DISABLE_LOG 1026
#define NO_RECURSIVE_MERGE 1027
#define NO_VIRTUAL_AND_FIX 1028
#define MERGE_MUTATION_RATE 1029
#define MERGE_MUTATE_ADAPTIVE 1030
#define MERGE_MUTATE_SLOPE 1031
#define MERGE_CMB_RATIO 1032
#define MERGE_POPSIZE_DIV 1033
#define MERGE_TRIAL_ROUNDS 1034
#define RA_MUTATION_RATE 1035
#define RA_PRUNE_COUNT 1036
#define NO_MERGE_PROB 1037

#define REGISTER_POWER_OPTIMIZATION_FILE 1039
#define PRINT_TRANSITON_POWER_ESTIMATION 1040
#define PRINT_COMPILABLE_ASSEMBLER 1041
#define PRINT_ASM_LINE_FILE 1042
#define REGISTER_POWER_MIN_TARGET 1043

#define RA_REG_RANDOM 1044
#define RA_REG_COPY 1045
#define RA_REG_COMBINE 1046
#define RA_REG_MUTATE 1047
#define RA_REG_COMBINE_MUTATE 1048
#define RA_REG_TOURNAMENT_SIZE 1049
#define RA_REG_TOURNAMENT_ABORT 1050
#define RA_REG_HIERARCHY_DESCEND 1051
#define RA_REG_ROUNDS 1052
#define RA_REG_NO_IMPROVEMENT_ABORT 1053
#define RA_REG_POPULATION 1054
#define RA_REG_MAX_COMBINE_ROOTS 1055
#define RA_REG_MAX_GENERATIONS 1056
#define PRINT_TRANSITON_POWER_ESTIMATION_CSV 1057
#define RA_REG_MAX_MUTATION_FRAC 1058
#define RA_REG_GREEDY_REPAIR 1059
#define RA_REG_POPULATION_VERIFICATION_DUMP 1060
#define RA_REG_COMBINE_GREEDY 1061
#define RA_REG_MUATION_GREEDY 1062
#define RA_REG_POWER_EQUAL_THRESHOLD 1063
#define RA_REG_INSTRUCTION_MODEL 1064

#define SLM_WEIGHTS_FILE 1065
#define SCHEDULE_STATS_OUTPUT_FILE 1066
#define FIX_DOT_GRAPH 1067

bool CONSTANT::minPower;
float CONSTANT::powerEqualThreshold;

struct optionStruct {
  std::string name;
  std::string helpMessage;
  std::string enableMessage;
  std::string valueSetMesssage;

  std::string printHelp() const {
    return "\t   --" + name + "\n\t\t " + helpMessage + "\n";
  }
};

static std::map<int, optionStruct> optionsPowerRegisterAllocation = {
    {REGISTER_POWER_OPTIMIZATION_FILE,
     optionStruct{"registerPowerFile",
                  "A file containing the power characteristics of the "
                  "processor. It is a matlab model export.",
                  "Will be using a power characteristics file. ",
                  "Using %s as power characteristics file.\n"}},
    {RA_REG_INSTRUCTION_MODEL,
     optionStruct{
         "instructionModelFile",
         "A CSV file containing instruction Model information.",
         "Will be using instruction model with genetic power scheduling.",
         "Using %s as instruction Model file.\n"}},
    {REGISTER_POWER_MIN_TARGET,
     optionStruct{
         "minPowerOptimization",
         "Power Optimization Target of Genetic Register Allocation. 1 to min "
         "power (default), 0 to max power ",
         "Enabled Power Minimization for Genetic Register Allocation. ",
         "Setting Optimization Target of GA Register Allocation to : %s.\n"}},
    {PRINT_TRANSITON_POWER_ESTIMATION,
     optionStruct{"printEstimatedTransitionPower",
                  "Print Assembly with Energy Estimation. ",
                  "Print Estimated TransitionPower Power. ",
                  "Printing Energy Estimation to %s\n"}},
    {PRINT_TRANSITON_POWER_ESTIMATION_CSV,
     optionStruct{"printEstimatedTransitionPowerCSV",
                  "CSV Export: Print Estimated TransitionPower Power to each "
                  "Assembly line. ",
                  "Print Estimated TransitionPower Power. ",
                  "Printing Energy Estimation to %s\n"}},
    {RA_REG_MAX_MUTATION_FRAC,
     optionStruct{"raRegMaxMutationFrac",
                  "Fraction of Virtual Register to at most mutate.",
                  "Override Genetic RA max Mutation Fraction.",
                  "Set Genetic RA max Mutation Fraction to %f\n"}},
    {RA_REG_RANDOM,
     optionStruct{
         "raRegRandom",
         " Fraction of Population to be randomly produced in Energy Aware "
         "Genetic Register Allocation. ",
         "Will override Random Fraction of Energy Aware Register Allocation. ",
         "Using %f of the population for RANDOM Individuals in Energy Aware "
         "Genetic Register Allocation.\n"}},
    {RA_REG_COPY, optionStruct{"raRegCopy",
                               " Fraction of Population to be copied (Elitism) "
                               "in Energy Aware Genetic Register Allocation. ",
                               "Will override Elitism Fraction of Energy Aware "
                               "Genetic Register Allocation. ",
                               "Using %f of the population for ELITISM in "
                               "Energy Aware Genetic Register Allocation.\n"}},
    {RA_REG_COMBINE,
     optionStruct{"raRegCombine",
                  " Fraction of Population to be combined in Energy Aware "
                  "Genetic Register Allocation.",
                  "Will override combination Fraction of Energy Aware Genetic "
                  "Register Allocation. ",
                  "Using %f of the population for COMBINATION in Energy Aware "
                  "Genetic Register Allocation.\n"}},
    {RA_REG_COMBINE_MUTATE,
     optionStruct{"raRegCombineMutate",
                  " Fraction of combins to be mutated in Energy Aware Genetic "
                  "Register Allocation. ",
                  "Will override Mutation Fraction of Combines of Energy Aware "
                  "Genetic Register Allocation. ",
                  "Using %f of the population for MUTATION of COMBINATION in "
                  "Energy Aware Genetic Register Allocation.\n"}},
    {RA_REG_HIERARCHY_DESCEND,
     optionStruct{"raRegHierarchyDescend",
                  " Percent of Genes to Descend in combination in Energy Aware "
                  "Genetic Register Allocation. ",
                  "Will override Percent of Genes to Descend in combination  "
                  "of Energy Aware Genetic Register Allocation. ",
                  "Using %f Percent of Genes to Descend in combination  in "
                  "Energy Aware Genetic Register Allocation.\n"}},
    {RA_REG_MAX_COMBINE_ROOTS,
     optionStruct{
         "raRegMaxCombineRoots",
         " How many Combines to max. process ina COMBINATION. Minimum is 1.",
         "Will customize max combine roots per COMBINATION.",
         "Will use up to %d combines.\n"}},

    {RA_REG_MUTATE,
     optionStruct{"raRegMutate",
                  " Fraction of Population to be mutated in Energy Aware "
                  "Genetic Register Allocation. ",
                  "Will override mutation Fraction of Energy Aware Genetic "
                  "Register Allocation. ",
                  "Using %f of the population for MUTATION in Energy Aware "
                  "Genetic Register Allocation.\n"}},
    {RA_REG_TOURNAMENT_SIZE,
     optionStruct{
         "raRegTournamentSize",
         " Tournament Size in Energy Aware Genetic Register Allocation. ",
         "Will override Tournament Size of Energy Aware Genetic Register "
         "Allocation. ",
         "Using Tournament size of %d in Energy Aware Genetic Register "
         "Allocation.\n"}},
    {RA_REG_TOURNAMENT_ABORT,
     optionStruct{
         "raRegTournamentAbort",
         " Tournament Abort Repetitions after not finding a different "
         "individual in Energy Aware Genetic Register Allocation. ",
         "Will override Tournament Abort Repetitions after not finding a "
         "different individual of Energy Aware Genetic Register Allocation. ",
         "Using Tournament Abort Repetitions after not finding a different "
         "individual of %d in Energy Aware Genetic Register Allocation.\n"}},

    {RA_REG_ROUNDS,
     optionStruct{"raRegMinRounds",
                  " Minimum Generations before aborting Energy Aware Genetic "
                  "Register Allocation. ",
                  "Will override Minimum Generations before aborting of Energy "
                  "Aware Genetic Register Allocation. ",
                  "Using %d Minimum Generations before aborting in Energy "
                  "Aware Genetic Register Allocation.\n"}},

    {RA_REG_NO_IMPROVEMENT_ABORT,
     optionStruct{
         "raRegNoImprovementAbort",
         " How many Generations after not finding a better Individual before "
         "aborting Energy Aware Genetic Register Allocation. ",
         "Will override how many Generations after not finding a better "
         "Individual before aborting of Energy Aware Genetic Register "
         "Allocation. ",
         "Using %d Generations after not finding a better Individual before "
         "aborting in Energy Aware Genetic Register Allocation.\n"}},

    {RA_REG_POPULATION,
     optionStruct{"raRegPopulation",
                  " Optimization Level of the Energyware Register Allocation "
                  "Genetic Population. ",
                  "Will override how many individuals a Generation of Energy "
                  "Aware Genetic Register "
                  "Allocation has. ",
                  "Using Optimization Level for Population Size of %d in "
                  "Energy Aware Genetic Register Allocation.\n"}},
    {RA_REG_MAX_GENERATIONS,
     optionStruct{"raRegMaxGeneration", " Maximum number of Generations . ",
                  "Will limit max number of generations ",
                  "Max Number of Generations %d in "
                  "Energy Aware Genetic Register Allocation.\n"}},
    {RA_REG_GREEDY_REPAIR,
     optionStruct{
         "raRegGreedyRepair",
         " Toggle to greedy repairGeneGreedy instead of random while repairing "
         "combination conflicts.",
         "Overwritten Greedy Repair of invalid combinations.",
         "Greedy Repair is %s. \n"}},
    {RA_REG_COMBINE_GREEDY,
     optionStruct{"raRegGreedyCombine",
                  "1 to turn on Greedy combine of available genes.",
                  "Overwritten Greedy Combine of available Genes.",
                  "Greedy Combine is %s.\n"}},
    {RA_REG_MUATION_GREEDY,
     optionStruct{"raRegGreedyMutation", "1 to turn on Greedy Mutation.",
                  "Overwritten Greedy Mutation Parameter."}},
    {RA_REG_POPULATION_VERIFICATION_DUMP,
     optionStruct{"raRegPopulationDump",
                  " Dump first 10 Idividuals of each Generation compiblable to "
                  "run Verification Script on GA process. ",
                  "Enabled Verification Dumping of Individuals.",
                  "Will dump assembler files for verification in %s\n"}},
    {RA_REG_POWER_EQUAL_THRESHOLD,
     optionStruct{"raRegPowerEqualThreshold",
                  "Set power equal threshold. "
                  "In a % of equal register power, use instruction transitions "
                  "as sorting criterion (naive Instruction model).",
                  "Overwrite Power Threshold.",
                  "Overwrite Threshold with %f\n"}}

};

// static optionStruct MinPowerOptimization{
//    "minPowerOptimization",
//    "Power Optimization Target of Genetic Register Allocation. 1 to min power
//    (default), 0 to max power", "Enabled Power Minimization for Genetic
//    Register Allocation.", "Setting Optimization Target of GA Register
//    Allocation to %s\n"};

static optionStruct PrintCompilableAssembler{
    "compilableAssembler=",
    " Prints a file with compilable Assembler (not "
    "the pretty version)",
    "Will print compilable Assembler", " Will use %s"};

static optionStruct PrintAsmLineOption{
    "graphWeights=", "Write out dot graph with MO ASM line numbers",
    "Will print .Dot-Graph weight of MOs in the Dot format.",
    "Will save .dot File in %s.\n"};

static optionStruct optionSLMWeights{
    "smlweights=FILE",
    "Weight of ML Model for List Scheduling. SLM ID (starts with 1) next is "
    "MO-ID, Weight (higher is scheduled earlier), optional alone (int > 0).",
    "Will use ML based scheduling weights for List Scheduling.\n",
    "Will use this file for ML weights in List scheduling: %s\n"};

static optionStruct Schedule_StatsOption{
    "StatsFile", "Filename/path where to save scheduling stats",
    "Will log scheduling statistics in separate file.\n",
    "Write schedule stats to file: %s\n"};

void printHelpOpts(char *name) {
  printf("usage: \n%s [OPTIONS]\n", name);
  printf("\t Required Arguments:\n");
  printf("\t   -a | --in_asm=FILE\n\t\t The assembler file which which should "
         "be compiled\n");
  printf("\t   -c | --config=FILE\n\t\t The processor configuration as xml "
         "file\n");
  printf("\t Statistic and output:\n");
  printf("\t   -d | --graphOut=FILE\n\t\t Write dot graph of the ASM code into "
         "file and exit.\n");
  printf("\t   -p | --precompile=FILE\n\t\t Write out code after precompiling "
         "into file.\n");
  printf(PrintAsmLineOption.printHelp().c_str());
  printf(
      "\t   -v | --verbose\n\t\t NO LONGER USED! Use --enableLog instead!\n");
  printf(
      "\t   -b | --binaryOut=FILE\n\t\t the file to write the binary code to. "
      "if no file is specified. there will be no binary code produced.\n");
  printf("\t   -r | --out_asm=FILE\n\t\t the file to write the readable "
         "scheduled code to. if no file is specified, cout will be used.\n");
  printf(PrintCompilableAssembler.printHelp().c_str());
  printf("\t   -k | --non_alloc_asm=FILE\n\t\t readable scheduled code without "
         "allocated registers.\n");
  printf("\t Options:\n");
  printf("\t   -t | --threads=NUMBER\n\t\t set the number of threads. (default "
         "= 1)\n");
  printf("\t   -i | --ignore\n\t\t it will ignore most errors during run.\n");
  printf("\t   --startSLM=NUMBER\n\t\t Start compiling the program at the SLM "
         "with the given ID.\n");
  printf("\t   --stopSLM=NUMBER\n\t\t Stop compiling after SLM with the given "
         "ID.\n");
  printf("\t Optimizations:\n");
  printf("\t   -l | --reduceLong\n\t\t a substitution of IL with I commands is "
         "performed to improve performance\n");
  printf("\t   -m | --optimizeMV\n\t\t MV from and to virtual registers are "
         "removed from the code.\n");
  printf("\t   -h | --replaceHard\n\t\t Hardware registers are replaced with "
         "virtual registers to reduce register pressure.\n");
  printf("\t   -A | --optimizeAll\n\t\t All optimizations are switched on - "
         "short for -h -m -l\n");
  printf("\t Scheduling:\n");
  printf("\t   -o | --optimization=NUMBER\n\t\t define the optimization level. "
         "(default = 1)\n");
  printf("\t     0 = no optimization. code is parsed and written out.\n");
  printf("\t     1 = a list scheduling algorithm is used for scheduling\n");
  printf("\t     2 = a genetic algorithm with small number of populations and "
         "tries is used.\n");
  printf("\t     3 = the population is bigger and won't give up so easily\n");
  printf("\t     4..10 = the population is increased even more\n");
  printf("\t   -O | --schedPop=NUMBER\n\t\t Define the population size for "
         "genetic scheduling. Implies -o.\n");
  printf(
      "\t   -n | --schedRounds=NUMBER\n\t\t Give number of rounds in the "
      "genetic scheduling.\n\t\t Positive: Number of rounds without "
      "improvement before stopping,\n\t\t negative: total number of rounds.\n");
  printf("\t   --schedTrialRounds=NUMBER\n\t\t Number of rounds while no "
         "solution is found.\n");
  printf("\t   -S | --stats=FILE\n\t\t Generate scheduling statistics\n");
  printf("\t   --partProb=NUMBER\n\t\t Probability for an MO to split the SLM "
         "(range 0-1).\n");
  printf("\t   --aloneProb=NUMBER\n\t\t Probability for an MO to stand alone "
         "in a MI (range 0-1).\n");
  printf("\t   --printSchedPop\n\t\t Print the scheduling population after "
         "each round of the GA.\n");
  printf("\t   --schedTourn=NUMBER\n\t\t Tournament size for crossover in "
         "genetic scheduling.\n");
  printf("\t   --schedRandomRatio\n\t\t Ratio of randomly created individuals "
         "per generation during scheduling.\n");
  printf("\t X2 Merging:\n");
  printf("\t   -x | --X2\n\t\t 2 equal instructions are substituted with a X2 "
         "instruction where possible\n");
  printf("\t     0 = no X2 merging is used.\n");
  printf("\t     1 = a simple genetic algorithm for X2 merging without "
         "generating moves is used.\n");
  printf("\t     2 = a genetic algorithm for X2 merging with increasing "
         "optimization level is used.\n");
  printf("\t     3..10 = the population is increased even more\n");
  printf("\t   -X | --mergePop=NUMBER\n\t\t Define the population size for "
         "genetic merging. Implies -x.\n");
  printf(
      "\t   -N | --mergeRounds=NUMBER\n\t\t Give number of rounds in the "
      "genetic merging.\n\t\t Positive: Number of rounds without improvement "
      "before stopping,\n\t\t negative: total number of rounds.\n");
  printf(
      "\t   --mergePopDiv=NUMBER\n\t\t Divider for merging population size.\n");
  printf("\t   --mergeCmbRatio=NUMBER\n\t\t Percentage (0-100) of individuals "
         "generated from crossover.\n");
  printf("\t   --printMergePop\n\t\t Print the X2 merging population after "
         "each round of the GA.\n");
  printf("\t   --mergeTourn=NUMBER\n\t\t Tournament size for crossover in "
         "genetic merging.\n");
  printf("\t   --enableSchedSkip\n\t\t Enable skipping of scheduling, if "
         "merging did not gain enough.\n");
  printf("\t   --schedSkipOffset\n\t\t If minsize of merged SLM is bigger than "
         "smallest minsize so far (+ offset), instruction scheduling is "
         "skipped.\n");
  printf("\t   --noRecursiveMerge\n\t\t Disable recursive merging.\n");
  printf("\t   --mergeMutate=NUMBER\n\t\t Mutation rate during instruction "
         "merging.\n");
  printf("\t   --mergeMutateAdaptive\n\t\t Enable adaptive mutation rate "
         "during instruction merging.\n");
  printf("\t   --mergeMutateSlope=NUMBER\n\t\t Slope for adaptive mutation "
         "rate curve.\n");
  printf("\t   --noMergeProb=NUMBER\n\t\t Probability for genes in first "
         "round's individuals to perform no merge.\n");
  printf(optionSLMWeights.printHelp().c_str());
  printf(Schedule_StatsOption.printHelp().c_str());
  printf("\t Register Allocation:\n");
  printf("\t   --noHeuristicReg\n\t\t Disable heuristic register allocation. "
         "You should enable one of the other register allocation methods when "
         "using this switch.\n");
  printf("\t   -R | --geneticRegister\n\t\t WARNING Legacy. The GA is now in "
         "--geneticPortReg.\n");
  printf(
      "\t   -P | --geneticPortReg {NUMBER}\n\t\t A genetic algorithm for "
      "read/write "
      "port optimal register allocation. The value specifies the optimization "
      "level.\n");
  printf("\t   --portRegRounds\n\t\t Number of rounds for port optimal "
         "register allocation.\n");
  printf("\t   --portRegTournament\n\t\t Tournament size for port optimal "
         "register allocation.\n");
  printf("\t   --printRegPop\n\t\t Print the individuals of genetic register "
         "allocation.\n");
  printf("\t   --enableRASkip\n\t\t Enable skipping of register allocation, if "
         "scheduling result is bigger than smallest SLM found so far "
         "(+offset).\n");
  printf("\t   --RAskipOffset\n\t\t Offset on the size of SLM, for which RA is "
         "performed.\n");
  printf(
      "\t   --RAsmallerPenalty\n\t\t Penalty added to getFitness of SLMs for "
      "which register allocation was skipped due to --RAonlySmaller.\n");
  printf("\t   --noVirtualAndFix\n\t\t Disable merging of fix and virtual "
         "registers.\n");
  printf("\t   --RAPruneCount=NUMBER\n\t\t Maximal number of (sched) "
         "individuals for which a genetic RA is attempted.\n");
  printf("\t Logging:\n");
  printf("\t   -L | --logfile=FILE\n\t\t Write output to log file. If not "
         "given, use stdout\n");
  printf("\t   --enableLog=SECTION\n\t\t Enable log section with the given "
         "name\n");
  printf("\t   --disableLog=SECTION\n\t\t Disable log section with the given "
         "name\n");
  printf("\t       Log sections:\n\t         ");
  for (const auto logMask : LOG_MASKS) {
    printf(" %s ", logMask.first.c_str());
  }

  printf("\n");
  printf("\t Energy Aware Register Allocation:\n");
  for (const auto iter : optionsPowerRegisterAllocation) {
    printf(iter.second.printHelp().c_str());
  }
}

void opts(int argc, char **argv) {
  int c;

  static struct option long_options[] = {
      {"reduceLong", no_argument, nullptr, 'l'},
      {"verbose", no_argument, nullptr, 'v'},
      {"ignore-error", no_argument, nullptr, 'i'},
      {"geneticRegister", no_argument, nullptr, 'R'},
      {"optimizeMV", no_argument, nullptr, 'm'},
      {"replaceHard", no_argument, nullptr, 'h'},
      {"optimizeAll", no_argument, nullptr, 'A'},
      {"binaryOut", required_argument, nullptr, 'b'},
      {"graphOut", required_argument, nullptr, 'd'},
      {"precompile", required_argument, nullptr, 'p'},
      {"readableOut", required_argument, nullptr, 'r'},
      {"config-file", required_argument, nullptr, 'c'},
      {"asm-file", required_argument, nullptr, 'a'},
      {"threads", required_argument, nullptr, 't'},
      {"optimization", required_argument, nullptr, 'o'},
      {"X2", required_argument, nullptr, 'x'},
      {"schedPop", required_argument, nullptr, 'O'},
      {"mergePop", required_argument, nullptr, 'X'},
      {"schedRounds", required_argument, nullptr, 'n'},
      {"mergeRounds", required_argument, nullptr, 'N'},
      {"stats", required_argument, nullptr, 'S'},
      {"partProb", required_argument, nullptr, SCHED_PART_PROB},
      {"aloneProb", required_argument, nullptr, SCHED_ALONE_PROB},
      {"includeDDGWeights", required_argument, nullptr, INCLUDE_DDG_WEIGHTS},
      {"printSchedPop", no_argument, nullptr, PRINT_SCHED_POP},
      {"printMergePop", no_argument, nullptr, PRINT_MERGE_POP},
      {"schedTourn", required_argument, nullptr, SCHED_TOURNAMENT},
      {"mergeTourn", required_argument, nullptr, MERGE_TOURNAMENT},
      {"schedTrialRounds", required_argument, nullptr, SCHED_TRIAL_ROUNDS},
      {"initialAssembler", required_argument, nullptr, INITIAL_ASSEMBLER_PATH},
      {"noHeuristicReg", no_argument, nullptr, NO_HEURISTIC_REG},
      {"geneticPortReg", required_argument, nullptr, 'P'},
      {"portRegRounds", required_argument, nullptr, PORT_REG_ROUNDS},
      {"portRegTournament", required_argument, nullptr, PORT_REG_TOURNAMENT},
      {"printRegPop", no_argument, nullptr, PRINT_REG_POP},
      {"enableRASkip", no_argument, nullptr, RA_ENABLE_SKIP},
      {"RAskipOffset", required_argument, nullptr, RA_SKIP_OFFSET},
      {"RAsmallerPenalty", required_argument, nullptr, RA_SMALLER_PENALTY},
      {"enableSchedSkip", no_argument, nullptr, ENABLE_SCHED_SKIP},
      {"schedSkipOffset", required_argument, nullptr, SCHED_SKIP_OFFSET},
      {"schedRandomRatio", required_argument, nullptr, SCHED_RANDOM_RATIO},
      {"schedEliteCount", required_argument, nullptr, SCHED_ELITE_COUNT},
      {"logfile", required_argument, nullptr, 'L'},
      {"startSLM", required_argument, nullptr, START_SLM},
      {"stopSLM", required_argument, nullptr, STOP_SLM},
      {"enableLog", required_argument, nullptr, ENABLE_LOG},
      {"disableLog", required_argument, nullptr, DISABLE_LOG},
      {"noRecursiveMerge", no_argument, nullptr, NO_RECURSIVE_MERGE},
      {"noVirtualAndFix", no_argument, nullptr, NO_VIRTUAL_AND_FIX},
      {"non_alloc_asm", required_argument, nullptr, 'k'},
      {"mergeMutate", required_argument, nullptr, MERGE_MUTATION_RATE},
      {"mergeMutateAdaptive", no_argument, nullptr, MERGE_MUTATE_ADAPTIVE},
      {"mergeMutateSlope", required_argument, nullptr, MERGE_MUTATE_SLOPE},
      {"mergeCmbRatio", required_argument, nullptr, MERGE_CMB_RATIO},
      {"mergePopDiv", required_argument, nullptr, MERGE_POPSIZE_DIV},
      {"raMutationRate", required_argument, nullptr, RA_MUTATION_RATE},
      {"RAPruneCount", required_argument, nullptr, RA_PRUNE_COUNT},
      {"noMergeProb", required_argument, nullptr, NO_MERGE_PROB},

      {optionsPowerRegisterAllocation[REGISTER_POWER_OPTIMIZATION_FILE]
           .name.c_str(),
       required_argument, nullptr, REGISTER_POWER_OPTIMIZATION_FILE},
      {optionsPowerRegisterAllocation[REGISTER_POWER_MIN_TARGET].name.c_str(),
       required_argument, nullptr, REGISTER_POWER_MIN_TARGET},
      {optionsPowerRegisterAllocation[PRINT_TRANSITON_POWER_ESTIMATION]
           .name.c_str(),
       required_argument, nullptr, PRINT_TRANSITON_POWER_ESTIMATION},
      {optionsPowerRegisterAllocation[PRINT_TRANSITON_POWER_ESTIMATION_CSV]
           .name.c_str(),
       required_argument, nullptr, PRINT_TRANSITON_POWER_ESTIMATION_CSV},
      {optionsPowerRegisterAllocation[RA_REG_RANDOM].name.c_str(),
       required_argument, nullptr, RA_REG_RANDOM},
      {optionsPowerRegisterAllocation[RA_REG_COPY].name.c_str(),
       required_argument, nullptr, RA_REG_COPY},
      {optionsPowerRegisterAllocation[RA_REG_COMBINE].name.c_str(),
       required_argument, nullptr, RA_REG_COMBINE},
      {optionsPowerRegisterAllocation[RA_REG_COMBINE_MUTATE].name.c_str(),
       required_argument, nullptr, RA_REG_COMBINE_MUTATE},
      {optionsPowerRegisterAllocation[RA_REG_HIERARCHY_DESCEND].name.c_str(),
       required_argument, nullptr, RA_REG_HIERARCHY_DESCEND},
      {optionsPowerRegisterAllocation[RA_REG_MUTATE].name.c_str(),
       required_argument, nullptr, RA_REG_MUTATE},
      {optionsPowerRegisterAllocation[RA_REG_TOURNAMENT_SIZE].name.c_str(),
       required_argument, nullptr, RA_REG_TOURNAMENT_SIZE},
      {optionsPowerRegisterAllocation[RA_REG_TOURNAMENT_ABORT].name.c_str(),
       required_argument, nullptr, RA_REG_TOURNAMENT_ABORT},
      {optionsPowerRegisterAllocation[RA_REG_ROUNDS].name.c_str(),
       required_argument, nullptr, RA_REG_ROUNDS},
      {optionsPowerRegisterAllocation[RA_REG_NO_IMPROVEMENT_ABORT].name.c_str(),
       required_argument, nullptr, RA_REG_NO_IMPROVEMENT_ABORT},
      {optionsPowerRegisterAllocation[RA_REG_POPULATION].name.c_str(),
       required_argument, nullptr, RA_REG_POPULATION},
      {optionsPowerRegisterAllocation[RA_REG_MAX_COMBINE_ROOTS].name.c_str(),
       required_argument, nullptr, RA_REG_MAX_COMBINE_ROOTS},
      {optionsPowerRegisterAllocation[RA_REG_MAX_GENERATIONS].name.c_str(),
       required_argument, nullptr, RA_REG_MAX_GENERATIONS},
      {optionsPowerRegisterAllocation[RA_REG_MAX_MUTATION_FRAC].name.c_str(),
       required_argument, nullptr, RA_REG_MAX_MUTATION_FRAC},
      {optionsPowerRegisterAllocation[RA_REG_GREEDY_REPAIR].name.c_str(),
       required_argument, nullptr, RA_REG_GREEDY_REPAIR},
      {optionsPowerRegisterAllocation[RA_REG_POPULATION_VERIFICATION_DUMP]
           .name.c_str(),
       required_argument, nullptr, RA_REG_POPULATION_VERIFICATION_DUMP},
      {optionsPowerRegisterAllocation[RA_REG_COMBINE_GREEDY].name.c_str(),
       required_argument, nullptr, RA_REG_COMBINE_GREEDY},
      {optionsPowerRegisterAllocation[RA_REG_MUATION_GREEDY].name.c_str(),
       required_argument, nullptr, RA_REG_MUATION_GREEDY},
      {optionsPowerRegisterAllocation[RA_REG_POWER_EQUAL_THRESHOLD]
           .name.c_str(),
       required_argument, nullptr, RA_REG_POWER_EQUAL_THRESHOLD},
      {optionsPowerRegisterAllocation[RA_REG_INSTRUCTION_MODEL].name.c_str(),
       required_argument, nullptr, RA_REG_INSTRUCTION_MODEL},

      {PrintCompilableAssembler.name.c_str(), required_argument, nullptr,
       PRINT_COMPILABLE_ASSEMBLER},
      {PrintAsmLineOption.name.c_str(), required_argument, nullptr,
       PRINT_ASM_LINE_FILE},
      {optionSLMWeights.name.c_str(), required_argument, 0, SLM_WEIGHTS_FILE},
      {Schedule_StatsOption.name.c_str(), required_argument, 0,
       SCHEDULE_STATS_OUTPUT_FILE},
      {nullptr, 0, nullptr, 0}};

  /* getopt_long stores the option index here. */
  int option_index = 0;

  // cout << "OVERWRITING LOG" << endl;
  //  params.logMask |= LOG_M_RDG_DEBUG;
  //  params.logMask |= LOG_M_CHECK_EXEC;
  params.logMask |= LOG_M_REG_ENERGY;
  CONSTANT::minPower = false;
  CONSTANT::powerEqualThreshold = -1.0;

  while (true) {

    c = getopt_long(argc, argv,
                    "lviRmhAb:d:p:r:c:a:t:o:x:O:X:n:N:S:P:L:k:", long_options,
                    &option_index);

    /* Detect the end of the options. */
    if (c == -1)
      break;

    switch (c) {

    case 'a':
      LOG_OUTPUT(LOG_M_ALWAYS, "using ASM-File %s\n", optarg);
      params.asmfile = optarg;
      break;

    case 'c':
      LOG_OUTPUT(LOG_M_ALWAYS, "using Config-File %s\n", optarg);
      params.config = optarg;
      break;

    case 'b':
      LOG_OUTPUT(LOG_M_ALWAYS, "using file %s for binary output\n", optarg);
      params.binary = optarg;
      break;

    case 'd':
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "using file %s for dot output - WARNING: PRECOMPILE!!!!\n",
                 optarg);
      params.dot = optarg;
      break;

    case PRINT_ASM_LINE_FILE:
      LOG_OUTPUT(LOG_M_ALWAYS, PrintAsmLineOption.enableMessage.c_str());
      params.dotAsmLineFile = optarg;
      LOG_OUTPUT(LOG_M_ALWAYS, PrintAsmLineOption.valueSetMesssage.c_str(),
                 params.dotAsmLineFile.c_str());
      break;

    case 'p':
      LOG_OUTPUT(LOG_M_ALWAYS, "using file %s for precompiled output\n",
                 optarg);
      params.precompiled = optarg;
      break;

    case 'l':
      LOG_OUTPUT(LOG_M_ALWAYS, "Long immediate are replaced if possible\n");
      params.replace = true;
      break;

    case 'm':
      LOG_OUTPUT(LOG_M_ALWAYS, "MV operations are removed where possible\n");
      params.mvOptimizer = true;
      break;

    case 'h':
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Hardware registers are removed where possible\n");
      params.replaceHard = true;
      break;

    case 'A':
      LOG_OUTPUT(LOG_M_ALWAYS, "MV operations are removed where possible\n");
      params.mvOptimizer = true;
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Hardware registers are removed where possible\n");
      params.replaceHard = true;
      LOG_OUTPUT(LOG_M_ALWAYS, "Long immediate are replaced if possible\n");
      params.replace = true;
      break;

    case 'x':
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Will merge instructions into X2 where possible with "
                 "optimization level %s\n",
                 optarg);
      params.mergeLevel = atoi(optarg);
      break;

    case 'X':
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "set population size for genetic merging to %s\n", optarg);
      params.mergePopulation = atoi(optarg);
      params.mergeLevel = 2;
      break;

    case NO_HEURISTIC_REG:
      LOG_OUTPUT(LOG_M_ALWAYS, "Disabling heuristic register allocation\n");
      params.heuristicReg = false;
      break;

    case PRINT_TRANSITON_POWER_ESTIMATION:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[PRINT_TRANSITON_POWER_ESTIMATION]
              .enableMessage.c_str());
      params.printEstimatedTransitionPower = std::string(optarg);
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[PRINT_TRANSITON_POWER_ESTIMATION]
              .valueSetMesssage.c_str(),
          params.printEstimatedTransitionPower.c_str());
      break;

    case PRINT_TRANSITON_POWER_ESTIMATION_CSV:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[PRINT_TRANSITON_POWER_ESTIMATION_CSV]
              .enableMessage.c_str());
      params.printEstimatedTransitionPowerCSV = std::string(optarg);
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[PRINT_TRANSITON_POWER_ESTIMATION_CSV]
              .valueSetMesssage.c_str(),
          params.printEstimatedTransitionPowerCSV.c_str());
      break;

    case RA_REG_MAX_MUTATION_FRAC:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MAX_MUTATION_FRAC]
                     .enableMessage.c_str());
      params.raRegMaxMutationFraction = atof(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MAX_MUTATION_FRAC]
                     .valueSetMesssage.c_str());
      break;
    case PRINT_COMPILABLE_ASSEMBLER:
      LOG_OUTPUT(LOG_M_ALWAYS, PrintCompilableAssembler.enableMessage.c_str());
      params.compilableAssemblerFile = optarg;
      LOG_OUTPUT(LOG_M_ALWAYS,
                 PrintCompilableAssembler.valueSetMesssage.c_str(),
                 params.compilableAssemblerFile.c_str());
      break;

    case REGISTER_POWER_OPTIMIZATION_FILE:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[REGISTER_POWER_OPTIMIZATION_FILE]
              .enableMessage.c_str());
      params.powerOptimizationFile = optarg;
      try {
        ModelImporter::getInstance();
      } catch (...) {
        LOG_OUTPUT(LOG_M_ALWAYS,
                   "\n\n----------------------------\nPower Register Model "
                   "Input-File import went Wrong!\nDoes the file exist?");
        exit(1);
      }
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[REGISTER_POWER_OPTIMIZATION_FILE]
              .valueSetMesssage.c_str(),
          params.powerOptimizationFile.c_str());
      break;

    case SLM_WEIGHTS_FILE:
      LOG_OUTPUT(LOG_M_ALWAYS, optionSLMWeights.enableMessage.c_str());
      params.fileSlmWeights = optarg;
      params.optimization = 1;
      params.mergeLevel = 0;
      LOG_OUTPUT(LOG_M_ALWAYS, optionSLMWeights.valueSetMesssage.c_str(),
                 params.fileSlmWeights.c_str());
      break;

    case SCHEDULE_STATS_OUTPUT_FILE:
      LOG_OUTPUT(LOG_M_ALWAYS, Schedule_StatsOption.enableMessage.c_str());
      params.stats_file = optarg;

      LOG_OUTPUT(LOG_M_ALWAYS, Schedule_StatsOption.valueSetMesssage.c_str(),
                 params.stats_file.c_str());

      break;
    case RA_REG_INSTRUCTION_MODEL:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_INSTRUCTION_MODEL]
                     .enableMessage.c_str());
      params.instructionModelFile = optarg;

      {
        std::ifstream csvread;
        csvread.open(params.instructionModelFile);
        if (not csvread) {
          LOG_OUTPUT(
              LOG_M_ALWAYS,
              "\n\n----------------------------\nPower Instruction Model "
              "Input-File import went Wrong!\nDoes the file exist?");
          exit(1);
        }
        csvread.close();
      }
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_INSTRUCTION_MODEL]
                     .valueSetMesssage.c_str(),
                 params.instructionModelFile.c_str());
      break;

    case RA_REG_POPULATION_VERIFICATION_DUMP:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_POPULATION_VERIFICATION_DUMP]
              .enableMessage.c_str());
      params.raRegPopulationVerificationDir = optarg;
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_POPULATION_VERIFICATION_DUMP]
              .valueSetMesssage.c_str(),
          params.raRegPopulationVerificationDir.c_str());
      break;

    case REGISTER_POWER_MIN_TARGET: {

      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[REGISTER_POWER_MIN_TARGET]
                     .enableMessage.c_str());
      int minPower = atoi(optarg);
      std::string powerTarget = "Energy MINIMIZE";
      CONSTANT::minPower = true;
      if (minPower == 0) {
        powerTarget = "Energy MAXIMIZE";
        CONSTANT::minPower = false;
      }
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[REGISTER_POWER_MIN_TARGET]
                     .valueSetMesssage.c_str(),
                 powerTarget.c_str());
      break;
    }

    case RA_REG_MAX_GENERATIONS: {
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MAX_GENERATIONS]
                     .enableMessage.c_str());
      int maxGenerations = atoi(optarg);
      params.raRegMaxGeneration = maxGenerations;
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MAX_GENERATIONS]
                     .valueSetMesssage.c_str(),
                 params.raRegMaxGeneration);

      break;
    }

    case RA_REG_GREEDY_REPAIR: {

      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_GREEDY_REPAIR]
                     .enableMessage.c_str());
      int value = atoi(optarg);
      if (value > 0) {
        params.raRegRepairCombineGreedy = true;
      } else {
        params.raRegRepairCombineGreedy = false;
      }
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_GREEDY_REPAIR]
                     .valueSetMesssage.c_str(),
                 params.raRegRepairCombineGreedy ? "true" : "false");
      break;
    }

    case RA_REG_COMBINE_GREEDY: {

      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_COMBINE_GREEDY]
                     .enableMessage.c_str());
      int value = atoi(optarg);
      if (value > 0) {
        params.raRegCombineGreedy = true;
      } else {
        params.raRegCombineGreedy = false;
      }
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_COMBINE_GREEDY]
                     .valueSetMesssage.c_str(),
                 params.raRegCombineGreedy ? "true" : "false");
      break;
    }

    case RA_REG_MUATION_GREEDY: {

      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MUATION_GREEDY]
                     .enableMessage.c_str());
      int value = atoi(optarg);
      if (value > 0) {
        params.raRegMutateGreedy = true;
      } else {
        params.raRegMutateGreedy = false;
      }
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MUATION_GREEDY]
                     .valueSetMesssage.c_str(),
                 params.raRegMutateGreedy ? "true" : "false");
      break;
    }

    case 'P':
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Enabling port optimal register allocation with optimization "
                 "level %s.\n",
                 optarg);
      params.portReg = atoi(optarg);
      break;

    case PORT_REG_ROUNDS:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Setting number of rounds for port optimal register "
                 "allocation to %s.\n",
                 optarg);
      params.portRegRounds = atoi(optarg);
      break;

    case PORT_REG_TOURNAMENT:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Setting tournament size for port optimal register allocation "
                 "to %s.\n",
                 optarg);
      params.portRegTournament = atoi(optarg);
      break;

    case 't':
#ifdef _OPENMP
      LOG_OUTPUT(LOG_M_ALWAYS, "using %s threads\n", optarg);
      params.threads = atoi(optarg);

#else
      if (atoi(optarg) != 1) {
        std::cerr << "This program has been compiled without OpenMP support."
                  << std::endl;
        std::cerr << "There will always be only one thread!" << std::endl;
      }
#endif
      break;

    case 'r':
      LOG_OUTPUT(LOG_M_ALWAYS, "using file %s for readable output\n", optarg);
      params.readable = optarg;
      break;

    case 'k':
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          "using file %s for readable output without register allocation\n",
          optarg);
      params.nonAllocReadable = optarg;
      break;

    case 'v':
      //			params.debuglevel++;
      LOG_OUTPUT(LOG_M_ALWAYS, "WARNING: Debug level switch -v no longer used. "
                               "Use --enableLog instead!\n");
      break;

    case 'i':
      params.ignoreerror = true;
      LOG_OUTPUT(LOG_M_ALWAYS, "ignoring errors\n");
      break;

    case 'o':
      LOG_OUTPUT(LOG_M_ALWAYS, "set optimization level to %s\n", optarg);
      params.optimization = atoi(optarg);
      break;

    case 'O':
      LOG_OUTPUT(LOG_M_ALWAYS, "set scheduling population size to %s\n",
                 optarg);
      params.schedPopulation = atoi(optarg);
      params.optimization = 2;
      break;

    case 'n':
      LOG_OUTPUT(LOG_M_ALWAYS, "set scheduling rounds to %s\n", optarg);
      params.schedRounds = atoi(optarg);
      break;

    case 'N':
      LOG_OUTPUT(LOG_M_ALWAYS, "set merging rounds to %s\n", optarg);
      params.mergeRounds = atoi(optarg);
      break;

    case 'S':
      LOG_OUTPUT(LOG_M_ALWAYS, "activating scheduling statistics\n");
      params.sched_stats = optarg;
      break;

    case SCHED_PART_PROB:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "setting scheduling partition probability to %s\n", optarg);
      params.partProb = atof(optarg);
      break;

    case SCHED_ALONE_PROB:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "setting scheduling single MO probability to %s\n", optarg);
      params.aloneProb = atof(optarg);
      break;

    case PRINT_SCHED_POP:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "printing scheduling population after each round\n");
      params.printSchedPopulation = true;
      break;

    case PRINT_MERGE_POP:
      LOG_OUTPUT(LOG_M_ALWAYS, "printing merge population after each round\n");
      params.printMergePopulation = true;
      break;

    case INCLUDE_DDG_WEIGHTS:
      LOG_OUTPUT(LOG_M_ALWAYS, "setting include ddg weight: %s\n", optarg);
      params.includeDDGWeights = atoi(optarg);
      break;

    case SCHED_TOURNAMENT:
      LOG_OUTPUT(LOG_M_ALWAYS, "setting tournament size for scheduling to %s\n",
                 optarg);
      params.schedTournamentSize = atoi(optarg);
      break;

    case MERGE_TOURNAMENT:
      LOG_OUTPUT(LOG_M_ALWAYS, "setting tournament size for merging to %s\n",
                 optarg);
      params.mergeTournamentSize = atoi(optarg);
      break;

    case SCHED_TRIAL_ROUNDS:
      LOG_OUTPUT(LOG_M_ALWAYS, "setting scheduling trial rounds to %s\n",
                 optarg);
      params.schedTrialRounds = atoi(optarg);
      break;

    case INITIAL_ASSEMBLER_PATH:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "setting output path for initial assembler code to %s\n",
                 optarg);
      params.initialassembler = optarg;
      break;

    case PRINT_REG_POP:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "printing register allocation population after each round\n");
      params.printRegPopulation = true;
      break;

    case RA_ENABLE_SKIP:
      LOG_OUTPUT(LOG_M_ALWAYS, "Skipping of register allocation enabled\n");
      params.enableRASkip = true;
      break;

    case RA_SKIP_OFFSET:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting RAonlySmaller to %s\n", optarg);
      params.RASkipOffset = atoi(optarg);
      break;

    case RA_SMALLER_PENALTY:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Setting penalty for skipped register allocation to %s\n",
                 optarg);
      params.RAsmallerPenalty = atoi(optarg);
      break;

    case RA_MUTATION_RATE:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting RA mutation rate to %s\n", optarg);
      params.raMutationRate = atof(optarg);
      break;

    case ENABLE_SCHED_SKIP:
      LOG_OUTPUT(LOG_M_ALWAYS, "Skipping of instruction scheduling enabled\n");
      params.enableSchedSkip = true;
      break;

    case SCHED_SKIP_OFFSET:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting minSizeSchedOffset to %s\n", optarg);
      params.schedSkipOffset = atoi(optarg);
      break;

    case SCHED_RANDOM_RATIO:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting schedRandomRatio to %s\n", optarg);
      params.schedRandomRatio = atof(optarg);
      break;

    case SCHED_ELITE_COUNT:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting schedEliteCount to %s\n", optarg);
      params.schedEliteCount = atoi(optarg);
      break;

    case 'L':
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting output log file to %s\n", optarg);
      params.logFile = fopen(optarg, "w");
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting output log file to %s\n", optarg);
      break;

    case START_SLM:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting start SLM to %s\n", optarg);
      params.startSLM = atoi(optarg);
      break;

    case STOP_SLM:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting stop SLM to %s\n", optarg);
      params.stopSLM = atoi(optarg);
      break;

    case NO_RECURSIVE_MERGE:
      LOG_OUTPUT(LOG_M_ALWAYS, "Disabling recursive merging\n");
      params.recursiveMerge = false;
      break;

    case NO_VIRTUAL_AND_FIX:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Disabling merging of fix and virtual registers\n");
      params.mergeFixAndVirtual = false;
      break;

    case MERGE_MUTATION_RATE:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Setting mutation rate during instruction merging to %s\n",
                 optarg);
      params.mergeMutationRate = atof(optarg);
      break;

    case MERGE_MUTATE_ADAPTIVE:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          "Enabling adaptive mutation rate during instruction scheduling\n");
      params.mergeMutateAdaptively = true;
      break;

    case MERGE_MUTATE_SLOPE:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 "Setting merge adaptive mutation rate slope to %s\n", optarg);
      params.mergeMutateSlope = atof(optarg);
      break;

    case MERGE_CMB_RATIO:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting merge combine ratio to %s\n", optarg);
      params.mergeCmbRatio = atoi(optarg);
      break;

    case MERGE_POPSIZE_DIV:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting merge population size divider to %s\n",
                 optarg);
      params.mergePopsizeDiv = atoi(optarg);
      break;

    case MERGE_TRIAL_ROUNDS:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting merge trial rounds to %s\n", optarg);
      params.mergeTrialRounds = atoi(optarg);
      break;

    case NO_MERGE_PROB:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting noMergeProb to %s\n");
      params.noMergeProbability = atof(optarg);
      break;

    case ENABLE_LOG: {
      auto it = LOG_MASKS.find(optarg);
      if (it == LOG_MASKS.end()) {
        LOG_OUTPUT(LOG_M_ALWAYS, "Unknown log section '%s' ignored.\n", optarg);
        std::stringstream ss;
        ss << "Unknown log section '" << optarg << "' ignored.\n";
        throw std::runtime_error(ss.str());
        break;
      }
      int64_t mask = it->second;
      params.logMask |= mask;
      LOG_OUTPUT(LOG_M_ALWAYS, "Enabling log section '%s' -> %d\n", optarg,
                 params.logMask);
      break;
    }

    case DISABLE_LOG: {
      auto it = LOG_MASKS.find(optarg);
      if (it == LOG_MASKS.end()) {
        LOG_OUTPUT(LOG_M_ALWAYS, "Unknown log section '%s' ignored.\n", optarg);
        break;
      }
      int64_t mask = it->second;
      params.logMask &= ~mask;
      LOG_OUTPUT(LOG_M_ALWAYS, "Disabling log section '%s' -> %d\n", optarg,
                 params.logMask);
      break;
    }

    case RA_PRUNE_COUNT:
      LOG_OUTPUT(LOG_M_ALWAYS, "Setting RAPruneCount to %s\n", optarg);
      params.RAPruneCount = atoi(optarg);
      break;

    case RA_REG_RANDOM:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_RANDOM].enableMessage.c_str());
      params.raRegRandom = atof(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_RANDOM]
                     .valueSetMesssage.c_str(),
                 params.raRegRandom);
      break;
    case RA_REG_COPY:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_COPY].enableMessage.c_str());
      params.raRegCopy = atof(optarg);
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_COPY].valueSetMesssage.c_str(),
          params.raRegCopy);
      break;
    case RA_REG_COMBINE:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_COMBINE].enableMessage.c_str());
      params.raRegCombine = atof(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_COMBINE]
                     .valueSetMesssage.c_str(),
                 params.raRegCombine);
      break;
    case RA_REG_MUTATE:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_MUTATE].enableMessage.c_str());
      params.raRegMutate = atof(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MUTATE]
                     .valueSetMesssage.c_str(),
                 params.raRegMutate);
      break;
    case RA_REG_COMBINE_MUTATE:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_COMBINE_MUTATE]
                     .enableMessage.c_str());
      params.raRegCombineMutate = atof(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_COMBINE_MUTATE]
                     .valueSetMesssage.c_str(),
                 params.raRegCombineMutate);
      break;

    case RA_REG_MAX_COMBINE_ROOTS:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MAX_COMBINE_ROOTS]
                     .enableMessage.c_str());
      params.raRegMaxCombineRoots = atoi(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_MAX_COMBINE_ROOTS]
                     .valueSetMesssage.c_str(),
                 params.raRegMaxCombineRoots);
      break;
    case RA_REG_TOURNAMENT_SIZE:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_TOURNAMENT_SIZE]
                     .enableMessage.c_str());
      params.raRegTournamentSize = atoi(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_TOURNAMENT_SIZE]
                     .valueSetMesssage.c_str(),
                 params.raRegTournamentSize);
      break;
    case RA_REG_TOURNAMENT_ABORT:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_TOURNAMENT_ABORT]
                     .enableMessage.c_str());
      params.raRegTournamentAbort = atoi(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_TOURNAMENT_ABORT]
                     .valueSetMesssage.c_str(),
                 params.raRegTournamentAbort);
      break;
    case RA_REG_HIERARCHY_DESCEND:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_HIERARCHY_DESCEND]
                     .enableMessage.c_str());
      params.raRegHierarchyDescend = atof(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_HIERARCHY_DESCEND]
                     .valueSetMesssage.c_str(),
                 params.raRegHierarchyDescend);
      break;
    case RA_REG_ROUNDS:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_ROUNDS].enableMessage.c_str());
      params.raRegMinRounds = atoi(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_ROUNDS]
                     .valueSetMesssage.c_str(),
                 params.raRegMinRounds);
      break;
    case RA_REG_NO_IMPROVEMENT_ABORT:
      LOG_OUTPUT(
          LOG_M_ALWAYS,
          optionsPowerRegisterAllocation[RA_REG_ROUNDS].enableMessage.c_str());
      params.raRegNoImprovementAbort = atoi(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_ROUNDS]
                     .valueSetMesssage.c_str(),
                 params.raRegNoImprovementAbort);
      break;
    case RA_REG_POPULATION:
      LOG_OUTPUT(LOG_M_ALWAYS, optionsPowerRegisterAllocation[RA_REG_POPULATION]
                                   .enableMessage.c_str());
      params.raRegPopSize = atoi(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_POPULATION]
                     .valueSetMesssage.c_str(),
                 params.raRegPopSize);
      break;

    case RA_REG_POWER_EQUAL_THRESHOLD:
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_POWER_EQUAL_THRESHOLD]
                     .enableMessage.c_str());
      CONSTANT::powerEqualThreshold = atof(optarg);
      LOG_OUTPUT(LOG_M_ALWAYS,
                 optionsPowerRegisterAllocation[RA_REG_POWER_EQUAL_THRESHOLD]
                     .valueSetMesssage.c_str(),
                 CONSTANT::powerEqualThreshold);
      break;

    case '?':
      /* getopt_long already printed an error message. */
      printHelpOpts(argv[0]);
      exit(-1);

    default:
      abort();
    }
  }

  // Check if all necessary values are set, else exit printing error.
  if (params.asmfile == nullptr) {
    fprintf(stderr, "ERROR: ASM-File is not set\n");
    printHelpOpts(argv[0]);
    exit(-1);
  }
  if (params.config == nullptr) {
    fprintf(stderr, "ERROR: Config-File is not set\n");
    printHelpOpts(argv[0]);
    exit(-1);
  }
  if (params.fileSlmWeights != "") {
    params.optimization = 1;
    params.mergeLevel = 0;

    // enable parameters to read out the failure points
    params.application_mode = false;
    params.debugging = true;
    params.logMask |= LOG_M_CHECK_EXEC;

    LOG_OUTPUT(LOG_M_ALWAYS, "Overriding optimization level to %d\n",
               params.optimization);
    LOG_OUTPUT(LOG_M_ALWAYS, "Overriding merge level to %d\n",
               params.mergeLevel);

    ifstream ifile(params.fileSlmWeights);
    if (!ifile) {
      fprintf(stderr, "ERROR: Machine learning model file does not exist\n");
      printHelpOpts(argv[0]);
      exit(-1);
    }
  }
}

bool isPowerOptimization() {
  return !params.powerOptimizationFile.empty() & params.raRegPopSize > 0;
}