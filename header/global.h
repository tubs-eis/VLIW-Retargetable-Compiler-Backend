// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#ifndef GLOBAL_H
#define GLOBAL_H 1

#include "Constants.h"
#include <map>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#ifdef _OPENMP
#include "Constants.h"
#include <omp.h>
#endif

typedef int64_t LOG_MASK_T;

/** @brief Struct for the command line arguments.
 *
 * struct to help organize the command line arguments and have a better and more
 * understandable structure
 */
struct parameters {
  /* ***** INPUT/OUTPUT *************** */

  /** name of the ASM-File */
  char *asmfile = nullptr;
  /** name of the config file / processor description */
  char *config = nullptr;
  /** name of the binary output file */
  char *binary = nullptr;
  /** name of the readable output file */
  char *readable = nullptr;
  /** name of the output file for compiled code without register allocation */
  char *nonAllocReadable = nullptr;
  /** name of the graph output file */
  char *dot = nullptr;
  std::string dotAsmLineFile;
  /** name of the precompiled output file */
  char *precompiled = nullptr;
  /** name of the output file for code after initial read and regex replacement
   */
  char *initialassembler = nullptr;
  /** boolean, that defines, whether to ignore errors in the files or not */
  bool ignoreerror = false;
  /** Pointer to log output file. If no file is given on the command line,
   * stdout is used */
  FILE *logFile = stdout;
  /** Log mask */
  LOG_MASK_T logMask = 0;

  /* ***** SCHEDULING **************** */

  /* defines the level of optimization */
  int optimization = 1;
  /** Population size for the genetic scheduling */
  int schedPopulation = -1;
  /** Number of rounds in genetic scheduling */
  int schedRounds = 25;
  /** Number of best individuals to copy from one generation to the next. */
  int schedEliteCount = 3;
  /** Factor of population size: Number of randomly created individuals per
   * generation. */
  float schedRandomRatio = .01;
  /** Number of rounds when no solution can be found. If a solution is found,
   * continue with schedRounds.
   */
  int schedTrialRounds = abs(schedRounds);
  /* if to print scheduling statistics */
  char *sched_stats = nullptr;
  /** Scheduling is started only for merged SLMs when the minimal size if
   * smaller than the current best (+ offset) */
  bool enableSchedSkip = true;
  int schedSkipOffset = 0;
  float partProb = 0.01;
  float aloneProb = 0.001;
  bool includeDDGWeights = true;
  int schedTournamentSize = 3;
  bool printSchedPopulation = false;

  std::string compilableAssemblerFile;

  // external scheduling heuristic
  std::string fileMLModel = "";
  std::string stats_file = "";
  /* ***** REGISTER ALLOCATION ************************* */

  /** Is the heuristic register allocation enabled? */
  bool heuristicReg = true;
  /** Genetic algorithm for port optimal register allocation. */
  int portReg = 0;
  /** Number of rounds, before port optimal register allocation stops without a
   * result. */
  int portRegRounds = 50;
  int portRegNoImprovementAbort = 50;
  int portRegElitism = 5;
  /** Tournament rounds in port optimal register allocation. */
  int portRegTournament = 3;
  bool enableRASkip = true;
  int RASkipOffset = 0;
  int RAsmallerPenalty = NON_SCHEDULEABLE_VIRTUAL_ALLOC_OFFSET * 2;
  float raMutationRate = 0.01;
  bool printRegPopulation = false;
  /** Maximum number of individuals (scheduling) for which a genetic RA is
   * attemted */
  int RAPruneCount = -1;

  /* ***** POWER Register Allocation Parameter ************************* */

  std::string powerOptimizationFile;
  std::string instructionModelFile;

  std::string printEstimatedTransitionPower;
  std::string printEstimatedTransitionPowerCSV;

  float raRegRandom = 0.1;
  float raRegCopy = 0.05;
  float raRegCombine = 0.55; // =RA combine + raRegCombineMutate
  float raRegHierarchyDescend = 0.5;
  int raRegMaxCombineRoots = 3;
  float raRegCombineMutate = 0.30; // % vom Combine sollen mutiert werden
  float raRegMutate = 0.3;
  int raRegTournamentSize = 3;
  int raRegTournamentAbort = 30;
  int raRegMinRounds = 50;
  int raRegNoImprovementAbort = 20;
  int raRegPopSize = 0;
  int raRegDoublicateAbort = 5;
  int raRegMAXDoublicatesInPopulation = 3;
  int raRegMaxMutationDoublicateAbort = 4;
  float raRegFitnessEqualIgnoreForSecondarySelection = 1;
  int raRegMaxGeneration = 10000;
  float raRegMaxMutationFraction = 0.1;
  bool raRegRepairCombineGreedy = false;
  bool raRegCombineGreedy = false;
  bool raRegMutateGreedy = true;
  std::string raRegPopulationVerificationDir = "";
  float raRegMinImprovementFrac = 0.001;
  int raRegNoMinImprovement = 3;

  /* ***** RUNTIME ********************* */
  /** the number of threads to run in parallel */
  int threads = 1;

  /* ***** OPTIMIZATIONS **************** */

  /** if to replace immediate longs with immediate */
  bool replace = false;
  /** if to remove MV operations from and to virtual registers. */
  bool mvOptimizer = false;
  /** if to replace hardware registers with virtual ones.*/
  bool replaceHard = false;

  /* ***** INSTRUCTION MERGING ****************** */

  /** if to merge instructions into x2 */
  int mergeLevel = 0;
  /** population size for genetic merging */
  int mergePopulation = -1;
  /** Divider for merge population size */
  int mergePopsizeDiv = 4;
  /** Percentage of individuals from combine in merging */
  int mergeCmbRatio = 90;
  /** Number of rounds in genetic merging */
  int mergeRounds = 15;
  int mergeTrialRounds = abs(mergeRounds);
  /* mutation rate during instruction merging */
  float mergeMutationRate = 0.01;
  /* Use adaptive mutation rate in instruction merging */
  bool mergeMutateAdaptively = false;
  float mergeMutateSlope = 1;
  int mergeTournamentSize = 3;
  bool printMergePopulation = false;
  bool recursiveMerge = false;
  bool mergeFixAndVirtual = true;
  /** Probability for the individuals of first merging round to perform no
   * merging */
  float noMergeProbability = -1;

  /** The number (ID) of the SLM, where compilation starts. */
  int startSLM = 0;
  int stopSLM = -1;

  bool minPower = true;
};

/* The Logging system:
 *  Masks are used to enable single log sections, their are declared here and
 * defined in global.cpp. The function LOG_OUTPUT(mask, ...) can be used in the
 * code to print log information for the particular log section (specified by
 * the mask). Command line options allow selecting the log sections that should
 * be active.
 */
extern const LOG_MASK_T LOG_M_ALWAYS;
extern const LOG_MASK_T LOG_M_BASIC;

extern const LOG_MASK_T LOG_M_MERGE;
extern const LOG_MASK_T LOG_M_MERGE_DETAIL;
extern const LOG_MASK_T LOG_M_MERGE_DEBUG;
extern const LOG_MASK_T LOG_M_SCHED_SKIP;

extern const LOG_MASK_T LOG_M_SCHED;
extern const LOG_MASK_T LOG_M_SCHED_DETAIL;
extern const LOG_MASK_T LOG_M_SCHED_DEBUG;

extern const LOG_MASK_T LOG_M_RA;
extern const LOG_MASK_T LOG_M_RA_DETAIL;
extern const LOG_MASK_T LOG_M_RA_DEBUG;

extern const LOG_MASK_T LOG_M_INIT_PROC;
extern const LOG_MASK_T LOG_M_PARSE;
extern const LOG_MASK_T LOG_M_CHECK_EXEC;

extern const LOG_MASK_T LOG_M_RDG;
extern const LOG_MASK_T LOG_M_RDG_DETAIL;
extern const LOG_MASK_T LOG_M_RDG_DEBUG;
extern const LOG_MASK_T LOG_M_RDG_STAT;

extern const LOG_MASK_T LOG_M_RA_CONFLICT;
extern const LOG_MASK_T LOG_M_RA_CONFLICT_DETAIL;

extern const LOG_MASK_T LOG_M_RA_PREP;

extern const LOG_MASK_T LOG_M_SCHED_SIZE_HIST;
extern const LOG_MASK_T LOG_M_MERGE_SIZE_HIST;
extern const LOG_MASK_T LOG_M_RA_ROUNDS_HIST;
extern const LOG_MASK_T LOG_M_RA_SIZE_HIST;

extern const LOG_MASK_T LOG_M_REG_ENERGY_DEBUG;
// extern const LOG_MASK_T LOG_M_REG_DETAIL;
extern const LOG_MASK_T LOG_M_REG_ENERGY;

void LOG_OUTPUT(bool cond, ...);
void LOG_OUTPUT(LOG_MASK_T mask, ...);
bool isLog(LOG_MASK_T mask);
typedef std::map<std::string, LOG_MASK_T> LOG_MAP_T;
extern LOG_MAP_T LOG_MASKS;
void flushLog();

// distances are generated when allocating virtual registers. It gives a sense
// of how long a pipeline could be. the printout can easily be visualized using
// gnuplot.
//#define PRINTOUT_DISTANCES 1

#ifdef PRINTOUT_DISTANCES
#ifdef SETDEBUG
int distances[2048];
#else
extern int distances[2048];
#endif
#endif

#ifdef SETDEBUG
/** \brief Global indicator if the program has received a SIGINT. */
bool killed;
/** \brief The Command line parameters.
 *
 *  This is global, since some functions need the debug- and optimizationlevel.
 */
struct parameters params;
#else
extern bool killed;
extern parameters params;
#endif

#define DEBUG_VIRTUAL_ALLOC 0

#define CHECK_RA_TIMING 0
#define CHECK_GA_STATS 0

#ifdef _OPENMP
#define MAX_THREAD_COUNT 128
#define CURRENT_THREAD_NUM omp_get_thread_num()
#else
#define MAX_THREAD_COUNT 1
#define CURRENT_THREAD_NUM 0
#endif

/** \brief trims leading ad tailing whitespace from a string. */
char *trim(char *s, int length);
/** \brief Copies the content from one string to another. */
void strcopy(char *to, char *from, int length = -1);

#if (__GNUC__ == 4 && __GNUC_MINOR__ > 8) || __GNUC__ > 4
#define GCCREGEX 1
#else
// not everyone needs 32 bit as a maximum for a parameter. maybe make it
// flexible for 64 bit/16 bit?
// @test check if this really works with 4.9 or just with 5.1.
#define char32_t uint32_t
#endif

/** \brief The maximum number of bits an Opcode could have.
 *
 * Opcode is the binary coding of an operation and might first look like this:
 * 01001AAAAAABBBBCCCC
 * Here A,B,C are placeholder for the bitcoding of the first, second, third
 * operand.
 * */
#ifndef OPCODELENGTH
#define OPCODELENGTH 64
#endif

/** \brief The maximum number of bits for an operation name. */
#ifndef NAMELENGTH
#define NAMELENGTH 16
#endif

/** \brief The maximum number of arguments an operation could have
 *
 * This must include all implicit arguments. So for example, even if the MAC
 * operation only has 3 Arguments, the OPCODE must consist of 4, since the
 * second write register is implicit given by the first argument and generated
 * during execution.
 *
 * It is now set to 8 Arguments, since 1-4 are used for MAC operations and 5-8
 * are the copies for X2 Operations. it might be possible to break it down to 6
 * or even 5, but at the moment, this was easier to program and the extra memory
 * is not that big.
 */
#ifndef MAXARGNUMBER
#define MAXARGNUMBER 8
#endif

#define EXIT_ERROR                                                             \
  {                                                                            \
    fprintf(params.logFile, "ERROR: it occoured in %s %s line %d\n", __FILE__, \
            __PRETTY_FUNCTION__, __LINE__);                                    \
    if (!params.ignoreerror)                                                   \
      exit(-1);                                                                \
  }

#ifdef __GNUC__
#define DEPRECATED(func) func __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED(func) __declspec(deprecated) func
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED(func) func
#endif

#endif
