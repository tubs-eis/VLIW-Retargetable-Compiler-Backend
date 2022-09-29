// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#ifndef SLM_H
#define SLM_H 1

#include "MI.h"
#include "MO.h"
#include "global.h"
#include "virtual_reg.h"
#include <set>
#include <unordered_map>

class SLM;
class Processor;

/** @brief a collection of microoperations with at maximum one branch operation
 * at the end SLM stands for Straight Line of Microcode and is a collection of
 * microcode which has no outside connections and could be scheduled alone.
 *
 * Code is separated into SLM's by using labels, branch-operations and via the
 * pseudo-instructions
 * --scheduling-{on|off}
 *
 * An SLM contains information about register allocation results from heuristic
 * and/or genetic allocation. Additionally an alternative scheduling of the SLM
 * is saved, which has better stats than this SLM. However it has the same SLMs
 * inside!
 *
 */
class SLM {
private:
  /** all microoperations in this SLM. */
  vector<MO *> *ops;

  MO *branchOperation;

  //#if defined(_OPENMP)
  //        /** \brief Lock variable for adding the last instruction.
  //         *
  //         * Since the genetic algorithm runs in parallel and multiple
  //         instances may want to add their results,
  //         * this lock will prevent them from having race conditions when
  //         comparing for the shortest number of instructions.
  //         * it is likely never going to happen, but better safe than sorry.
  //         */
  //        omp_lock_t addLastInsLock;
  //#endif
  /** @the Label before this SLM
   * if there was none, this is NULL
   */
  char32_t label;
  /** @brief The offset to the first SLM
   * this must be set after the scheduling and should be the offset of the
   * previous SLM plus the Size of the previous SLM
   */
  char32_t offset;

  /** the ID of this SLM */
  int id;
  /** \brief id of the father SLM
   *
   * If an SLM is copied, the ID of the SLM being copied is passed on to the son
   * and stored in this variable. The reason is that some helper functions store
   * data, depending on the SLM ID.
   */
  int orignalid;
  /** the minimum number of MI's for this SLM. */
  int minimalSize;
  /** Length of longest path through the operations. */
  int criticalPath;
  /** \brief ID of registers being free'd in the code via the "FREEREG" macro.
   *
   * This is used as a transporter from the read file to the virtual register
   * initialization, where the blocking is calculated.
   *
   */
  std::set<char32_t> freeReg;
  /** \bried ID if registers being fixed in the code via the "FIXREG"
   * pseudo-instruction
   *
   */
  std::set<char32_t> fixReg;

  /* \brief The mapping from virtual to physical registers.
   *
   * This is set together with ins, and is used by the verifier to map the
   * registers. They cannot be set on probing, because otherwise no other
   * configurations would be available.
   */
  VirtualRegisterMap *map = 0;
  VirtualRegisterMap *map_per_thread[MAX_THREAD_COUNT];
  /** \brief List of MI which form the best known compaction.
   *
   */
  Program *ins = 0;
  Program *ins_per_thread[MAX_THREAD_COUNT];
  /* \brief Pointer to the alternative SLM.
   *
   * When an alternative SLM is chosen to perform better, it is stored here.
   */
  SLM *alternative;

  float booster;

  int minDistanceX2Merging;
  int maxDistanceX2Merging;

  int optimizationLevel;
  int mergeLevel;
  float partProb;
  float aloneProb;
  bool _haveRAPruneCount;
  int RApruneCount;

  int _registerHeuristicFit;
  int _registerGeneticFit;

public:
  void setInstructionAndMap(Program **ins, VirtualRegisterMap **map);

  /** a static counter to give every SLM its own ID. */
  static int counter[MAX_THREAD_COUNT];
  /** \brief Default value for maxDistanceX2Merging when not set in assembler
   * code */
  static const int emptyMaxDistanceX2Merging;

  /**
   *
   * @return The ID of the original SLM ID. (Who is the root SLM-ID?)
   */
  int getOriginalID() { return orignalid; }
  void setOriginalID(int id) { orignalid = id; }
  std::set<char32_t> *getFreeReg() { return &freeReg; }

  std::set<char32_t> *getFixReg() { return &fixReg; }

  void addFreeReg(char32_t ID) { freeReg.insert(ID); }

  void addFixReg(char32_t ID) { fixReg.insert(ID); }

  MO *getBranchOperation() { return branchOperation; }
  /** @brief Initializes an SLM
   * an empty SLM with no Operations is created.
   * @param [in] schedule defines if the operations could be reordert.
   */
  SLM();
  ~SLM();
  void releaseMIs();
  /** @brief adds an MO to the SLM
   * the List of MO to execute is extended by the given MO
   * @param [in] operation A microoperation to extend this SLM
   */
  void addMO(MO *operation);
  /** @brief sets the Label for this SLM
   * the label schould be immediatly before the first instruction of this SLM
   * an ID for any Label could be obtained by using char32_t
   * label::getLabelID(char* label);
   * @sa label::getLabelID(char* label);
   * @param [in] label The ID of the label.
   */
  void setLabel(char32_t label);
  /** @brief returns the set label.
   * @return -2 if no Label is set.
   * @return The ID of the set label
   */
  char32_t getLabel() { return label; }

  /** @brief return the number of bytes, before this SLM starts.
   * this method is used for the binary representation of the labels.
   * as every label points to a point in the code, this point is the offset of
   * the corrresponding SLM. the offset can be interpreted as the number of MI
   * before this SLM
   * @return the Offset to the first SLM
   */
  char32_t getOffset() { return offset; }
  /** @brief Sets the offset to the beginning of Code
   * @param [in] offset The Offset to for this SLM
   */
  void setOffset(char32_t off) {
    if (alternative != NULL)
      alternative->setOffset(off);
    this->offset = off;
  }
  /** @brief returns the number of MI this SLM has.
   * it returns the length of the MI returned by getLastShortestInstruction()
   * @return the minimal number of MI
   */
  char32_t getSize() {
    if (alternative != NULL)
      return alternative->getSize();
    if (ins != NULL)
      return ins->size();
    return 0;
  }
  /** @brief Writes a binary representation of the parsed SLM to the given
   * outputstream.
   *
   * a binary representation of the of the compiled code is written to the
   * outputstream. The shortest possible path as defined by
   * getLastShortestInstruction() is written out.
   *
   * @param out an output stream. Should be to a file.
   */
  void writeOutBin(ostream &out);
  /** @brief Writes human readable representation of the parsed SLM's to the
   * given outputstream.
   *
   * a human readable representation of the of the compiled code is written to
   * the outputstream. The shortest possible path as defined by
   * getLastShortestInstruction() is written out.
   *
   * @param out an output stream. Could be std::out.
   */
  void writeOutReadable(ostream &out, bool printEnergy = false);

  void writeOutCompilable(ostream &out);

  void writeOutDot(Processor *pro, const char *path);
  void writeOutDot(Processor *pro, ostream &out);

  void releaseGraph();
  /**
   * Generates a Def-Use Graph for the MOs inside this SLM.
   * The Graph information are embedded into the MOs through adding
   * Followers/FollowerCondition etc.
   * @param pro
   * @param initialGraph
   */
  void calculateGraph(Processor *pro, bool initialGraph);

  void releaseRegisterMapping();

  int weightSum();
  int depthSum();

  /** @brief returns the ID of this SLM
   * the ID is unique over every SLM and could be used to identify the SLM's
   * @return the ID
   */
  int getID() { return id; }

  /** @brief Returns all operations of this SLM
   *
   * @note it returns the link to the vector used by this SLM, so no copy is
   * generated and alterations directly affects this SLM.
   * @return The adress of the MO vector.
   */
  vector<MO *> *getOperations() { return ops; }

  const vector<MO *> *getOperations() const { return ops; }

  /** @brief Setting the shortest list of MI
   *
   * It is crucial to understand, that this function is not a real setter. It is
   * more a compare and cleanup function. The given parameters can be null. If
   * they are not, then the SLM handles their destruction. One can not assume
   * the vectors to be available after passing them to the function. If they
   * form the next best result, they are stored for future use, if not, they are
   * immediately deleted to free memory. Delete is also used on every element of
   * the instruction vector.
   *
   *
   * @param [in] ins Vector of MI which form a valid set of instructions.
   * @param [in] map The mapping from virtual to real registers as given back by
   * the virtual register testing.
   */
  bool setInstructions(Program **ins, VirtualRegisterMap **map);
  void collectInstructionsFromThreads();
  /** \brief Set a SLM with an alternative instruction set but the same
   * functionality.
   *
   * This is not a real setter! More a compare and cleanup function.
   * The given SLM can be null. If not, their instruction set are compared with
   * this SLM and if it better, it is stored for future use. If not the SLM is
   * deleted to free memory resources. One must not use the given SLM after
   * calling this function any more!
   *
   */
  void setAlternative(SLM **alt);

  /** @brief
   */
  Program *getShortestInstruction() {
    if (alternative != NULL)
      return alternative->ins;
    return ins;
  }
  unsigned int getShortestSize() {
    Program *ins = getShortestInstruction();
    if (ins)
      return ins->size();
    else
      return (unsigned int)-1;
  }
  VirtualRegisterMap *getVirtualMapping() {
    if (alternative != NULL)
      return alternative->map;
    return map;
  }

  int getMinimalSize() {
    if (alternative != NULL)
      return alternative->minimalSize;
    return minimalSize;
  }

  int getCriticalPath() {
    if (alternative != NULL)
      return alternative->criticalPath;
    return criticalPath;
  }

  /** Gets the booster value for an SLM.
   *
   */
  float getBooster() const { return booster; }

  /** Sets the booster value for an SLM
   *
   * the booster can be used to partially increase the number of for a specific
   * SLM.
   */
  void setBooster(float booster) {
    this->booster = std::max((float)1.0, booster);
  }

  int getMinDistanceX2Merging() const { return minDistanceX2Merging; }

  void setMinDistanceX2Merging(int minDistanceX2Merging) {
    this->minDistanceX2Merging = minDistanceX2Merging;
  }

  int getMaxDistanceX2Merging() const { return maxDistanceX2Merging; }

  void setMaxDistanceX2Merging(int maxDistanceX2Merging) {
    this->maxDistanceX2Merging = maxDistanceX2Merging;
  }

  void setOptimizationLevel(int opt_level) {
    this->optimizationLevel = opt_level;
  }

  int getOptimizationLevel() { return this->optimizationLevel; }

  void setMergeLevel(int merge_level) { this->mergeLevel = merge_level; }

  int getMergeLevel() { return this->mergeLevel; }

  float getPartProb() const { return partProb; }

  void setPartProb(float prob) { partProb = prob; }

  float getAloneProb() const { return aloneProb; }

  void setAloneProb(float prob) { aloneProb = prob; }

  int getRAPruneCount() const { return RApruneCount; }

  void setRAPruneCount(int count) {
    RApruneCount = count;
    _haveRAPruneCount = true;
  }

  void clearRAPruneCount() { _haveRAPruneCount = false; }

  bool haveRAPruneCount() const { return _haveRAPruneCount; }

  void setHeuristicRegisterFitness(int const heuristicRegisterFit) {
    _registerHeuristicFit = heuristicRegisterFit;
  }

  int getHeuristicRegisterFitness() const { return _registerHeuristicFit; }

  void setGeneticRegisterFitness(int const registerGeneticFit) {
    _registerGeneticFit = registerGeneticFit;
  }

  int getGeneticRegisterFitness() const { return _registerGeneticFit; }

  /**
   * Print SLM to LOG and also return a string containing the information.
   */
  void print() {
    stringstream ss;
    for (auto mo : *ops) {
      ss << mo->getString() << "\n";
    }
    cout << ss.str();
    LOG_OUTPUT(LOG_M_ALWAYS, "\nPRINT SLM--> \n%s", ss.str().c_str());
  }

  void writeOutTransitionPowerEstimate(std::ostream &out);
  void calculateTransitionEnergy();
};

int getMONum(const std::vector<MO *> *ops, int i);

#endif
