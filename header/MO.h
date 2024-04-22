// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef MO_H
#define MO_H 1

#include "gen_sched.h"
#include "global.h"
#include "operation.h"
#include "processor.h"
#include <bitset>
#include <iostream>
#include <set>
#include <string>

// class gen_sched::sched_chromosome; /** Representation of a microoperation */
class MO {
private:
  /** the arguments this operation has
   * Mapping of arg.
   * 0 = write port
   * 1 = src0
   * 2 = src1
   * 3 = x
   * 4 = 2. write port (MAC  uses this)
   * 5 = src0 2. read port (X2)
   * 6 = src1 2. read port (X2)
   * 7 = x
   * */
  char32_t arg[MAXARGNUMBER];
  /** the arguments this operation has */
  char dir[MAXARGNUMBER];
  /** the types of the arguments. wheather it is a Register, Label or an
   * Immediate.
   * Mapping of types.
   * 0 = write port
   * 1 = src0
   * 2 = src1
   * 3 = x
   * 4 = 2. write port (MAC  uses this)
   * 5 = src0 2. read port (X2)
   * 6 = src1 2. read port (X2)
   * 7 = x
   * */
  OPtype types[MAXARGNUMBER];
  /** bitcoded where the lowest bit ist slot 0 and a 1 means only there
   * possible. */
  int issueSlotContraint;
  /** the number of arguments */
  int argNumber;
  int latency;
  /** Latency used, if the MO is placed near the end of the scheduled SLM.
   * If this is lower than the latency, this might reduce the number of NOPs
   * introduced as wait cycles.
   */
  int finalLatency;
  std::vector<MO *> previous, following;
  std::vector<MO *> weakPrevious, weakFollowing;
  std::vector<MO *> previousCondition, followingCondition;
  std::vector<MO *> previousFlags;
  std::vector<MO *> followingFlags;
  std::vector<MO *> weakPreviousFlags, weakFollowingFlags;

  /** Length of longest path below this MO + this->latency */
  int weight;
  /** Depth of the MO in the DDG. Length of a (longest) path from a root to this
   * MO */
  int depth;
  int follower;
  void calculateFollower(set<MO *> *IDs);
  /** the operation this MO belongs to */
  Operation *op; // reference to the operation to produce the bytecode.
  /** the line number of the ASM file where this MO is created from */
  uint lineNumber;
  /** \brief second line number if this operation is merged from two existing
   * ones. */
  uint lineNumber2;
  /** the unique ID */
  int id;
  bool parting;

  bool reorderable;
  bool onLongesPath;
  bool CSOperation;
  bool CROperation;
  bool CRSOperation;
  bool SMVtoSingleCondselFlag;
  bool SMVtoAllCondselFlag;
  int idConditions;

  std::string printSchedChromosomeInfo(
      gen_sched::sched_chromosome *schedChromosome = nullptr, int index = 0);

public:
#if STORE_PARTINGS_AND_ALONE
  bool wasParting, wasAlone;
#endif
  /** counter to set the ID. Counts the number of MO */
  static int counter[MAX_THREAD_COUNT];
  /** helper function
   *
   * this should only be used to search for an MO by ID. It is not possible to
   * schedule this MO and may result in Nullpointerexceptions if trying to call
   * any of its functions.
   */
  MO(int id) {
    this->id = id;
    this->idConditions = id;
    finalLatency = -1;
    op = nullptr;
  }
  MO(Operation *op, char32_t *arguments, OPtype *typ, char *dir);
  MO(Operation *op);
  ~MO() {
    previous.clear();
    following.clear();
  }

  /** @brief sets the constrains for issueslots.
   *
   * The constrains has to be read as followed:
   * for each issue slot the corresponding bit is set to one, if a constraint
   * exist. so, if there are no constraints, this value should be 0. if the
   * ASM-Code restricts the execution to just issueslot 3 the value should be 4
   * (0b00000100) if the ASM-Code restricts the execution to issueslot 1 and 2
   * the value should be 3 (0b00000011)
   * @param slot the constrain bitcode.
   */
  void setIssueSlotConstraint(int slot) { issueSlotContraint = slot; }
  /** @brief returns the constrains for issueslots.
   *
   * The constrains has to be read as followed:
   * for each issue slot the corresponding bit is set to one, if a constraint
   * exist. so, if there are no constraints, this value is 0. if the ASM-Code
   * restricts the execution to just issueslot 3 the returned value would be 4
   * (0b00000100) if the ASM-Code restricts the execution to issueslot 1 and 2
   * the returned value would be 3 (0b00000011)
   * @return the constrain bitcode.
   */
  int getIssueSlotConstraint() const { return issueSlotContraint; }
  /** @brief Returns wheter or not this operation branches to another point.
   *
   * @return true If it branches.
   */
  bool isBranchOperation() { return op->isBranchOperation(); }
  /** @brief Check for STORE operation into memory.
   *
   * If STORE is used to write to DMA control registers, this does not count as
   * a STORE into memory!
   */
  bool isMemStore(Processor::DMAAddrConfig *dmaAddrConfig);
  bool isMemLoad();

  bool isCRBranchOperation() { return op->isCRBranchOperation(); }
  /** @brief Does this operation write to an idirect operand? */
  bool isWriteIndirect();
  bool isReadIndirect();
  /** @brief returns the multiplier of this MO.
   *
   * If an Operation uses more then one issue slot, because its opcode is a
   * multiple of the codesize, then its multiplier is higher then 1. This
   * function returns the number of issueslots which are needed to execute the
   * operation.
   * @return Number of issue slots.
   */
  int opLengthMultiplier() const { return op->opLengthMultiplier(); }
  /** @brief Sets the line number
   *
   * @param line The line number.
   */
  void setLineNumber(uint line) { lineNumber = line; }
  /** @brief Returns the line number this MO has been parsed from.
   *
   * The line of code in the ASM file.
   * @return the line number
   */
  const uint getLineNumber() const { return lineNumber; }
  /** @brief Sets the second line number
   *
   * This line number is set, if the operation is parsed as a X2 operation from
   * two different normal operations.
   * @param line The line number.
   */
  void setSecondLineNumber(uint line) { lineNumber2 = line; }
  /** @brief Returns the second line number this MO has been parsed from.
   *
   * This line number is set, if the operation is parsed as a X2 operation from
   * two different normal operations. The line of code in the ASM file.
   * @return the line number
   */
  const uint getSecondLineNumber() const { return lineNumber2; }
  /** @brief Returns the ID
   *
   * The ID is unique over all MO.
   * @return The ID
   */
  int getID() const { return id; }

  int getIDConditions() const { return idConditions; }
  /** @sets the Operation of this MO new
   *
   * this function should be used with care, since it just sets the operation,
   * but not the arguments and everything else.
   * @param o The new Operation
   */
  void reSetOperation(Operation *o) { this->op = o; }
  /** @brief Returns the Operation used by this MO
   *
   * @return the operation
   */
  Operation *getOperation() { return op; }

  const Operation *getOperation() const { return op; }
  /** @brief Returns the types of the arguments
   *
   * A Pointer to the array of types used by this MO is returned. Altering them
   * will directly affect the MO.
   * @return array of types
   */
  OPtype *getTypes() { return types; }
  /** @brief Returns the types of the arguments
   *
   * A Pointer to the array of types used by this MO is returned. Altering them
   * will directly affect the MO.
   * @return array of types
   */
  const OPtype *getTypes() const { return types; }
  /** @brief Returns the arguments
   *
   * A Pointer to the array of arguments used by this MO is returned. Altering
   * them will directly affect the MO.
   * The arguments map the src0,src1 arguments to the proper
   * register/virtualregs Mapping of arg. 0 = write port 1 = src0 2 = src1 3 = x
   * 4 = 2. write port (MAC  uses this)
   * 5 = src0 2. read port (X2)
   * 6 = src1 2. read port (X2)
   * 7 = x
   *
   * @return array of arguments
   */
  char32_t *getArguments() { return arg; }
  /** @brief Returns the arguments
   *
   * A Pointer to the array of arguments
   * used by this MO is returned. Altering
   * them will directly affect the MO.
   * Mapping of arg.
   * 0 = write port
   * 1 = src0
   * 2 = src1
   * 3 = x
   * 4 = 2. write port (MAC  uses this)
   * 5 = x
   * 6 = x
   * 7 = x
   *
   * @return array of arguments
   */
  char32_t const *getArguments() const { return arg; }

  /** @brief Returns the directions of the arguments.
   *
   * A pointer to to array of bool values used by this MO is returned. Altering
   * them will directly affect the MO.
   *
   * if an Value is TRUE, the argument writes to an register, FALSE otherwise.
   * @return array of boolean values.
   */
  char *getDirections() { return dir; }
  /** @brief Returns the number of arguments for this MO
   *
   * A number between 0 and MAXARGNUMBER
   * @return The number of arguments.
   */
  int getArgNumber() const { return argNumber; }
  /** @brief Sets the number of arguments for this MO
   *
   * A number between 0 and MAXARGNUMBER
   * @param n The number of arguments.
   */
  void setArgNumber(int n) { argNumber = n; }
  /** @brief returns the latency for this operation
   *
   * @return The latency in cycles.
   */
  int getLatency() const { return latency; }

  /** @brief Return latency for the operation at the end of a scheduled SLM.
   *
   * If the final latency is not set, the default latency is returned.
   */
  int getFinalLatency() const {
    return finalLatency == -1 ? latency : finalLatency;
  }

  void setFinalLatency(int final) { finalLatency = final; }

  /** @brief Sets the latency for this operation
   *
   * normally this should not be nessesary, since the latency is gathered from
   * the operation, but it can be overwritten by this.
   * @param New latency value in clock cycle.
   */
  void reSetLatency(int lat) { latency = lat; }
  /** @brief Writes a binary representation of the MO to the given outputstream.
   *
   * A binary representation of the of the compiled code is written to the
   * outputstream. The arguments of an MO are combined with the opcode of the
   * operation and insertet at the right places.
   *
   * @param out An output stream. Should be to a file.
   */
  void writeOutBin(ostream &out) { op->writeOutBin(out, this); }
  /** @brief Writes human readable representation of the parsed SLM's to the
   * given outputstream.
   *
   * A human readable representation of the the compiled code is written to the
   * outputstream. The arguments of an MO are combined with the opcode of the
   * operation and insertet at the right places.
   *
   * @param out An output stream. Could be std::out.
   */
  void writeOutReadable(ostream &out,
                        const VirtualRegisterMap *mapping = nullptr) const {
    op->writeOutReadable(out, this, mapping);
  }

  void writeOutCompilable(ostream &out,
                          const VirtualRegisterMap *mapping = nullptr) const {
    op->writeOutCompilable(out, this, mapping);
  }

  std::string to_string(VirtualRegisterMap const *mapping = nullptr) {
    if (op) {
      return op->to_string(this, mapping);
    }
    return "";
  }
  /** for nice printing out*/
  friend std::ostream &operator<<(std::ostream &out, const MO &mo) {
    mo.op->writeOutReadable(out, (MO *)&mo);
    return out << " Issueconstraint: " << std::bitset<2>(mo.issueSlotContraint);
  }

  string getString() {
    stringstream ss;
    ss << op->getName() << "\t";
    for (int i = 0; i < MAXARGNUMBER; i++) {
      switch (types[i]) {
      case OPtype::Error:
        ss << "-";
        break;
      case OPtype::REG:
        ss << OPtypeStr[types[i]] << " : " << registers::getName(arg[i]);
        break;
      default:
        ss << OPtypeStr[types[i]] << " : " << arg[i];
      }
      ss << "\t";
    }
    return ss.str();
  }

  /**
   * This method is used in dot graph generation to keep track of alive
   * registers of the MAC operation (both read/write register).
   * @return
   */
  string getRegisterString() {
    stringstream ss;
    //    std::vector<int> ports =
    std::string prefix = "w=";
    for (int i : {0, 4, 1, 2, 5, 6}) {
      switch (types[i]) {
      case OPtype::REG:
        ss << prefix << registers::getName(arg[i]) << "+";
        break;
      default:
        break;
      }
      if (i == 4) {
        prefix = "r=";
      }
    }
    return ss.str();
  }

  void setParting(bool par) { parting = par; }

  bool isParting() const { return parting; }

  bool isX2Operation() const { return op->isX2Operation(); }

  void setCSOperation() { CSOperation = true; }

  bool isCSOperation() const { return CSOperation; }

  void setCROperation() { CROperation = true; }

  bool isCROperation() const { return CROperation; }

  void setCRSOperation() { CRSOperation = true; }

  bool isCRSOperation() const { return CRSOperation; }

  void setSMVtoSingleCondselFlag() { SMVtoSingleCondselFlag = true; }

  bool isSMVtoSingleCondselFlag() const { return SMVtoSingleCondselFlag; }

  void setSMVtoAllCondselFlag() { SMVtoAllCondselFlag = true; }

  bool isSMVtoAllCondselFlag() const { return SMVtoAllCondselFlag; }

  bool isImm() const {
    return (types[2] == OPtype::Immediate || types[2] == OPtype::Imm32);
  }

  bool operator==(const MO &param) const { return param.id == id; }
  bool operator!=(const MO &param) const { return param.id != id; }

  bool isWriteReg(int argIdx);
  bool isVirtualWriteArg(int argIdx);
  bool isPhysicalWriteArg(int argIdx);

  const RegisterCouple *isCoupledWriteArg(const rdg::RDG *rdg) const;

  void setOperation(Operation *operation);

  /**
   * Add a dependency.
   * @param next has to be executed after this operation (not even same issue
   * slot).
   */
  void addFollower(MO *next);
  /***
   * Add a weak dependency.
   * @param next has to be executed at or after this operation (can be in same
   * issue slot, however never before this operation).
   */
  void addWeakFollower(MO *next);

  void addFollowerCondition(MO *next);
  void addFollowerFlags(MO *next);
  void addWeakFollowerFlags(MO *next);

  void deleteFollowers();
  bool hasPrev() const { return !previous.empty(); }
  bool hasWeakPrev() const { return !weakPrevious.empty(); }
  bool hasPrevFlags() const { return !previousFlags.empty(); }
  bool hasWeakPrevFlags() { return !weakPreviousFlags.empty(); }
  bool hasFollower() const { return !following.empty(); }
  int countPrev() const { return previous.size(); }
  /***
   *
   * @return return the list scheduling weight. (max. latency path to leaf)
   */
  int getWeight();
  int getDepth();
  int countFollower(int weight = 0);
  int getNumberOfFollower();
  std::vector<MO *> *getFollowing() { return &following; }
  std::vector<MO *> *getWeakFollowers() { return &weakFollowing; }
  std::vector<MO *> *getFollowingCondition() { return &followingCondition; }
  std::vector<MO *> *getPrevious() { return &previous; }
  std::vector<MO *> *getWeakPrevious() { return &weakPrevious; }
  std::vector<MO *> *getPreviousCondition() { return &previousCondition; }
  std::vector<MO *> *getPreviousFlags() { return &previousFlags; }
  std::vector<MO *> *getFollowingFlags() { return &followingFlags; }
  std::vector<MO *> *getWeakPreviousFlags() { return &weakPreviousFlags; }
  std::vector<MO *> *getWeakFollowingFlags() { return &weakFollowingFlags; }
  void writeOutDot(ostream &out,
                   gen_sched::sched_chromosome *schedChromosome = nullptr,
                   int index = 0, const rdg::RDG *rdg = nullptr,
                   const VirtualRegisterMap *map = nullptr);
  std::string get_register_info();
  MO *copy();

  /** \brief Setter if scheduling could be done.
   *
   * When encountering a --scheduling-off instruction the following
   * instructions are set to not be reorderable. This happens until a
   * --scheduling-on instruction is found.
   * @sa isReorderable()
   * @param [in] albe Reorderability of this instruction.
   */
  void setReorderable(bool able) { reorderable = able; }

  /**\brief Indicates whether or not this MO can be reordert.
   *
   * When encountering a --scheduling-off instruction the following
   * instructions are set to not be reorderable. This happens until a
   * --scheduling-on instruction is found.
   * @sa setReorderable()
   * @return Wheter this instruction can be reordert with other instructions.
   */
  bool isReorderable() const { return reorderable; }

  void setOnLongestPath() {
    if (onLongesPath)
      return;
    onLongesPath = true;
    for (auto it = following.begin(), end = following.end(); it != end; it++) {
      int lat = (op->getBaseName() == "STORERCUPPL") ? 1 : latency;
      if ((*it)->getWeight() == weight - lat)
        (*it)->setOnLongestPath();
    }
  }

  bool isOnLongestPath() const { return onLongesPath; }

  bool hasFollower(const string &name);

  void _draw_dot_physical_register_from_previous_iteration(ostream &out);
};

#endif
