// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_RDG_H_
#define HEADER_RDG_H_

// #include "MI.h"

#include "Program.h"
#include "operation.h"
#include "virtual_reg.h"

#include <functional>
#include <ostream>
#include <vector>

namespace portOptReg {
class Chromosome;
}

namespace rdg {

#define DEBUG_INFEASIBLE_RDG 0
#define MAX_INFEASIBLE_DEBUG 100

/*!
 * \brief A Register dependency graph.
 */
class RDG {
public:
  static const bool SAME_RF = true;
  static const bool DIFFERENT_RF = false;
  static const char32_t NO_REG;

  enum DEP_TYPE { DEP_NO_DEP, DEP_SAME_RF, DEP_DIFFERENT_RF };

  /**
   * \brief Entry in the Register Dependency Graph.
   * This is an entry inside the RDG. One Entry knowns its parents as well as
   * all constraints (such X2, MAC and RF Unity).
   */
  struct Entry {
    /** Can be a physical register, a buffer register or a virtual register. */
    char32_t reg;
    Entry *parent;
    bool same_rf;
    Entry *cond_parent;
    bool same_rf_cond;
    int cond_rf;

    int depth;
    int readCount;
    int index;
    const RegisterCouple *couple;

    Entry(char32_t reg)
        : reg(reg), parent(0), same_rf(true), cond_parent(0),
          same_rf_cond(true), cond_rf(0), depth(1), readCount(0), index(-1),
          couple(0) {}

    bool isVirtual() const { return registers::isVirtualReg(reg); }
    bool isPhysical() const { return registers::isPhysicalRegister(reg); }

    bool isRoot() const { return !parent; }

    bool isSingleRegister() const { return !couple; }

    void setParent(Entry *parent) { this->parent = parent; }

    int treeDepth() const {
      int d = 0;
      const Entry *e = this;
      while (e) {
        ++d;
        e = e->parent;
      }
      return d;
    }
  };
  struct RegArgument {
    char32_t reg;
    char32_t reg_X2;

    RegArgument(char32_t reg) : reg(reg), reg_X2(RDG::NO_REG) {}
    RegArgument(char32_t reg, char32_t reg_X2) : reg(reg), reg_X2(reg_X2) {}
    bool isX2() const { return reg_X2 != RDG::NO_REG; }
    bool isVirtual() const { return registers::isVirtualReg(reg); }
    bool isPhysical() const { return registers::isPhysicalRegister(reg); }
  };

  void setAllocation(int *allocations, const int *genes, const Entry *e,
                     bool *visited, int depth = 1) const;

private:
  /** Sum of all number of physical and virtual registers. **/
  unsigned int _size;
  Entry **_entries;
  int _rootCount;
  Entry *find(char32_t reg, bool &same_rf) const;

  void setParent(Entry *sub, Entry *parent, bool same_rf, int cond_RF);
  bool checkFixRegisters(Entry *r1, Entry *r2, bool same_rf);

  static char32_t *DEFAULT_REG;
  static char32_t getDefaultReg(int regfile);

  static unsigned int _maxReadPorts, _maxWritePorts;

  struct RegisterArg {
    char32_t reg;
    OPtype type;
    char dir;
  };

  /**
   * \brief Process register _couplings from calculateCouplings.
   *
   * The register _couplings may be between virtual+virtual, or
   * virtual+fix/fix+virtual. For _couplings between two fix registers, the
   * coupling is checked for feasibility. If the coupling of two fix register is
   * not allowed, the function returns <code>false</code>.
   *
   * \param couplings
   * 		The list of register _couplings as computed by
   * calculateCouplings.
   */
  bool recordRegisterCouplings(const RegisterCoupling *couplings);
  bool analyseInstructions(Program *instructions, VirtualRegisterMap *map);
  bool checkPortConflicts(const RegArgument &arg, unsigned int *freePorts);
  bool checkRegisterDependencies(MO *mo1, MO *mo2, VirtualRegisterMap *map);
  bool checkFeasibility(Program *instructions, VirtualRegisterMap *map,
                        ass_reg_t *blockedRegs);
  bool checkReadPorts(std::vector<RegisterArg> &registerArgs,
                      unsigned int registerCount, unsigned int maxReadPorts,
                      unsigned int argIdx);
  bool recordWriteConflicts(const RegArgument &a1, const RDG::RegArgument &a2,
                            int cond_RF = -1);
  void updateLiveRegs(Program::iterator miIt, char32_t arg,
                      VirtualRegisterMap *map, std::set<char32_t> &dying,
                      int &liveRegs);

  static std::vector<RegisterArg> collectRegisterArgs(MI *mi);

  static void collectArgs(MO *mo, std::vector<RegArgument> &arguments,
                          std::function<bool(char)> filter);
  static std::vector<RegArgument> collectWriteArgs(MO *mo);
  static std::vector<RegArgument> collectReadArgs(MO *mo);

  void printEdge(char32_t reg1, char32_t reg2, bool same_rf,
                 int cond_RF = -1) const;
  void printNode(char32_t reg, bool in_edge) const;
  void printEdge(std::ostream &outStr, char32_t reg1, char32_t reg2,
                 bool same_rf, int cond_RF = -1) const;
  void printNode(std::ostream &outStr, char32_t reg, bool in_edge) const;

  bool detectUsedPorts(const std::vector<RegArgument> &args,
                       unsigned int *portCounts);

  /*! \brief Increment read counts for registers in the MO.
   *
   * The RDG stores for each entry the number of read accesses. This method
   * can be used to increment the read count for the entries corresponding to
   * register read accesses in the given MO.
   *
   * \param mo The MO.
   */
  void countReadRegisters(MO *mo);

  /*! \brief Store a dependency between two registers.
   *
   * The dependency between two registers is stored as links between nodes. They
   * might not be direct links.
   *
   * \param reg1 First register of the register pair.
   * \param reg2 Second register of the register pair.
   * \param same_rf If the register have to be in the same register file.
   * \param cond_RF Dependency is only active, if specified register file is
   * used.
   */
  bool recordDependency(char32_t reg1, char32_t reg2, bool same_rf,
                        int cond_RF = -1);
  bool recordRegisterCouple(const RegisterCouple *cpl);

  void compress();
  void compress(Entry *e);

  static unsigned int maxReadPorts();
  static unsigned int maxWritePorts();

  /*! \brief Create a new RDG for a maximal number of virtual registers.
   *
   * The RDG is created with a capacity to store all fix registers plus the
   * given number of virtual registers.
   */
  RDG(unsigned int size);

public:
  virtual ~RDG();

  /*! \brief Method for building a register dependency graph for a program.
   *
   * Builds a register dependency graph for a scheduled program. Also performs
   * feasibility checks. If a register allocation for the given program is
   * detected to be impossible, no RDG is created and the method returns 0.
   */
  static RDG *analyseRegisterDependencies(Program *instructions,
                                          RegisterCoupling *couplings,
                                          VirtualRegisterMap *map,
                                          ass_reg_t *blocked,
                                          bool delOnError = true);

  int rootCount() const;
  int virtualRegCount() const;
  const Entry *getEntry(char32_t reg) const;
  const Entry *getRootEntry(char32_t reg) const;
  const Entry *getRootEntry(char32_t reg, bool &same_rf) const;
  const Entry *getEntryByIndex(unsigned int index) const;
  DEP_TYPE getDependency(char32_t reg1, char32_t reg2) const;
  unsigned int size() const { return _size; }

  void printDot() const;
  void printDot(std::ostream &outStr) const;
  int maxDepth() const;

  /**
   * \brief Get register file Allocation Vector for all register entries
   * (physical & virtual). The physical Register will be allocated to the
   * correct Register File. Only the virtual Register contain a to allocate tag
   * (-1).
   * @param chrm
   * @return int vector of register file allocations (RF0 or RF1)
   */
  int *deriveAllocations(const portOptReg::Chromosome *chrm) const;
};

} // namespace rdg

#endif /* HEADER_RDG_H_ */
