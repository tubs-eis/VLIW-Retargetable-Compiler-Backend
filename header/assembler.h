// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef ASSEMBLER_H
#define ASSEMBLER_H

#include <string>
#include <vector>
class Processor;
class SLM;

class Context;

class Assembler {
private:
  /** an ASM-file which is processed */
  std::string asmFile;
  /** a configuration for a processor */
  Processor *pro;
  /** a vector with SLM's parsed from the ASM-file */
  std::vector<SLM *> slms;
  /** shows if compile has been called */
  bool scheduled;
  /** calls all the schedule functions for every slm */
  void preCompile();
  void postCompile();

  void calculateTransitionEnergy();

  // should the processor be deleted in the destructor. only in standalone
  // (library call) should the processor not be deleted!
  bool owns_processor = true;

public:
  /** \brief Initializes an Assembler with the configuration file given by the
   * filename
   *
   * the config file must be a valid XML file.
   *
   * Reset ID counter for MIs, MOs, SLMs. This was previously done in main,
   * however you could not run multiple Assembler instances without crashing
   * from a mismatch between IDs from SLMs at blockedRegsInSLM.
   *
   * @param [in] configFile the name of the configuration file.
   * @sa Pro
   *
   * cessor(char* configFile)
   */
  Assembler(char *configFile);

  Assembler(Processor *pro);
  /** \brief Destroys the assembler and cleans up. */
  ~Assembler();

  /** @brief sets the filedescriptor for a valid ASM-file
   *
   * the file is read in, comments are deletet, and all preprocessor operations
   * as defined in the config file are performed. on set all previusly
   * calculatet SLM's are destroyed
   *
   * @param [in] ASMFile the name of the ASM-file
   */
  void setASM(char *ASMFile);
  /** @brief sets ASM content to parse.
   *
   * comments are deletet, and all preprocessor operations as defined in the
   * config file are performed. on set all previusly calculatet SLM's are
   * destroyed
   *
   * @param [in] ASM a string representation of an ASM-Code
   */
  void setASM(std::string ASM);
  /** @brief returns the parsed ASM-file.
   *
   * @return returns the parsed ASM-file.
   */
  std::string getASM() { return this->asmFile; }
  /** @brief provides a way to alter the ASM-file after it has been read in.
   *
   * it is recommended to use this function to alter the ASM-file after read in,
   * because the setASM function makes use of regexReplacement and comment
   * deletion, which takes a lot of time. the function provided to this function
   * gets the complete file as one string representation as a parameter.
   */
  void asmChange(std::string (*f)(std::string)) { asmFile = (*f)(asmFile); }
  /** @brief This routine iterates over all SLM's
   *
   * The function given as a parameter does not have to be threadsafe.
   * The function is called with the SLM given in their natural order.
   */
  void preScheduling(void (*f)(SLM *, Processor *));

  void preScheduling(void (*f)(std::vector<SLM *> *, Processor *));

  void writePrecompiled();

  /** @brief Writes a binary representation of the parsed SLM's to the given
   * outputstream.
   *
   * a binary representation of the of the compiled code is written to the
   * outputstream for every SLM the shortest possible path as defined by
   * SLM.getLastShortestInstruction() is written out.
   *
   * @note
   * when called the first time, it parses the ASM-File into SLM's
   *
   * @param out an output stream. Should be a file.
   * @sa SLM
   */
  void writeOutBin(std::ostream &out);
  /** @brief Writes human readable representation of the parsed SLM's to the
   * given outputstream.
   *
   * a human readable representation of the of the compiled code is written to
   * the outputstream for every SLM the shortest possible path as defined by
   * SLM.getLastShortestInstruction() is written out.
   *
   * @note
   * when called the first time, it parses the ASM-File into SLM's
   *
   * @param out an output stream. Could be std::out.
   * @sa SLM
   */
  void writeOutReadable(std::ostream &out, bool printEnergy = false);
  /** @brief Writes a dot representation of the parsed SLM's to the given
   * outputstream.
   *
   * A Graph representation is written out to the outputstream.
   * Every instruction is a node and the dependencies between them are
   * represented as directed edges.
   *
   * @note
   * when called the first time, it parses the ASM-File into SLM's
   *
   * @param out an output stream. Could be std::out.
   * @sa SLM
   */
  void writeOutDot(std::ostream &out);

  void writeOutDotDetailed(std::ostream &out);

  /** \brief Compiles and optimizes a SLM.
   *
   * This function can be used in two ways. the first one is by giving it a
   * number between 0 and getSlmCount(). This way, the SLM with the given number
   * gets compiled. The second way is by calling it without a parameter (or a
   * negative one). Thix way, all SLM's are compiled and there is no need to
   * iterate over getSlmCount();
   *
   * @param number of the SLM.
   */
  void compileSLM(int id = -1);

  /** \brief returns the number of SLM's parsed by the Assembler
   *
   * @return Number of SLM's
   */
  int getSlmCount() {
    preCompile();
    return slms.size();
  }

  std::vector<SLM *> &getSLMS() { return slms; }

  void writeOutTransitionPowerEstimateCSV(std::ostream &out);
  void writeOutTransitionPowerEstimate(std::ostream &out);

  void writeOutCompilableAssembler(std::ostream &out) const;

  void writeOutInstructionTransitions(std::ostream &out) const;
  void writeCsvOutput(SLM *slm, bool error = false) const;
};

/***
 * Compile a SLM with the processor as configuration.
 * 1) calculate a Graph (for Dependency Analysis)
 * 2 a) Do a heuristic scheduling followed by
 * @param slm
 * @param pro
 * @param ctx
 * @return
 */
int simpleCompile(SLM *slm, Processor *pro, const Context &ctx);

#endif // ASSEMBLER_H
