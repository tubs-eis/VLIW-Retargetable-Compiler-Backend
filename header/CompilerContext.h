// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT

#ifndef HEADER_COMPILERCONTEXT_H_
#define HEADER_COMPILERCONTEXT_H_

#include <ostream>
#include <sstream>
#include <string>

/**
 * The context stores information about the current state of the merger,
 * scheduler, and register allocator. This includes the number of the generation
 * during the genetic algorithms, as well as the number of the currently
 * processed individual.
 */
class Context {
private:
  int _mergeRound, _mergeIndiv;
  int _schedRound, _schedIndiv;
  int _regRound, _regIndiv;
  std::string _stage;
  Context(int mergeRound, int mergeIndiv, int schedRound, int schedIndiv,
          int regRound, int regIndiv, const std::string &stage)
      : _mergeRound(mergeRound), _mergeIndiv(mergeIndiv),
        _schedRound(schedRound), _schedIndiv(schedIndiv), _regRound(regRound),
        _regIndiv(regIndiv), _stage(stage) {}

public:
  static const Context EMPTY;

  static Context NewMergeContext(int mergeRound, int mergeIndiv,
                                 const std::string &stage = "") {
    return Context(mergeRound, mergeIndiv, -1, -1, -1, -1, stage);
  }

  static Context NewSchedContext(int schedRound, int schedIndiv,
                                 const std::string &stage = "") {
    return Context(-1, -1, schedRound, schedIndiv, -1, -1, stage);
  }

  static Context Merge(const Context &ctx, int mergeRound, int mergeIndiv,
                       const std::string &stage = "") {
    Context c(ctx);
    c.merge(mergeRound, mergeIndiv);
    if (!stage.empty())
      c.appendStage(stage);
    return c;
  }

  static Context Schedule(const Context &ctx, int schedRound, int schedIndiv,
                          const std::string &stage = "") {
    Context c(ctx);
    c.schedule(schedRound, schedIndiv);
    if (!stage.empty())
      c.appendStage(stage);
    return c;
  }

  static Context RegisterAlloc(const Context &ctx, int regRound, int regIndiv,
                               const std::string &stage = "") {
    Context c(ctx);
    c.regalloc(regRound, regIndiv);
    if (!stage.empty())
      c.appendStage(stage);
    return c;
  }

  /**
   * Creates a Context for logging purposes.
   * @param ctx
   * @return
   */
  static Context EnergyAlloc(const Context &ctx) {
    Context c(ctx);

    return c;
  }

  static Context nextStage(const Context &ctx, const std::string &stage) {
    Context c(ctx);
    c.appendStage(stage);
    return c;
  }

  explicit Context(const std::string &stage)
      : Context(-1, -1, -1, -1, -1, -1, stage) {}

  Context(const Context &ctx) {
    _mergeRound = ctx.mergeRound();
    _mergeIndiv = ctx.mergeIndiv();
    _schedRound = ctx.schedRound();
    _schedIndiv = ctx.schedIndiv();
    _regRound = ctx.regRound();
    _regIndiv = ctx.regIndiv();
    _stage = ctx.stage();
  }

  Context &operator=(const Context &ctx) {
    _mergeRound = ctx.mergeRound();
    _mergeIndiv = ctx.mergeIndiv();
    _schedRound = ctx.schedRound();
    _schedIndiv = ctx.schedIndiv();
    _regRound = ctx.regRound();
    _regIndiv = ctx.regIndiv();
    _stage = ctx.stage();
    return *this;
  }

  virtual ~Context() {}

  int schedRound() const { return _schedRound; }

  int schedIndiv() const { return _schedIndiv; }

  int mergeRound() const { return _mergeRound; }

  int mergeIndiv() const { return _mergeIndiv; }

  int regRound() const { return _regRound; }

  int regIndiv() const { return _regIndiv; }

  const std::string &stage() const { return _stage; }

  Context &merge(int mergeRound, int mergeIndiv) {
    _mergeRound = mergeRound;
    _mergeIndiv = mergeIndiv;
    return *this;
  }

  Context &schedule(int schedRound, int schedIndiv) {
    _schedRound = schedRound;
    _schedIndiv = schedIndiv;
    return *this;
  }

  Context &regalloc(int regRound, int regIndiv) {
    _regRound = regRound;
    _regIndiv = regIndiv;
    return *this;
  }

  Context &appendStage(const std::string &stage) {
    _stage += " " + stage;
    return *this;
  }

  std::string asString() const {
    std::stringstream sstr;
    sstr << "[";
    if (!_stage.empty()) {
      sstr << _stage;
      if (mergeRound() != -1 || schedRound() != -1 || regRound() != -1)
        sstr << " | ";
    }
    if (mergeRound() != -1) {
      sstr << "M " << mergeIndiv() << " @" << mergeRound();
      if (schedRound() != -1)
        sstr << ", ";
    }
    if (schedRound() != -1) {
      sstr << "S " << schedIndiv() << " @" << schedRound();
      if (regRound() != -1)
        sstr << ", ";
    }
    if (regRound() != -1) {
      sstr << "R " << regIndiv() << " @" << regRound();
    }
    sstr << "]";
    return sstr.str();
  }
};

inline std::ostream &operator<<(std::ostream &os, const Context &ctx) {
  os << ctx.asString();
  return os;
}

#endif /* HEADER_COMPILERCONTEXT_H_ */
