// Copyright (c) 2022 Chair for Chip Design for Embedded Computing,
//                    Technische Universitaet Braunschweig, Germany
//                    www.tu-braunschweig.de/en/eis
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT
#ifndef LABEL_H
#define LABEL_H 1

#include "SLM.h"
#include "global.h"
/** \brief Namespace for Labels.
 *
 * It would have also been possible to use a class here, but since the labels
 * are globally accessible and we would only have one getInstance, a namespace
 * was better.
 *
 * Labels represent labels in code and are bound to one SLM.
 */
namespace label {
/** \brief Write out all the label Names to stdout.
 *
 * A function, that simply iterates over all already parsed labels, with the
 * exception of ":L_NOT_SCHEDULE" and prints them to screen.
 */
void printAllLabels();
/**  \brief Adds a Label to the index.
 *
 * For each label an ID is generated and can now be accessed through
 * parseLabel()
 * @param [in] labelname The name of the label.
 * */
void addLabel(const char *labelname);

/** \brief Parses a Label and returns a Pointer to the field after the parsed
 * text.
 *
 * The labelname could be followed by any text. As soon as a labelname is
 * parsed, the resulting ID is generated and a pointer to the text after the
 * Label is returned. This function is intended to be used in parsing lines.
 * This way a line can be parsed, by just calling the parser and if they parse
 * anything, their result is stored in the ID and the return text can be passed
 * to the next function.
 *
 * @param [in] labelname The name of the label to parse.
 * @param [out] ID The ID of the label.
 * @return Pointer to the text behind the label.
 * */
char *parseLabel(char *labelname, char32_t *ID);

/** \brief returns the name of the Label.
 *
 * @param [in] ID The ID of the label where the name should be returned.
 * @return Pointer to the first char of it's name.
 */
std::string getLabelName(char32_t ID);

/** \brief register an SLM to an ID.
 *
 * @param [in] ID The ID of the Label.
 * @param [in] slm Link to an SLM.
 */
void registerSLM(char32_t ID, SLM *slm);

/** \brief Return the SLM registered to this label.
 *
 * The SLM first has to be registered using registerSLM(),
 * otherwise an error is generated and the program exists.
 *
 * @param [in] ID The ID of the Label.
 * @return Link to the registered SLM or NULL.
 */
SLM *getSLM(char32_t ID);
} // namespace label
#endif
