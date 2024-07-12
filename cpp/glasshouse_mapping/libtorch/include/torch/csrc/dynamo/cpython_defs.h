#pragma once

#include <torch/csrc/utils/python_compat.h>

// Functions that need to be copied from the CPython source
// should go in cpython_defs.c. Copying is required when, e.g.,
// we need to call internal CPython functions that are not exposed.

#if IS_PYTHON_3_11_PLUS

#include <internal/pycore_frame.h>

int THP_PyFrame_FastToLocalsWithError(
    _PyInterpreterFrame* frame,
    int* free_vars_copied);

PyFunctionObject* _PyFunction_CopyWithNewCode(
    PyFunctionObject* o,
    PyCodeObject* code);

void THP_PyFrame_Clear(_PyInterpreterFrame* frame);

#endif
