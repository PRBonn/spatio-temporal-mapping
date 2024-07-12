#pragma once

// @generated by torchgen/gen.py from Function.h

#include <ATen/Context.h>
#include <ATen/DeviceGuard.h>
#include <ATen/TensorUtils.h>
#include <ATen/TracerMode.h>
#include <ATen/core/Generator.h>
#include <ATen/core/Reduction.h>
#include <ATen/core/Tensor.h>
#include <c10/core/Scalar.h>
#include <c10/core/Storage.h>
#include <c10/core/TensorOptions.h>
#include <c10/util/Deprecated.h>
#include <c10/util/Optional.h>



#include <ATen/ops/fbgemm_linear_quantize_weight_ops.h>

namespace at {


// aten::fbgemm_linear_quantize_weight(Tensor input) -> (Tensor, Tensor, float, int)
inline ::std::tuple<at::Tensor,at::Tensor,double,int64_t> fbgemm_linear_quantize_weight(const at::Tensor & input) {
    return at::_ops::fbgemm_linear_quantize_weight::call(input);
}

}
