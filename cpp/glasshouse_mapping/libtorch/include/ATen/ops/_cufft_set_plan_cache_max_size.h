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



#include <ATen/ops/_cufft_set_plan_cache_max_size_ops.h>

namespace at {


// aten::_cufft_set_plan_cache_max_size(DeviceIndex device_index, int max_size) -> ()
inline void _cufft_set_plan_cache_max_size(at::DeviceIndex device_index, int64_t max_size) {
    return at::_ops::_cufft_set_plan_cache_max_size::call(device_index, max_size);
}

}
