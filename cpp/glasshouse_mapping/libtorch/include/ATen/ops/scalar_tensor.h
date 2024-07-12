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



#include <ATen/ops/scalar_tensor_ops.h>

namespace at {


// aten::scalar_tensor(Scalar s, *, ScalarType? dtype=None, Layout? layout=None, Device? device=None, bool? pin_memory=None) -> Tensor
inline at::Tensor scalar_tensor(const at::Scalar & s, at::TensorOptions options={}) {
    return at::_ops::scalar_tensor::call(s, c10::optTypeMetaToScalarType(options.dtype_opt()), options.layout_opt(), options.device_opt(), options.pinned_memory_opt());
}
// aten::scalar_tensor(Scalar s, *, ScalarType? dtype=None, Layout? layout=None, Device? device=None, bool? pin_memory=None) -> Tensor
inline at::Tensor scalar_tensor(const at::Scalar & s, c10::optional<at::ScalarType> dtype, c10::optional<at::Layout> layout, c10::optional<at::Device> device, c10::optional<bool> pin_memory) {
    return at::_ops::scalar_tensor::call(s, dtype, layout, device, pin_memory);
}

// aten::scalar_tensor.out(Scalar s, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & scalar_tensor_out(at::Tensor & out, const at::Scalar & s) {
    return at::_ops::scalar_tensor_out::call(s, out);
}
// aten::scalar_tensor.out(Scalar s, *, Tensor(a!) out) -> Tensor(a!)
inline at::Tensor & scalar_tensor_outf(const at::Scalar & s, at::Tensor & out) {
    return at::_ops::scalar_tensor_out::call(s, out);
}

}
