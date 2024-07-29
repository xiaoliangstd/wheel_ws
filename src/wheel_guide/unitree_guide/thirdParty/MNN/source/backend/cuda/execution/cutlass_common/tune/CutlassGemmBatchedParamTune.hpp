#ifdef ENABLE_CUDA_TUNE_PARAM

#include "../../CutlassGemmParam.hpp"
#include "cutlass/gemm/device/gemm_batched.h"

namespace MNN {
namespace CUDA {
using BatchedSwizzleThreadBlock = cutlass::gemm::threadblock::GemmBatchedIdentityThreadblockSwizzle;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Row_Sm80_64x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 64>,
    cutlass::gemm::GemmShape<32, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Row_Sm80_128x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 64>,
    cutlass::gemm::GemmShape<64, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    2>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Row_Sm80_64x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<32, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    6>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Row_Sm80_128x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 32>,
    cutlass::gemm::GemmShape<64, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Row_Sm80_64x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 128, 32>,
    cutlass::gemm::GemmShape<32, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Row_Sm80_256x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<256, 64, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Row_Sm80_128x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 128, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Column_Sm80_64x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 64>,
    cutlass::gemm::GemmShape<32, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Column_Sm80_128x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 64>,
    cutlass::gemm::GemmShape<64, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    2>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Column_Sm80_64x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<32, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    6>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Column_Sm80_128x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 32>,
    cutlass::gemm::GemmShape<64, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Column_Sm80_64x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 128, 32>,
    cutlass::gemm::GemmShape<32, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Column_Sm80_256x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<256, 64, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignCuda_Row_Column_Sm80_128x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 128, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Row_Sm80_64x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 64>,
    cutlass::gemm::GemmShape<32, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Row_Sm80_128x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 64>,
    cutlass::gemm::GemmShape<64, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    2>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Row_Sm80_64x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<32, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    6>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Row_Sm80_128x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 32>,
    cutlass::gemm::GemmShape<64, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Row_Sm80_64x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 128, 32>,
    cutlass::gemm::GemmShape<32, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Row_Sm80_256x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<256, 64, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Row_Sm80_128x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 128, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Column_Sm80_64x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 64>,
    cutlass::gemm::GemmShape<32, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Column_Sm80_128x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 64>,
    cutlass::gemm::GemmShape<64, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    2>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Column_Sm80_64x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<32, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    6>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Column_Sm80_128x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 32>,
    cutlass::gemm::GemmShape<64, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Column_Sm80_64x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 128, 32>,
    cutlass::gemm::GemmShape<32, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Column_Sm80_256x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<256, 64, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F16_Linear_AlignTensor_Row_Column_Sm80_128x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    cutlass::half_t,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 128, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F16_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Row_Sm80_64x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 64>,
    cutlass::gemm::GemmShape<32, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Row_Sm80_128x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 64>,
    cutlass::gemm::GemmShape<64, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    2>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Row_Sm80_64x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<32, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    6>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Row_Sm80_128x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 32>,
    cutlass::gemm::GemmShape<64, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Row_Sm80_64x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 128, 32>,
    cutlass::gemm::GemmShape<32, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Row_Sm80_256x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<256, 64, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Row_Sm80_128x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 128, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Column_Sm80_64x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 64>,
    cutlass::gemm::GemmShape<32, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Column_Sm80_128x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 64>,
    cutlass::gemm::GemmShape<64, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    2>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Column_Sm80_64x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<32, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    6>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Column_Sm80_128x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 32>,
    cutlass::gemm::GemmShape<64, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Column_Sm80_64x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 128, 32>,
    cutlass::gemm::GemmShape<32, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Column_Sm80_256x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<256, 64, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignCuda_Row_Column_Sm80_128x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 128, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueCudaOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Row_Sm80_64x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 64>,
    cutlass::gemm::GemmShape<32, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Row_Sm80_128x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 64>,
    cutlass::gemm::GemmShape<64, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    2>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Row_Sm80_64x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<32, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    6>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Row_Sm80_128x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 32>,
    cutlass::gemm::GemmShape<64, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Row_Sm80_64x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 128, 32>,
    cutlass::gemm::GemmShape<32, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Row_Sm80_256x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<256, 64, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Row_Sm80_128x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::RowMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 128, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Column_Sm80_64x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 64>,
    cutlass::gemm::GemmShape<32, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Column_Sm80_128x64x64 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 64>,
    cutlass::gemm::GemmShape<64, 32, 64>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    2>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Column_Sm80_64x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<32, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    6>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Column_Sm80_128x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 64, 32>,
    cutlass::gemm::GemmShape<64, 32, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Column_Sm80_64x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<64, 128, 32>,
    cutlass::gemm::GemmShape<32, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    4>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Column_Sm80_256x64x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<256, 64, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

using GemmBatchedTensor_F16_F32_Linear_AlignTensor_Row_Column_Sm80_128x128x32 = cutlass::gemm::device::GemmBatched<
    cutlass::half_t,
    cutlass::layout::RowMajor,
    cutlass::half_t,
    cutlass::layout::ColumnMajor,
    float,
    LayoutOutput,
    ElementAccumulator,
    cutlass::arch::OpClassTensorOp,
    cutlass::arch::Sm80,
    cutlass::gemm::GemmShape<128, 128, 32>,
    cutlass::gemm::GemmShape<64, 64, 32>,
    cutlass::gemm::GemmShape<16, 8, 16>,
    EpilogueTensorOp_F32_Linear,
    BatchedSwizzleThreadBlock,
    3>;

}
}
#endif