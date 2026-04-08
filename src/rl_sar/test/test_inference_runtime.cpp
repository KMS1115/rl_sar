/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inference_runtime.hpp"

#ifdef USE_ONNX
#include <onnxruntime_cxx_api.h>
#endif

#include <chrono>
#include <iostream>
#include <string>
#include <vector>

struct TestResult
{
    std::string name;
    bool ok = false;
    double avg_ms = 0.0;
    size_t output_size = 0;
};

static void print_result(const TestResult& result)
{
    std::cout << result.name << ": "
              << (result.ok ? "OK" : "FAILED")
              << ", avg_ms=" << result.avg_ms
              << ", output_size=" << result.output_size
              << std::endl;
}

#ifdef USE_ONNX
static TestResult test_onnx_direct(const std::string& model_path, const std::vector<float>& input, int iterations)
{
    TestResult result;
    result.name = "ONNX direct";

    Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "onnx_test");
    Ort::SessionOptions session_options;
    session_options.SetIntraOpNumThreads(1);
    session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);
    Ort::Session session(env, model_path.c_str(), session_options);

    auto input_shape = session.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
    for (auto& dim : input_shape)
    {
        if (dim == -1)
        {
            dim = 1;
        }
    }

    auto input_name = session.GetInputNameAllocated(0, Ort::AllocatorWithDefaultOptions());
    auto output_name = session.GetOutputNameAllocated(0, Ort::AllocatorWithDefaultOptions());
    const char* input_names[] = {input_name.get()};
    const char* output_names[] = {output_name.get()};

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

    double total_ms = 0.0;
    for (int i = 0; i < iterations; ++i)
    {
        auto input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            const_cast<float*>(input.data()),
            input.size(),
            input_shape.data(),
            input_shape.size()
        );

        auto start = std::chrono::steady_clock::now();
        auto outputs = session.Run(Ort::RunOptions{nullptr}, input_names, &input_tensor, 1, output_names, 1);
        auto end = std::chrono::steady_clock::now();

        total_ms += std::chrono::duration<double, std::milli>(end - start).count();
        result.output_size = outputs[0].GetTensorTypeAndShapeInfo().GetElementCount();
    }

    result.avg_ms = total_ms / iterations;
    result.ok = true;
    return result;
}

static TestResult test_onnx_abstraction(const std::string& model_path, const std::vector<float>& input, int iterations)
{
    TestResult result;
    result.name = "ONNX inference_runtime";

    auto model = InferenceRuntime::ModelFactory::load_model(model_path);
    if (!model)
    {
        return result;
    }

    double total_ms = 0.0;
    for (int i = 0; i < iterations; ++i)
    {
        auto start = std::chrono::steady_clock::now();
        auto output = model->forward({input});
        auto end = std::chrono::steady_clock::now();

        total_ms += std::chrono::duration<double, std::milli>(end - start).count();
        result.output_size = output.size();
    }

    result.avg_ms = total_ms / iterations;
    result.ok = true;
    return result;
}
#endif

int main(int argc, char** argv)
{
    if (argc < 5)
    {
        std::cout << "Usage: " << argv[0] << " <model_path.onnx> <onnx> <use_abstraction> <input_size> [iterations]" << std::endl;
        std::cout << "Example: " << argv[0] << " policy/go2/robot_lab/policy.onnx onnx 1 45 100" << std::endl;
        return 1;
    }

    std::string model_path = argv[1];
    std::string framework = argv[2];
    bool use_abstraction = (std::stoi(argv[3]) != 0);
    int input_size = std::stoi(argv[4]);
    int iterations = (argc >= 6) ? std::stoi(argv[5]) : 30;

    if (framework != "onnx")
    {
        std::cout << "ERROR: this build only supports 'onnx'" << std::endl;
        return 1;
    }

#ifndef USE_ONNX
    std::cout << "ERROR: ONNX support not compiled in this build" << std::endl;
    return 1;
#else
    std::vector<float> input(input_size, 0.1f);
    TestResult result = use_abstraction
        ? test_onnx_abstraction(model_path, input, iterations)
        : test_onnx_direct(model_path, input, iterations);
    print_result(result);
    return result.ok ? 0 : 1;
#endif
}
