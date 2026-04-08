/*
 * Copyright (c) 2024-2025 Ziqi Fan
 * SPDX-License-Identifier: Apache-2.0
 */

#include "inference_runtime.hpp"

#include <algorithm>
#include <filesystem>
#include <stdexcept>

namespace InferenceRuntime
{

ONNXModel::ONNXModel()
#ifdef USE_ONNX
    : memory_info_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
#endif
{
#ifdef USE_ONNX
    env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "ONNXModel");
#endif
}

ONNXModel::~ONNXModel()
{
#ifdef USE_ONNX
    session_.reset();
    env_.reset();
#endif
}

bool ONNXModel::load(const std::string& model_path)
{
    try
    {
#ifdef USE_ONNX
        Ort::SessionOptions session_options;
        session_options.SetIntraOpNumThreads(1);
        session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_EXTENDED);

        session_ = std::make_unique<Ort::Session>(*env_, model_path.c_str(), session_options);
        setup_input_output_info();

        model_path_ = model_path;
        loaded_ = true;
        return true;
#else
        loaded_ = false;
        return false;
#endif
    }
    catch (const std::exception& e)
    {
        loaded_ = false;
        throw std::runtime_error(std::string("Failed to load ONNX model '") + model_path + "': " + e.what());
    }
}

std::vector<float> ONNXModel::forward(const std::vector<std::vector<float>>& inputs)
{
    if (!loaded_)
    {
        throw std::runtime_error("Model not loaded");
    }

#ifdef USE_ONNX
    try
    {
        Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

        const auto& input = inputs[0];
        const auto& input_shape = input_shapes_.at(0);

        auto input_tensor = Ort::Value::CreateTensor<float>(
            memory_info,
            const_cast<float*>(input.data()),
            input.size(),
            input_shape.data(),
            input_shape.size()
        );

        const char* input_names[] = {input_node_names_[0].c_str()};
        const char* output_names[] = {output_node_names_[0].c_str()};

        auto outputs = session_->Run(
            Ort::RunOptions{nullptr},
            input_names,
            &input_tensor,
            1,
            output_names,
            1
        );

        return extract_output_data(outputs);
    }
    catch (const std::exception& e)
    {
        throw std::runtime_error(std::string("ONNX inference error: ") + e.what());
    }
#else
    throw std::runtime_error("ONNX support not compiled");
#endif
}

#ifdef USE_ONNX
void ONNXModel::setup_input_output_info()
{
    const size_t num_input_nodes = session_->GetInputCount();
    input_node_names_.clear();
    input_shapes_.clear();
    input_node_names_.reserve(num_input_nodes);
    input_shapes_.reserve(num_input_nodes);
    Ort::AllocatorWithDefaultOptions allocator;

    for (size_t i = 0; i < num_input_nodes; ++i)
    {
        auto input_name = session_->GetInputNameAllocated(i, allocator);
        input_node_names_.push_back(std::string(input_name.get()));

        auto input_type_info = session_->GetInputTypeInfo(i);
        auto input_tensor_info = input_type_info.GetTensorTypeAndShapeInfo();
        auto input_dims = input_tensor_info.GetShape();

        std::vector<int64_t> shape;
        for (auto dim : input_dims)
        {
            shape.push_back(dim == -1 ? 1 : dim);
        }
        input_shapes_.push_back(shape);
    }

    const size_t num_output_nodes = session_->GetOutputCount();
    output_node_names_.clear();
    output_shapes_.clear();
    output_node_names_.reserve(num_output_nodes);
    output_shapes_.reserve(num_output_nodes);

    for (size_t i = 0; i < num_output_nodes; ++i)
    {
        auto output_name = session_->GetOutputNameAllocated(i, allocator);
        output_node_names_.push_back(std::string(output_name.get()));

        auto output_type_info = session_->GetOutputTypeInfo(i);
        auto output_tensor_info = output_type_info.GetTensorTypeAndShapeInfo();
        auto output_dims = output_tensor_info.GetShape();

        std::vector<int64_t> shape;
        for (auto dim : output_dims)
        {
            shape.push_back(dim == -1 ? 1 : dim);
        }
        output_shapes_.push_back(shape);
    }
}

std::vector<float> ONNXModel::extract_output_data(const std::vector<Ort::Value>& outputs)
{
    if (outputs.empty())
    {
        throw std::runtime_error("No outputs from ONNX model");
    }

    auto& output = outputs[0];
    float* output_data = const_cast<float*>(output.GetTensorData<float>());
    auto output_shape = output.GetTensorTypeAndShapeInfo().GetShape();

    int64_t num_elements = 1;
    for (auto dim : output_shape)
    {
        if (dim > 0)
        {
            num_elements *= dim;
        }
    }

    return std::vector<float>(output_data, output_data + num_elements);
}
#endif

std::unique_ptr<Model> ModelFactory::create_model(ModelType)
{
    return std::make_unique<ONNXModel>();
}

ModelFactory::ModelType ModelFactory::detect_model_type(const std::string& model_path)
{
    std::filesystem::path path(model_path);
    std::string extension = path.extension().string();
    std::transform(extension.begin(), extension.end(), extension.begin(), ::tolower);

    if (extension == ".onnx")
    {
        return ModelType::ONNX;
    }

    throw std::runtime_error("Unsupported model file extension: " + extension + ". This build only accepts .onnx policies.");
}

std::unique_ptr<Model> ModelFactory::load_model(const std::string& model_path, ModelType type)
{
    if (type == ModelType::AUTO)
    {
        type = detect_model_type(model_path);
    }

    auto model = create_model(type);
    if (model && model->load(model_path))
    {
        return model;
    }
    return nullptr;
}

} // namespace InferenceRuntime
